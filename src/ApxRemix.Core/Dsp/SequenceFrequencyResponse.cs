using System.Numerics;
using System.Runtime.InteropServices;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed record SequenceResponseCurve(double[] FrequencyHz, double[] Level);

/// <summary>Sequence Frequency Response 单路、同一采样率、无发生器 EQ 的 ChirpParams.FreqResponse 路径。
/// 先用 CreateExcitation 取得单位峰值扫频，按实际输出增益送给被测件，再把采回的 PCM 交给 Analyze。</summary>
public sealed class SequenceFrequencyResponse
{
    private readonly int rate;
    private readonly double startHz, stopHz, tailHz, duration, preTime;
    private readonly bool digital;
    private readonly Complex[] inverse, correction;
    public int AcquisitionLength { get; }
    public int FftLength { get; }

    // 固定采样率、扫频时长和起止频率，预先设计同一组采集可复用的逆滤波器与幅值校正。
    public SequenceFrequencyResponse(int sampleRate, double startHz, double stopHz, double sweepSeconds,
        double preSeconds, double extraSeconds, bool digital = true)
    {
        // 扫频必须递增且终点不超过每秒采样次数的一半；前后预留时间均以秒计。
        if (sampleRate <= 0 || !(startHz > 0 && startHz < stopHz && stopHz <= sampleRate / 2.0))
            throw new ArgumentException("起止频率须位于 0 到奈奎斯特频率之间且递增。");
        if (!(sweepSeconds > 0 && preSeconds >= 0 && extraSeconds >= 0)) throw new ArgumentOutOfRangeException(nameof(sweepSeconds));
        rate = sampleRate;
        this.startHz = startHz;
        this.stopHz = stopHz;
        this.digital = digital;
        duration = sweepSeconds;
        preTime = preSeconds;
        // 尾频比报告上限最多高一成，总时长换成点数；根据记录可用时长推算可容纳的谐波阶次。
        tailHz = Math.Min(stopHz * 1.1, rate / 2.0);
        AcquisitionLength = checked((int)Math.Round((duration + preTime + extraSeconds) * rate, MidpointRounding.AwayFromZero));
        int harmonics = (int)Math.Clamp(Math.Floor(Math.Pow(tailHz / startHz, AcquisitionLength / (2 * duration * rate)) - 0.25), 2, 20);
        // 外部播放的扫频止于配置带宽；内部用于反卷积的参考还覆盖谐波可能出现的频带。
        // 两者的扫速相同，因此可在同一时间坐标下生成它们。
        // 又要覆盖谐波又不能越过奈奎斯特，以此确定参考扫频的平段和尾段。
        double end = Math.Min(rate / 2.0, tailHz * harmonics);
        double flatEnd = Math.Max(stopHz, Math.Min(rate / 2.2, stopHz * harmonics));
        // 延长内部参考的输出区间，保留激励载波本身的 T3 和 F3。
        var reference = ContinuousSweepAnalyzer.GenerateApChirpSignal(-preTime, duration, startHz, tailHz,
            rate, FrequencyToTime(end), FrequencyToTime(flatEnd), out _);
        // 变换长度要装得下参考和采集记录，还需在中心以前留出 50 ms 供找脉冲。
        int n = SequenceTransform.NextLength(Math.Max(reference.Length, AcquisitionLength), allowShort: true);
        if (n / 2 < (int)(rate * 0.05)) throw new ArgumentException("扫频帧无法容纳时间零点之前的 50 ms。");
        FftLength = n;
        // 分别变换参考扫频和实际激励，二者共同确定反卷积系数及第二次变换后的响应校正。
        var referenceFft = Transform(reference, n);
        var driveFft = Transform(CreateExcitation(), n);
        SequenceBinEstimator.DesignResponse(n,rate,startHz,stopHz,digital,referenceFft,driveFft,out inverse,out correction);
    }

    // 对数扫频中频率比的对数随时间均匀增长；反解即得到达目标 Hz 的时刻。
    private double FrequencyToTime(double hz) =>
        duration / Math.Log(tailHz / startHz) * Math.Log(hz / startHz);

    // 生成要实际播放的扫频，包含预留前段，并从报告上限对应时刻起渐弱。
    public double[] CreateExcitation() => ContinuousSweepAnalyzer.GenerateApChirpSignal(
        -preTime, duration, startHz, tailHz, rate,
        Math.Max(duration,FrequencyToTime(tailHz)), FrequencyToTime(stopHz), out _);

    /// <summary>Locate the acquisition on the ADC sample clock. A synchronized DAC monitor,
    /// when available, preserves DUT delay. Otherwise use the measured tracking channel.</summary>
    public int LocateAcquisition(ReadOnlySpan<double> drive, ReadOnlySpan<double> left, ReadOnlySpan<double> right = default,
        bool trackSecond = false, ReadOnlySpan<double> synchronousOutput = default)
    {
        // 确认选中的跟踪声道存在且两路共用样本时间轴，供两路共享同一个定位结果。
        if (trackSecond && right.IsEmpty)
            throw new ArgumentException("The selected tracking input Ch2 is unavailable.");
        if (!right.IsEmpty && right.Length != left.Length)
            throw new ArgumentException("The two acquired channels have different lengths.");
        var trackingInput = trackSecond ? right : left;
        if (drive.IsEmpty || trackingInput.Length < AcquisitionLength)
            throw new ArgumentException("The recording is too short for the configured chirp and response tail.");
        // 若有同步输出副本，优先用该信号定位；没有时使用指定输入声道。
        var source = synchronousOutput.IsEmpty ? trackingInput : synchronousOutput;
        if (source.Length != trackingInput.Length)
            throw new ArgumentException("The synchronous output and recording must share the same sample clock and start.");
        int offset = -1;
        // 先寻找最多 64 点的精确前缀，再核对整段激励，以排除相同短前缀。
        if (!synchronousOutput.IsEmpty)
        {
            offset = source.IndexOf(drive[..Math.Min(64, drive.Length)]);
            if (offset >= 0 && !source[offset..].StartsWith(drive)) offset = -1;
        }
        // 精确匹配失败时改做相关搜索，用候选延时截出的整段记录验证此次定位。
        if (offset < 0)
        {
            offset = SweepAlignment.FindDelay(drive, source);
            if (offset >= 0 && offset <= source.Length - drive.Length)
            {
                // Reject a chance peak in unrelated noise. At the detected lag, compare
                // the signed dot product with eight standard deviations of uncorrelated samples.
                // 同时累加乘积、平方和与样本和，扣除均值贡献后的相关性不受两段直流偏置影响。
                double xy = 0, xx = 0, yy = 0, sumX = 0, sumY = 0;
                for (int i = 0; i < drive.Length; i++)
                {
                    double x = drive[i], y = source[offset+i];
                    xy += x*y; xx += x*x; yy += y*y; sumX += x; sumY += y;
                }
                xy -= sumX*sumY / drive.Length;
                xx -= sumX*sumX / drive.Length;
                yy -= sumY*sumY / drive.Length;
                // 相关系数取平方，允许极性倒转。能量为零的记录无法判定延时。
                if (!(xy*xy*drive.Length > 64 * xx*yy) || !(xx > 0 && yy > 0))
                    throw new ArgumentException("No correlated chirp was found in the recording.");
            }
        }
        // 找到激励起点后，后面还要有足够点数覆盖响应尾段。
        if (offset < 0 || offset > trackingInput.Length - AcquisitionLength)
            throw new ArgumentException("The recording does not contain the full aligned chirp and response tail.");
        return offset;
    }

    // 反卷积后先在时域定位并圈住脉冲，再以复频谱校正算出各频格的线性电平。
    public SequenceResponseCurve Analyze(ReadOnlySpan<double> acquired)
    {
        if (acquired.IsEmpty) throw new ArgumentOutOfRangeException(nameof(acquired));
        var impulse = Deconvolve(acquired);
        // 原版从居中记录连续截取零点前 50 ms 至记录末尾，再围绕实测峰值定窗。
        // 从时间零点以前 50 ms 搜索至可用尾部，取绝对值最大点，便于识别正脉冲和负脉冲。
        int before = (int)(rate * 0.05);
        int available = before + FftLength / 2;
        int impulseStart = FftLength / 2 - before;
        int peak = 0;
        double max = -1;
        for (int i = 0; i < available; i++)
        {
            double v = Math.Abs(impulse[impulseStart + i]);
            if (v > max) { max = v; peak = i; }
        }
        // 门长由校正谱决定；门居中遇到可用记录边缘时移回边界，数据太少则留零补齐。
        int length = 2 * (correction.Length - 1);
        int start = Math.Clamp(peak - length / 2, 0, Math.Max(0, available - length));
        int used = Math.Min(available, length);
        var window = CreateImpulseWindow(used,peak - start);
        var gated = new double[length];
        for (int i = 0; i < used; i++)
            gated[i] = impulse[impulseStart + start + i] * window[i];
        // 门内样本变换后，可在工作端完成复数校正及取模，也可在输出循环本地计算。
        var spectrum = Transform(gated, length);
        var magnitudes = SequenceBinEstimator.CorrectedMagnitudes(spectrum, correction);
        double df = (double)rate / length;
        double dt = 1.0 / rate;
        // 报告的首格向下取整、末格向上取整，再夹在正频率范围内，每格间隔 Fs/length Hz。
        int first = Math.Max(1, (int)Math.Floor(startHz * dt * length));
        int last = Math.Min(length / 2, (int)Math.Ceiling(stopHz * dt * length));
        var hz = new double[last - first + 1];
        var level = new double[hz.Length];
        for (int i = 0; i < hz.Length; i++)
        {
            int k = first + i;
            hz[i] = k * df;
            level[i] = magnitudes?[k] ?? (spectrum[k] * correction[k]).Magnitude;
        }
        return new(hz, level);
    }

    // 按主峰到两端的距离分别定渐入、渐出长度；中间平段保持系数 1，右端没空间就跳过。
    private static double[] CreateImpulseWindow(int used, int peak)
    {
        int left = (int)Math.Floor(peak * 0.9);
        int right = (int)Math.Floor((used - peak) * 0.95);
        var window = new double[used];
        Array.Fill(window, 1, left, used - left - right);
        Taper(left, -Math.PI, 0).CopyTo(window, 0);
        if (right > 0) Taper(right, 0, Math.PI * (1 - 2.0 / right)).CopyTo(window, used - right);
        return window;
    }

    // 按角度采样等波纹门边缘；零点数返回空段，有工作端时直接取其结果。
    private static double[] Taper(int count, double start, double stop)
    {
        if (count == 0) return [];
        var native = SequenceBinEstimator.EquirippleTaper(count,start,stop);
        if (native != null) return native;
        // 本地从 start 起以 (stop−start)/count 为步长逐点求窗，因此最后一个角度早于 stop。
        double step = (stop - start) / count;
        var coefficients = new double[count];
        for (int i = 0; i < count; i++)
            coefficients[i] = FftEngine.EvaluateApEquiripple(Math.Cos(i * step + start));
        return coefficients;
    }

    // 将时域记录截断或补零到设计长度，乘参考逆频响以解除扫频造成的时间展宽。
    private double[] Deconvolve(ReadOnlySpan<double> source)
    {
        var data = new double[FftLength];
        source[..Math.Min(source.Length,FftLength)].CopyTo(data);
        // 工作端可完成整段反卷积；回退时逐格相乘，内部正频格补上负频共轭，逆变换后取实部。
        var native = SequenceBinEstimator.Deconvolve(data,inverse);
        if (native != null) return native;
        var bins = Transform(data, FftLength);
        var full = new Complex[FftLength];
        for (int k = 0; k < bins.Length; k++)
        {
            full[k] = bins[k] * inverse[k];
            if (k > 0 && k < FftLength / 2) full[FftLength-k] = Complex.Conjugate(full[k]);
        }
        SequenceTransform.Inverse(full);
        return Array.ConvertAll(full,z => z.Real);
    }

    // 把实样本截断或补零后做正变换；原生实 FFT 不可用时改做完整复变换。
    private static Complex[] Transform(ReadOnlySpan<double> source, int length)
    {
        var samples = new double[length];
        source[..Math.Min(source.Length,length)].CopyTo(samples);
        var native = SequenceBinEstimator.ForwardReal(samples);
        if (native != null) return MemoryMarshal.Cast<double,Complex>(native).ToArray();
        var full = Array.ConvertAll(samples,x => new Complex(x,0));
        SequenceTransform.Forward(full);
        // 两端是自共轭格，虚部设零；其余负频率由共轭关系确定，输出只保留半谱。
        full[0] = new Complex(full[0].Real,0);
        full[length / 2] = new Complex(full[length / 2].Real,0);
        return full[..(length / 2 + 1)];
    }
}
