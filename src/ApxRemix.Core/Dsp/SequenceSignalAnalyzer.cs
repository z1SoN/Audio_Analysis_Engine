using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public enum SequenceAverage { Power, Synchronous, MaxHold, MinHold }
public sealed record SequenceSpectrum(double[] FrequencyHz, double[] TimeSeconds, double[] Waveform,
    double[] Spectrum, double[] PowerSpectralDensity, double[] AmplitudeSpectralDensity,
    Complex[] WindowedFft, double PeakCorrection, double TimeAxisMax);

/// <summary>一个实例保存一个声道同一组设置下的平均历史。每次 Push 交入一个已触发的采集块。</summary>
public sealed class SequenceSignalAnalyzer
{
    // 过零内插使用连续 127 点；波形与功率各自保存跨帧的计数和历史值。
    public const int PositiveZeroCrossingPreTrigger = 64;
    public const int PositiveZeroCrossingKernelLength = 2 * PositiveZeroCrossingPreTrigger - 1;
    private readonly int length, averages, halfWidth;
    private readonly double sampleRate;
    private readonly bool digital, dcCoupled, moveToBinCenter;
    private readonly bool desktopWorksheet;
    private readonly SequenceAverage mode;
    private readonly SequenceFftWindow window;
    private double[]? waveform, power;
    private int waveCount, powerCount;
    private bool waveInvalid, powerInvalid;
    public double? LastMoveToBinCenterRatio { get; private set; }

    // 一个实例固定采样率、偶数帧长及窗。平均次数至少为一，频格间距等于采样率除以帧长。
    public SequenceSignalAnalyzer(int fftLength, double sampleRate, FftWindowType windowType,
        SequenceAverage mode = SequenceAverage.Power, int averages = 1, bool digital = true, bool dcCoupled = true)
        : this(fftLength, sampleRate, windowType, mode, averages, digital, dcCoupled, SequenceBinEstimator.DesktopWorksheet)
    {
    }

    /// <summary>创建实际 Sequence Signal Analyzer 页面使用的桌面工作表。</summary>
    public static SequenceSignalAnalyzer CreateDesktop(int fftLength, double sampleRate, FftWindowType windowType,
        SequenceAverage mode = SequenceAverage.Power, int averages = 1)
        => new(fftLength, sampleRate, windowType, mode, averages, digital: true, dcCoupled: true, desktopWorksheet: true);

    private SequenceSignalAnalyzer(int fftLength, double sampleRate, FftWindowType windowType,
        SequenceAverage mode, int averages, bool digital, bool dcCoupled, bool desktopWorksheet)
    {
        if (fftLength < 16 || fftLength % 2 != 0) throw new ArgumentOutOfRangeException(nameof(fftLength));
        if (!(sampleRate > 0 && double.IsFinite(sampleRate))) throw new ArgumentOutOfRangeException(nameof(sampleRate));
        if (averages < 1) throw new ArgumentOutOfRangeException(nameof(averages));
        if (!Enum.IsDefined(mode) || !Enum.IsDefined(windowType)) throw new ArgumentException("未知的平均模式或窗。");
        length = fftLength;
        this.sampleRate = sampleRate;
        this.mode = mode;
        this.averages = averages;
        this.digital = digital;
        this.dcCoupled = dcCoupled;
        this.desktopWorksheet = desktopWorksheet;
        moveToBinCenter = windowType == FftWindowType.NoneMoveToBinCenter;
        bool previous = SequenceBinEstimator.DesktopWorksheet;
        SequenceBinEstimator.DesktopWorksheet = desktopWorksheet;
        try { window = SequenceFftWindow.Get(length, moveToBinCenter ? FftWindowType.Rectangular : windowType); }
        finally { SequenceBinEstimator.DesktopWorksheet = previous; }
        halfWidth = window.HalfWidth;
    }

    // 输入已触发的一块采样和触发位置。频谱固定点数，回传波形与完整采集块等长。
    public SequenceSpectrum Push(ReadOnlySpan<double> samples, double triggerPosition = 0,
        double triggerDelaySeconds = 0, double timeZeroSample = 0)
    {
        bool previous = SequenceBinEstimator.DesktopWorksheet;
        SequenceBinEstimator.DesktopWorksheet = desktopWorksheet;
        try { return PushCore(samples, triggerPosition, triggerDelaySeconds, timeZeroSample); }
        finally { SequenceBinEstimator.DesktopWorksheet = previous; }
    }

    private SequenceSpectrum PushCore(ReadOnlySpan<double> samples, double triggerPosition,
        double triggerDelaySeconds, double timeZeroSample)
    {
        // 逐点平均要求一组记录形状相同；触发位置用采样点计，延时另以秒计。
        if (samples.IsEmpty || waveform != null && waveform.Length != samples.Length)
            throw new ArgumentException("一次平均期间必须保持采集块长度一致。", nameof(samples));
        if (!(triggerPosition >= 0 && double.IsFinite(triggerPosition))) throw new ArgumentOutOfRangeException(nameof(triggerPosition));
        // 同步平均把同相位的采样点先相加；普通功率平均在 FFT 后相加，相反相位的两帧依然保留功率。
        Accumulate(ref waveform, ref waveCount, ref waveInvalid, samples, mode == SequenceAverage.Synchronous ? averages : 1, SequenceAverage.Power);
        var data = waveform!;
        // 起点取整并夹在最后一个能容纳 FFT 段的位置。MoveToBinCenter 模式下先重采样使主频居中再做无窗 FFT。
        var slice = new double[length];
        if (moveToBinCenter && data.Length >= ApxMoveToBinCenter.CalculateMinimumAcqLength(length))
        {
            int srcStart = (int)Math.Min(Math.Round(triggerPosition), Math.Max(0, data.Length - ApxMoveToBinCenter.CalculateMinimumAcqLength(length)));
            var moved = ApxMoveToBinCenter.MoveToBinCenter(data.AsSpan(srcStart), length, sampleRate, out double? ratio);
            LastMoveToBinCenterRatio = ratio;
            moved.AsSpan(0, Math.Min(length, moved.Length)).CopyTo(slice);
        }
        else
        {
            LastMoveToBinCenterRatio = null;
            int start = (int)Math.Min(Math.Round(triggerPosition), Math.Max(0, data.Length - length));
            data.AsSpan(start, Math.Min(length, data.Length - start)).CopyTo(slice);
        }
        // 数字输入乘 √2，使峰值为一的正弦按有效值报告为一；模拟输入保留原尺度。
        double units = digital ? Math.Sqrt(2) : 1;
        for (int i = 0; i < length; i++) slice[i] *= units;
        var raw = window.Transform(slice);
        var fft = raw;
        if (!dcCoupled)
        {
            // 原版用已换算成数字刻度的 FFT 直流值，同时修正返回的时域数组。
            // 因此这里的回传波形也进入下一次同步平均，测试要连续喂多帧才能看见这个行为。
            double dc = raw[0].Real * window.DcCorrection;
            fft = (Complex[])raw.Clone();
            window.SubtractDc(fft);
            for (int i = 0; i < data.Length; i++) data[i] -= dc;
        }
        // 去直流后的复谱变为本帧功率；同步平均在时域完成，其余模式在这里用功率更新历史。
        var current = SequenceFftWindow.Power(fft, length, fft.Length);
        Accumulate(ref power, ref powerCount, ref powerInvalid, current, mode == SequenceAverage.Synchronous ? 1 : averages, mode);
        int count = current.Length;
        var hz = new double[count];
        var spectrum = SequenceBinEstimator.SquareRoots(power!);
        var psd = new double[count];
        double df = sampleRate / length;
        for (int k = 0; k < count; k++)
        {
            hz[k] = k * df;
            // 同一格既可表示单音读数，也可表示每 Hz 的噪声功率。前者补偿窗主瓣，后者除以格宽。
            spectrum[k] *= window.PeakCorrection;
            psd[k] = power![k] * (length / sampleRate);
        }
        // PSD 开方得到幅值/√Hz；时轴每点递增 1/Fs 秒，再平移到触发定义的零点。
        var asd = SequenceBinEstimator.SquareRoots(psd);
        var time = new double[data.Length];
        double secondsPerSample = 1.0 / sampleRate;
        double origin = triggerDelaySeconds - timeZeroSample * secondsPerSample;
        for (int i = 0; i < time.Length; i++) time[i] = i * secondsPerSample + origin;
        // 用本帧主峰邻域求重心和主瓣功率。检测到周期成分占优时，用估计周期限制显示时长。
        int peak = Array.IndexOf(current, current.Max());
        double bin = SequenceTransform.Centroid(current, peak, halfWidth);
        double tonePower = 0;
        for (int k = Math.Max(0, peak - halfWidth); k <= Math.Min(count - 1, peak + halfWidth); k++) tonePower += current[k];
        double maxTime = time[^1];
        if (bin > 1 && SequenceTransform.NonPeriodicLevel(current, bin, halfWidth) / Math.Sqrt(tonePower) < 0.25)
            maxTime = Math.Min(maxTime, triggerDelaySeconds + 3.0 * length / (bin - 1) * secondsPerSample);
        // 返回波形副本以免后续 Push 改写；raw 保存本次去直流以前的加窗复谱。
        return new(hz, time, (double[])data.Clone(), spectrum, psd, asd, raw, window.PeakCorrection, maxTime);
    }

    // 在索引 63 和 64 之间查找正向过零时刻，结果以含小数的采样位置表示。
    public static double InterpolatePositiveZeroCrossing(ReadOnlySpan<double> samples)
    {
        const int previous = PositiveZeroCrossingPreTrigger - 1;
        const int phases = 256;
        // 需要覆盖所有 127 个抽头；中心两个样本之一已为零时直接返回对应整数位置。
        if (samples.Length < PositiveZeroCrossingKernelLength)
            throw new ArgumentException("Positive zero crossing requires 127 samples around the trigger.", nameof(samples));
        if (samples[previous + 1] == 0) return previous + 1;
        if (samples[previous] == 0) return previous;
        double low = 0, high = 1, middle = 0;
        // 每轮二分试一个位置，将分数延时分成 256 个相位间隔，以相邻两套核线性插值。
        for (int step = 0; step < 10; step++)
        {
            middle = (low + high) / 2;
            double phase = middle * phases;
            int index = (int)phase;
            double blend = phase - index;
            double value = 0;
            // 每个邻近样本乘该分数位置的权重后相加，得到试探时刻的波形幅值。
            for (int tap = 0; tap < PositiveZeroCrossingKernelLength; tap++)
            {
                double lower = SincTap(tap - previous - (double)index / phases);
                double upper = SincTap(tap - previous - (double)(index + 1) / phases);
                value += (blend * upper + (1 - blend) * lower) * samples[tap];
            }
            // 正向过零时正值指向较早的半段，负值指向较晚半段，正好为零则结束。
            if (value > 0) high = middle;
            else if (value < 0) low = middle;
            else break;
        }
        return previous + middle;
    }

    // sinc 核在中心的极限为 1；用余弦窗的平方压低较远样本对重建的影响。
    private static double SincTap(double offset)
    {
        double angle = Math.PI * offset;
        double window = 0.5 + 0.5 * Math.Cos(angle / PositiveZeroCrossingPreTrigger);
        return (angle == 0 ? 1 : Math.Sin(angle) / angle) * (window * window);
    }

    // 输入每个位置代表同一时刻或频格。历史数组、计数及无效标记由调用方跨帧保留。
    private static void Accumulate(ref double[]? previous, ref int count, ref bool wasInvalid, ReadOnlySpan<double> input,
        int limit, SequenceAverage mode)
    {
        // 首点的 NaN 状态改变时重新初始化；整帧出现 NaN 则不计入有效帧数。
        bool invalid = double.IsNaN(input[0]);
        if (previous == null || invalid != wasInvalid)
        {
            wasInvalid = invalid;
            bool hasNaN = input.Contains(double.NaN);
            previous = hasNaN && limit > 1 ? new double[input.Length] : input.ToArray();
            count = hasNaN ? 0 : 1;
            return;
        }
        // 计数到上限后固定。最大、最小保持逐点取极值，普通模式按当前计数给新帧分配权重。
        count = Math.Min(count + 1, limit);
        for (int i = 0; i < input.Length; i++)
        {
            // 达到平均次数后继续用 1/count 更新，所以第 4 帧在“两帧平均”中仍能留下第 1 帧的影响。
            previous[i] = mode switch
            {
                SequenceAverage.MaxHold => Math.Max(previous[i], input[i]),
                SequenceAverage.MinHold => Math.Min(previous[i], input[i]),
                _ => previous[i] * (1 - 1.0 / count) + input[i] * (1.0 / count)
            };
        }
    }
}
