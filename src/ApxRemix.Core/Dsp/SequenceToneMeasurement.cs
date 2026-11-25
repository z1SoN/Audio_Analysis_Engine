using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>一帧 PCM 的线性读数。数字输入以满幅正弦为 1，直流和绝对峰值始终使用样本自身的单位。</summary>
public sealed record SequenceToneReading(double Peak, double DcLevel, double RmsLevel, double RmsLevelWithDc,
    double FundamentalLevel, double FrequencyHz, double PhaseRadians, bool IsNoise,
    double ThdNLevel, double ThdLevel, double NoiseLevel, double[] HarmonicLevels)
{
    /// <summary>本帧频率是否使用了对应工作表进程的 Intel MKL 的数值路径。缺少该运行库时使用托管 FFT。</summary>
    public bool UsedMklFrequencyEstimator { get; init; }
    public double FundamentalBin { get; init; }
    // 加入一个二次谐波，两种分母就分开了。默认比值用全部交流有效值，IEEE 读数只用基波。
    // A file batch settles residual amplitude and ratio separately. Dividing
    // the reported residual by the last frame's RMS loses that history.
    public double? ReportedThdNRatio { get; init; }
    public double? ReportedThdNRatioIeee { get; init; }
    // 文件判稳提供独立的比值时使用该值；单帧可分别用总交流 RMS 或基波作为残差的分母。
    public double ThdNRatio => ReportedThdNRatio ?? (IsNoise ? double.NaN : ThdNLevel / RmsLevel);
    public double ThdNRatioIeee => ReportedThdNRatioIeee ?? (IsNoise ? double.NaN : ThdNLevel / FundamentalLevel);
    // 谐波比只计谐波，SINAD 用总交流电平除以含噪残差；噪声帧没有这些单音比值。
    public double ThdRatio => IsNoise ? double.NaN : ThdLevel / RmsLevel;
    public double Sinad => IsNoise ? double.NaN : RmsLevel / ThdNLevel;
    // 输入参考为正时，相同单位的 RMS 相除给出线性增益。
    public double GainRelativeTo(double referenceLevel) => referenceLevel > 0 ? RmsLevel / referenceLevel : double.NaN;
}

/// <summary>Sequence 的 Level &amp; Gain 和 THD+N 共用的帧计算，对应原版 ProcessingWorksheet。
/// 调用者提供同一声道的一次采样。需要两路相位差时传入同一次采集的两个读数。</summary>
public static class SequenceToneMeasurement
{
    // 单帧便捷入口关闭输入滤波，输入和残差工作表计权取平直，再把各设置传入通用计算。
    public static SequenceToneReading Analyze(ReadOnlySpan<double> samples, double sampleRate,
        FftWindowType windowType = FftWindowType.BlackmanHarris, bool digital = true,
        bool dcCoupled = true, double bandwidthFraction = 1, double? frequencyHintHz = null,
        double? fixedNotchHz = null, int maxHarmonic = 20, AnalyzerWeighting weighting = AnalyzerWeighting.Flat,
        double residualLowpassHz = 0)
        => AnalyzeFiltered(samples, sampleRate, 0, 0, 0, AnalyzerWeighting.Flat, AnalyzerWeighting.Flat, windowType, digital, dcCoupled,
            bandwidthFraction, frequencyHintHz, fixedNotchHz, maxHarmonic, weighting, residualLowpassHz);

    // APx500 evaluates the THD+N batch in its x64 desktop host after acquisition.
    // Preserve the other worksheet profiles and restore the caller's selection on failure.
    // 本次调用临时选择工作表位数及数值路径；即便失败，finally 也恢复先前的选择。
    public static SequenceToneReading AnalyzeWorksheet(bool desktopWorksheet, ReadOnlySpan<double> samples, double sampleRate,
        double inputHighpassHz, double inputButterworthLowpassHz, double inputAes17LowpassHz,
        AnalyzerWeighting inputWeighting, AnalyzerWeighting residualWeighting,
        double? frequencyHintHz = null, double? fixedNotchHz = null)
    {
        bool previous = SequenceBinEstimator.DesktopWorksheet;
        SequenceBinEstimator.DesktopWorksheet = desktopWorksheet;
        try
        {
            return AnalyzeCore(samples, sampleRate, inputHighpassHz, inputButterworthLowpassHz, inputAes17LowpassHz,
                inputWeighting, residualWeighting, FftWindowType.BlackmanHarris, true, true, 1, frequencyHintHz,
                fixedNotchHz, 20, AnalyzerWeighting.Flat, 0, excludeNyquist: desktopWorksheet);
        }
        finally { SequenceBinEstimator.DesktopWorksheet = previous; }
    }

    // 单独配置输入和残差滤波时走此入口，完整带宽可包含奈奎斯特格。
    public static SequenceToneReading AnalyzeFiltered(ReadOnlySpan<double> samples, double sampleRate,
        double inputHighpassHz, double inputButterworthLowpassHz, double inputAes17LowpassHz,
        AnalyzerWeighting inputWeighting, AnalyzerWeighting residualWeighting,
        FftWindowType windowType = FftWindowType.BlackmanHarris, bool digital = true,
        bool dcCoupled = true, double bandwidthFraction = 1, double? frequencyHintHz = null,
        double? fixedNotchHz = null, int maxHarmonic = 20, AnalyzerWeighting weighting = AnalyzerWeighting.Flat,
        double residualLowpassHz = 0)
        => AnalyzeCore(samples, sampleRate, inputHighpassHz, inputButterworthLowpassHz, inputAes17LowpassHz,
            inputWeighting, residualWeighting, windowType, digital, dcCoupled, bandwidthFraction, frequencyHintHz,
            fixedNotchHz, maxHarmonic, weighting, residualLowpassHz, excludeNyquist: false);

    // Fast Sweep 的 Hann 窗覆盖实测切片全长。稳态和失真切片都可能是奇数点。
    internal static SequenceToneReading AnalyzeFastSweepSlice(ReadOnlySpan<double> samples, int sampleRate, double frequencyHz)
    {
        bool previous = SequenceBinEstimator.DesktopWorksheet;
        SequenceBinEstimator.DesktopWorksheet = true;
        try
        {
            return AnalyzeCore(samples, sampleRate, 0, 0, 0, AnalyzerWeighting.Flat, AnalyzerWeighting.Flat,
                FftWindowType.Hann, true, true, 1, frequencyHz, null, 200, AnalyzerWeighting.Flat, 0,
                excludeNyquist: false, allowOddLength: true);
        }
        finally { SequenceBinEstimator.DesktopWorksheet = previous; }
    }

    // 先在整段记录中找出基波的位置，再从基波以外的分量估计失真和噪声。
    // N 个样本占 N/Fs 秒。频率为 f 的音在这段时间走过 f·N/Fs 个周期，
    // 这个数也是它在 FFT 频率轴上的位置。例如 Fs=48 kHz、N=4,800 时，1 kHz 恰好落在第 100 格。
    private static SequenceToneReading AnalyzeCore(ReadOnlySpan<double> samples, double sampleRate,
        double inputHighpassHz, double inputButterworthLowpassHz, double inputAes17LowpassHz,
        AnalyzerWeighting inputWeighting, AnalyzerWeighting residualWeighting, FftWindowType windowType,
        bool digital, bool dcCoupled, double bandwidthFraction, double? frequencyHintHz, double? fixedNotchHz,
        int maxHarmonic, AnalyzerWeighting weighting, double residualLowpassHz, bool excludeNyquist, bool allowOddLength = false)
    {
        // 偶数帧便于定位奈奎斯特端点；频率提示和陷波中心都必须落在可表达的频率范围。
        int n = samples.Length;
        if (n < 16) throw new ArgumentException("需要至少 16 个样本。", nameof(samples));
        if (!allowOddLength && n % 2 != 0) throw new ArgumentException("需要偶数帧长。", nameof(samples));
        if (!(double.IsFinite(sampleRate) && sampleRate > 0)) throw new ArgumentOutOfRangeException(nameof(sampleRate));
        if (!(bandwidthFraction > 0 && bandwidthFraction <= 1)) throw new ArgumentOutOfRangeException(nameof(bandwidthFraction));
        if (maxHarmonic < 2 || maxHarmonic > 1024) throw new ArgumentOutOfRangeException(nameof(maxHarmonic));
        if (frequencyHintHz.HasValue && !(frequencyHintHz >= 0 && frequencyHintHz <= sampleRate / 2))
            throw new ArgumentOutOfRangeException(nameof(frequencyHintHz));
        if (fixedNotchHz.HasValue && !(fixedNotchHz >= 0 && fixedNotchHz <= sampleRate / 2))
            throw new ArgumentOutOfRangeException(nameof(fixedNotchHz));
        // 计权方式须可识别；滤波截止频率须有限且非负，零用于关闭该项。
        if (!Enum.IsDefined(weighting) || !Enum.IsDefined(inputWeighting) || !Enum.IsDefined(residualWeighting)) throw new ArgumentOutOfRangeException(nameof(weighting));
        if (!(inputHighpassHz >= 0 && double.IsFinite(inputHighpassHz) && inputButterworthLowpassHz >= 0 && double.IsFinite(inputButterworthLowpassHz) &&
            inputAes17LowpassHz >= 0 && double.IsFinite(inputAes17LowpassHz)))
            throw new ArgumentOutOfRangeException(nameof(inputHighpassHz));
        if (!(residualLowpassHz >= 0 && double.IsFinite(residualLowpassHz))) throw new ArgumentOutOfRangeException(nameof(residualLowpassHz));
        // 由窗型决定积分半宽，截取分析频带时保留带边额外格，但最远止于 Fs/2。
        var window = SequenceFftWindow.Get(n, windowType);
        int width = window.HalfWidth;
        double scale = digital ? Math.Sqrt(2) : 1;
        int count = Math.Min(n / 2 + 1, (int)Math.Ceiling(bandwidthFraction * n / 2) + 4);
        var ac = window.Transform(samples);
        double dc = ac[0].Real * window.DcCorrection;
        double[]? inputPowerGain = null;
        // 逐格将各输入滤波级的功率增益相乘，作为稍后几种读数共用的增益数组。
        if (inputHighpassHz > 0 || inputButterworthLowpassHz > 0 || inputAes17LowpassHz > 0 || inputWeighting != AnalyzerWeighting.Flat)
        {
            inputPowerGain = new double[count];
            for (int k = 0; k < count; k++)
                inputPowerGain[k] = AudioMetricsAnalyzer.ComputeAnalyzerPowerGain(k * sampleRate / n, inputHighpassHz,
                    inputButterworthLowpassHz, inputWeighting, (int)sampleRate) * LowpassPower(k, n, sampleRate, inputAes17LowpassHz);
        }
        // 含直流读数必须先在未经去直流的频谱上积分，交流耦合时此字段无读数。
        var fullPower = dcCoupled ? SequenceFftWindow.Power(ac, n, count) : null;
        if (fullPower != null && inputPowerGain != null)
            for (int k = 0; k < count; k++) fullPower[k] *= inputPowerGain[k];
        double withDc = fullPower == null ? double.NaN : scale * Math.Sqrt(fullPower.Sum());
        // 减掉窗造成的直流泄漏后找最大格；自动跟踪可用频率提示指定局部搜索的中心。
        window.SubtractDc(ac);
        var power = SequenceFftWindow.Power(ac, n, count);
        int peak = 0;
        for (int k = 1; k < count; k++)
            if (power[k] > power[peak]) peak = k;
        int center = frequencyHintHz.HasValue && !fixedNotchHz.HasValue
            ? Math.Min(count - 1, (int)Math.Round(Math.Min(frequencyHintHz.Value / sampleRate, bandwidthFraction / 2) * n)) : peak;
        // 窗把一根谱线的能量摊给相邻几格。以功率为砝码求重心，可读出落在两格之间的基频。
        // The desktop worksheet reads the spectrum it just computed. Rebuilding
        // its taper here could pick a different alignment for the native dot product.
        bool usedMkl = SequenceBinEstimator.UsesNativeLibrary;
        double bin;
        // 非桌面配置若可用则由工作端估计分数格，其他情况在本次功率谱求重心；三项窗另补重心偏差。
        if (!SequenceBinEstimator.DesktopWorksheet && SequenceBinEstimator.TryEstimate(samples, windowType, count, width,
                frequencyHintHz.HasValue && !fixedNotchHz.HasValue ? center : null, out int measuredPeak, out bin))
            center = measuredPeak;
        else
            bin = SequenceTransform.Centroid(power, center, width);
        if (windowType == FftWindowType.BlackmanHarris) bin -= CentroidBias(bin - center);
        // State always supplies the AES17 descriptor at full bandwidth. Its sole
        // affected bin is Fs/2, after peak search and the unfiltered RMS with DC.
        // 桌面全带宽计算到此才清掉最高格，前面的选峰及含直流读数都已经完成。
        bool clearNyquist = excludeNyquist && count == n / 2 + 1;
        if (clearNyquist) power[^1] = 0;
        // 原版先在未计权频谱定位，再计权并读幅值。否则开启 A 计权可能让另一根高频谱线变成“基波”。
        if (weighting != AnalyzerWeighting.Flat || fixedNotchHz.HasValue && residualLowpassHz > 0)
            for (int k = 0; k < count; k++)
                power[k] *= fixedNotchHz.HasValue ? LowpassPower(k, n, sampleRate, residualLowpassHz)
                    : AudioMetricsAnalyzer.ComputeWeightingPowerGain(k * sampleRate / n, weighting);
        // 重建正弦的电平取自施加输入滤波前的主瓣；施加增益后同一区间的功率另用于报告基波。
        double removalLevel = scale * Math.Sqrt(Band(power, center, width));
        if (inputPowerGain != null)
            for (int k = 0; k < count; k++) power[k] *= inputPowerGain[k];
        double fundamental = scale * Math.Sqrt(Band(power, center, width));
        // 剩余非周期电平相对主峰过大就标记噪声。无可用测频提示或使用固定陷波时频率也置 NaN。
        double noiseRatio = SequenceTransform.NonPeriodicLevel(power, peak, width) / (scale * Math.Sqrt(Band(power, peak, width)));
        bool noise = !(noiseRatio <= 0.25);
        if (noise && (!frequencyHintHz.HasValue || fixedNotchHz.HasValue)) bin = double.NaN;
        // 用分数格下方的复频格方向估相，并扣除格内偏移带来的相移；无有效频格则相位无定义。
        int lower = double.IsFinite(bin) ? (int)bin : -1;
        double phase = lower >= 0 && lower < count
            ? SequenceBinEstimator.BinPhase(ac[lower].Imaginary, ac[lower].Real, bin - lower) : double.NaN;
        double rms = scale * Math.Sqrt(power.Sum());

        // 可调陷波先在时域减掉一根同频同相的正弦。固定陷波只有指定中心，它不尝试消掉输入正弦。
        // 固定中心、提示频率、当前估计依次决定陷波位置。复制原始样本供后续时域相减。
        double notch = fixedNotchHz.HasValue ? Math.Min(fixedNotchHz.Value / sampleRate, bandwidthFraction / 2) * n :
            frequencyHintHz.HasValue ? frequencyHintHz.Value * n / sampleRate : bin;
        var residual = samples.ToArray();
        if (!fixedNotchHz.HasValue && double.IsFinite(notch))
            SubtractSine(residual, removalLevel / scale, phase, notch);
        // 对新残差从头做加窗 FFT，去直流后得到可按频率施加计权并积分的功率谱。
        var residualSpectrum = window.Transform(residual);
        window.SubtractDc(residualSpectrum);
        var remaining = SequenceFftWindow.Power(residualSpectrum, n, count);
        if (clearNyquist) remaining[^1] = 0;
        // 残差从原始时域数据重新计算，因此要把输入链增益和残差链的功率增益都乘入。
        if (weighting != AnalyzerWeighting.Flat || residualLowpassHz > 0 || inputPowerGain != null || residualWeighting != AnalyzerWeighting.Flat)
            for (int k = 0; k < count; k++)
            {
                double hz = k * sampleRate / n;
                remaining[k] *= AudioMetricsAnalyzer.ComputeWeightingPowerGain(hz, weighting)
                    * LowpassPower(k, n, sampleRate, residualLowpassHz)
                    * (inputPowerGain?[k] ?? 1)
                    * AudioMetricsAnalyzer.ComputeWeightingPowerGain(hz, residualWeighting);
            }
        // m 次谐波位于 m 倍基频，按最近整数格定位后积分邻近主瓣；中心越过带宽即结束。
        var harmonics = new double[maxHarmonic - 1];
        double harmonicsPower = 0;
        for (int order = 2; order <= maxHarmonic && double.IsFinite(notch); order++)
        {
            int harmonic = (int)Math.Round(order * notch);
            if (harmonic >= count) break;
            double level = Math.Sqrt(Band(remaining, harmonic, width));
            harmonics[order - 2] = scale * level;
            harmonicsPower += level * level;
        }
        // 各次谐波读完后才对频域残差施加陷波，其功率总和给出 THD+N；绝对峰值单独扫描原样本。
        if (double.IsFinite(notch)) ApplyResidualNotch(remaining, notch);
        double residualPower = remaining.Sum();
        double peakSample = 0;
        foreach (double x in samples) peakSample = Math.Max(peakSample, Math.Abs(x));
        // 谐波先于陷波积分，因此最后一次相减有可能为负；原版此时给出 NaN。
        return new(peakSample, dc, rms, withDc, fundamental, bin * (sampleRate / n), phase, noise,
            scale * Math.Sqrt(residualPower), scale * Math.Sqrt(harmonicsPower),
            scale * Math.Sqrt(residualPower - harmonicsPower), harmonics) { UsedMklFrequencyEstimator = usedMkl, FundamentalBin = bin };
    }

    // RMS 乘 √2 得到正弦峰值，每点相位增加 2π·周期数/点数。专用数值端接手时已完成原地扣除。
    private static void SubtractSine(double[] samples, double rms, double phase, double cycles)
    {
        if (SequenceBinEstimator.TrySubtractSine(samples, rms, phase, cycles)) return;
        double amplitude = rms * Math.Sqrt(2);
        double step = cycles * 2 * Math.PI / samples.Length;
        for (int i = 0; i < samples.Length; i++, phase += step) samples[i] -= amplitude * Math.Sin(phase);
    }

    // 对每一格乘相应的陷波功率增益，清掉基波附近残留后才能积分总残差。
    private static void ApplyResidualNotch(double[] power, double fundamentalBin)
    {
        if (SequenceBinEstimator.TryApplyResidualNotch(power, fundamentalBin)) return;
        for (int k = 0; k < power.Length; k++)
            power[k] *= AudioMetricsAnalyzer.ComputeThdNotchPowerGain(k, fundamentalBin);
    }

    // 中心两侧各取 width 格积分，两端包括在内，同时跳过零频并防止越界。
    private static double Band(double[] power, int center, int width)
    {
        double sum = 0;
        for (int k = Math.Max(1, center - width); k <= Math.Min(power.Length - 1, center + width); k++) sum += power[k];
        return sum;
    }

    // 结果为乘在功率上的低通增益；截止频率为零时全通，否则按帧长换成截止频格。
    private static double LowpassPower(int bin, int length, double sampleRate, double cutoff)
    {
        if (cutoff <= 0) return 1;
        // Worksheet 的 AES17 滤波器以频格为单位保留截止点旁边的四格，再按每十倍频率
        // 1000 dB 衰减。低采样点数时，直接在截止频率套连续公式会提前衰减这几格。
        double cutoffBin = cutoff * length / sampleRate;
        int nyquist = length / 2;
        // 通带延至奈奎斯特附近时只清该端点；一般情形先判全通、全阻，再算过渡带内的衰减。
        if (Math.Ceiling(cutoffBin) + 4 >= nyquist) return bin == nyquist ? 0 : 1;
        if (bin < Math.Min((int)Math.Floor(cutoffBin) + 4, nyquist - 1)) return 1;
        if (bin >= Math.Ceiling(cutoffBin * Math.Pow(10, 0.15))) return 0;
        return AudioMetricsAnalyzer.ComputeAes17LowpassPowerGain(bin * sampleRate / length, cutoff);
    }

    // 10.0.0.17 黑曼-哈里斯三项窗的频格偏差校准。保留常数和偶次项；
    // 正负半格附近，单靠对称曲线拟合会引入另一个细小偏差。
    private static readonly double[] FrequencyCorrection = [2.37891457165261e-9, -0.00110723343611227,
        -1.8475185200519e-9, 0.00166903398253309, 5.32459150969075e-10, -0.00123232314615846,
        -6.90743575402e-11, 0.000526878390668708, 3.88232073778795e-12, -0.000106426316883181,
        -7.6543894418315e-14, 1.29063839040229e-6, -3.4481791499697e-17];

    // 分数格偏移夹在半格以内，用 Horner 递推计算所需减去的偏差（单位仍是频格）。
    private static double CentroidBias(double offset)
    {
        double x = Math.Clamp(offset,-0.5,0.5);
        double bias = 0;
        foreach (double coefficient in FrequencyCorrection) bias = bias*x + coefficient;
        return bias;
    }

    // 两路相近频率相减才有稳定的相位差；通过取整周将弧度差折算到 [−180°,180°)。
    public static double RelativePhaseDegrees(SequenceToneReading channel, SequenceToneReading reference)
    {
        if (ReferenceEquals(channel, reference) || channel.IsNoise || !(Math.Abs(channel.FrequencyHz / reference.FrequencyHz - 1) < 0.001)) return double.NaN;
        double turns = (channel.PhaseRadians - reference.PhaseRadians) / (2 * Math.PI);
        return 360 * (turns - Math.Floor(turns + 0.5));
    }

    /// <summary>分析普通 Sequence 相位页使用的同一帧双通道 PCM，返回两个参考方向及下一帧长度。</summary>
    public static (double LeftDegrees, double RightDegrees, double LeftFrequencyHz, double RightFrequencyHz, int NextLength)
        AnalyzeInterchannelPhase(ReadOnlySpan<double> left, ReadOnlySpan<double> right, int sampleRate)
    {
        if (left.Length < 16 || left.Length != right.Length)
            throw new ArgumentException("Phase requires two equal-length input records of at least 16 samples.");
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(sampleRate);

        int length = left.Length;
        bool previous = SequenceBinEstimator.DesktopWorksheet;
        SequenceBinEstimator.DesktopWorksheet = true;
        try
        {
            var window = SequenceFftWindow.Get(length, FftWindowType.BlackmanHarris);
            var first = PhaseChannel(left, window);
            var second = PhaseChannel(right, window);
            // AcqLen uses the integer peak and requires two window half-widths for phase.
            int minimumBin = 2 * window.HalfWidth;
            int lowest = Math.Min(first.Noise ? int.MaxValue : first.Peak, second.Noise ? int.MaxValue : second.Peak);
            int maximumLength = SequenceTransform.NextLength((int)((minimumBin + 1) / 5.0 * sampleRate), allowShort: true);
            int nextLength = lowest == int.MaxValue ? (int)Math.Ceiling(0.05 * sampleRate)
                : Math.Min(maximumLength, lowest < minimumBin ? length * 4
                    : SequenceTransform.NextLength(Math.Max(1024, length * minimumBin / lowest), allowShort: true));
            double leftCycles = first.Bin / length, rightCycles = second.Bin / length;
            return (PhaseDifference(first.Angle, leftCycles, first.Noise, second.Angle, rightCycles),
                PhaseDifference(second.Angle, rightCycles, second.Noise, first.Angle, leftCycles),
                first.Bin * ((double)sampleRate / length), second.Bin * ((double)sampleRate / length), nextLength);
        }
        finally { SequenceBinEstimator.DesktopWorksheet = previous; }
    }

    private static (int Peak, double Bin, double Angle, bool Noise) PhaseChannel(ReadOnlySpan<double> samples, SequenceFftWindow window)
    {
        var spectrum = window.Transform(samples);
        window.SubtractDc(spectrum);
        var power = SequenceFftWindow.Power(spectrum, samples.Length, spectrum.Length);
        int peak = 0;
        for (int k = 1; k < power.Length; k++)
            if (power[k] > power[peak]) peak = k;

        int width = window.HalfWidth;
        double bin = SequenceTransform.Centroid(power, peak, width);
        bin -= CentroidBias(bin - peak);
        if (!double.IsFinite(bin)) return (peak, bin, double.NaN, true);
        double amplitude = Math.Sqrt(Band(power, (int)Math.Round(bin), width));
        // XfrmObj keeps the unmasked spectrum for phase. Only its noise estimate masks edge bins and 19 harmonics.
        Array.Clear(power, 0, width);
        Array.Clear(power, power.Length - width, width);
        for (int order = 1; order < 20; order++)
        {
            int center = (int)Math.Round(bin * order);
            int first = Math.Max(0, center - width);
            int last = Math.Min(power.Length - 1, center + width);
            if (first <= last) Array.Clear(power, first, last - first + 1);
        }
        double noisePower = 0, increment = 1.0 / (power.Length - 1);
        for (int k = 0; k < power.Length; k++) noisePower += power[k] * (k * increment);
        bool noise = !(amplitude > 0 && Math.Sqrt(noisePower) / amplitude <= 0.25);
        return (peak, bin, ComputePhaseCore(spectrum, bin), noise);
    }

    private static double PhaseDifference(double angle, double cycles, bool noise, double referenceAngle, double referenceCycles)
    {
        if (noise || !(Math.Abs((cycles - referenceCycles) / referenceCycles) < 0.001)) return double.NaN;
        double difference = angle - referenceAngle;
        if (difference < -Math.PI) difference += 2 * Math.PI;
        if (difference >= Math.PI) difference -= 2 * Math.PI;
        return difference * (180.0 / Math.PI);
    }

    /// <summary>
    /// 对齐 APx500 <c>PhaseMath.PhaseCore(double[] xfrm, double freq)</c>：按分数频格偏移执行半周期复数旋转求相位（弧度）。
    /// </summary>
    public static double ComputePhaseCore(ReadOnlySpan<System.Numerics.Complex> spectrum, double fractionalBin)
    {
        if (spectrum.IsEmpty || !double.IsFinite(fractionalBin) || fractionalBin < 0.0 || fractionalBin >= spectrum.Length)
            return double.NaN;
        int bin = (int)Math.Floor(fractionalBin);
        double angle = (fractionalBin - bin) * Math.PI;
        double cos = Math.Cos(angle);
        double sin = Math.Sin(angle);
        double re = spectrum[bin].Real;
        double im = spectrum[bin].Imaginary;
        double num = cos * re + sin * im;
        double den = sin * re - cos * im;
        return den != 0.0 ? Math.Atan2(num, den) : double.NaN;
    }

    /// <summary>对齐 APx500 <c>PhaseMath.Unwrap</c>（默认阈值 180°）。</summary>
    public static double[] UnwrapPhases(ReadOnlySpan<double> phases, double threshold = 180.0)
    {
        if (phases.IsEmpty) return [];
        var output = new double[phases.Length];
        double invTwoThreshold = 1.0 / (2.0 * threshold);
        output[0] = phases[0];
        for (int i = 1; i < phases.Length; i++)
        {
            double delta = phases[i] - output[i - 1];
            output[i] = Math.Abs(delta) > threshold
                ? phases[i] - Math.Round(delta * invTwoThreshold) * (2.0 * threshold)
                : phases[i];
        }
        return output;
    }

    // 从所有可用单音中取最低频，决定下一次需要的采样点数，兼顾低频主瓣及陷波的宽度。
    public static int NextAcquisitionLength(IReadOnlyList<SequenceToneReading> channels, int frameLength,
        double sampleRate, bool measureThd, FftWindowType window = FftWindowType.BlackmanHarris, double minimumHz = 5)
    {
        if (!(double.IsFinite(sampleRate) && double.IsFinite(minimumHz) && sampleRate > 0 && minimumHz > 0) || frameLength <= 0)
            throw new ArgumentOutOfRangeException(nameof(frameLength));
        // Hz 乘当前帧时长即帧内的有效周期数；没有非噪声声道时返回 1024 点。
        int width = SequenceTransform.HalfWidth(window);
        double lowest = channels.Where(ch => !ch.IsNoise).Select(ch => ch.FrequencyHz * frameLength / sampleRate)
            .DefaultIfEmpty(double.PositiveInfinity).Min();
        if (double.IsPositiveInfinity(lowest)) return 1024;
        // THD 的陷波带比电平表的主瓣更宽。Q=1.25 决定低端边界，
        // 至少要让基波下侧留出三格，当前帧不足时原版把下一帧加长四倍。
        const double q = 1.25;
        double notchFactor = 0.5 / q + Math.Sqrt(1 + 0.25 / (q * q));
        double required = measureThd ? Math.Ceiling(3 / (1 - 1 / notchFactor)) : width;
        // 周期数充足时按比例缩短，不足则扩到四倍，再按最低使用频率限幅、取支持的长度。
        double target = lowest >= required ? required / lowest * frameLength : 4.0 * frameLength;
        int cap = SequenceTransform.NextLength(checked((int)((width + 1) * sampleRate / minimumHz)));
        return SequenceTransform.NextLength((int)Math.Min(Math.Round(target), cap));
    }
}
