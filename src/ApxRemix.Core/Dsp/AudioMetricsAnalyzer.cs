// 已有一段左右声道采样时，界面可从这里一次计算电平、谐波、频谱等快照。沿 AnalyzeBenchSnapshot 看各结果怎样组合；实时 Bench 还会合并 StreamingBenchPipeline 在采样过程中得到的读数。
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APx500 核心音频测量指标分析器（服务 Bench Mode 全功能与 Sequence Mode）。
/// <para>
/// 包含：
/// - Level &amp; Gain（AES17 标定、独立输入/输出电压标定、AC Peak / Raw Peak Clip、Hann 加权 DC √2 电压、Crest Factor）；
/// - Analyzer 带宽滤波（时域四阶高通 / 八阶低通 Butterworth）与 IEC 61672-1 A 计权；
/// - THD+N（支持 Generator/Measured/Fixed 陷波调谐、Residual/Total 分母、Pure THD、THD+N 绝对电平与无信号 N/A 保护）；
/// - Bandpass Level（窄带高 Q 带通电平）与 IMD Ratio（SMPTE 60Hz+7kHz &amp; CCIF 19k+20k）；
/// - FFT 触发延时窗口截取与派生结果（带内总功率、噪声谱密度 dBFS/√Hz、PAR 与 SFDR dBc）。
/// </para>
/// </summary>
public static class AudioMetricsAnalyzer
{
    public const double Sqrt2 = 1.4142135623730951;
    public const double ZeroDbuVoltageVrms = 0.7745966692414834;

    // 同样的 dBFS 在不同量程上对应不同电压。为使输入减输出表示电压增益，再加 20log10(输入满量程 Vrms/输出满量程 Vrms)。
    /// <summary>
    /// 统一计算输入/输出满量程电压标定引入的物理电压增益补偿（dB）：<c>20·log10(V_IN_FS / V_OUT_FS)</c>。
    /// Level &amp; Gain、Stepped Sweep 与 Continuous Sweep 均共用此唯一入口，确保多处增益读数完全一致。
    /// </summary>
    public static double ComputeCalibrationGainOffsetDb(double inputFullScaleVoltageVrms, double outputFullScaleVoltageVrms) =>
        20.0 * Math.Log10(Math.Max(inputFullScaleVoltageVrms, 1e-6) / Math.Max(outputFullScaleVoltageVrms, 1e-6));

    // PCM 的直流值以满幅峰值为 1，校准量却是满幅正弦有效值。先乘 √2 恢复每单位样本对应的峰值伏特。
    /// <summary>将归一化直流样本均值换算为物理直流电压（Volts）：<c>dc_fs × √2 × V_FS_RMS</c>。</summary>
    public static double ConvertDcFsToVolts(double dcLevelFs, double fullScaleSineVrms) =>
        dcLevelFs * Sqrt2 * Math.Max(fullScaleSineVrms, 0.0);

    // IEC 61672-1 精确标准极点频率（用于与 APx500 wt_A / wt_C 1 kHz 精确归一化对齐）
    private static readonly double AWeightRef1k = EvalAWeightMagnitude(1000.0);
    private static readonly double CWeightRef1k = EvalCWeightMagnitude(1000.0);
    private static readonly double CcirRef1k = EvalCcir468Magnitude(1000.0);
    private static readonly double CcirRef2k = EvalCcir468Magnitude(2000.0);

    // 计算未按 1 kHz 归一的 A 计权幅度。频率用 Hz，各转折点与输入须取同一单位才可在平方后相加。
    private static double EvalAWeightMagnitude(double frequencyHz)
    {
        double f2 = frequencyHz * frequencyHz;
        const double c1 = 20.598997 * 20.598997;
        const double c2 = 107.65265 * 107.65265;
        const double c3 = 737.86223 * 737.86223;
        const double c4 = 12194.217 * 12194.217;
        return (c4 * f2 * f2) / ((f2 + c1) * Math.Sqrt((f2 + c2) * (f2 + c3)) * (f2 + c4));
    }

    // C 曲线只用这里的低、高频转折项。返回未归一的线性幅度，后续才换成功率权重。
    private static double EvalCWeightMagnitude(double frequencyHz)
    {
        double f2 = frequencyHz * frequencyHz;
        const double c1 = 20.598997 * 20.598997;
        const double c4 = 12194.217 * 12194.217;
        return (c4 * f2) / ((f2 + c1) * (f2 + c4));
    }

    // 将 CCIR 模型分母拆成实部和虚部求模。频率升高时各阶项作用不同，得到随频率变化的线性幅度权重。
    private static double EvalCcir468Magnitude(double frequencyHz)
    {
        double f = frequencyHz;
        double f2 = f * f;
        double f3 = f2 * f;
        double f4 = f2 * f2;
        double f5 = f4 * f;
        double f6 = f4 * f2;
        double h1 = -4.737338981378384e-24 * f6 + 2.043828333606125e-15 * f4 - 1.363894795463638e-7 * f2 + 1.0;
        double h2 = 1.306612257412824e-19 * f5 - 2.118150887518656e-11 * f3 + 5.559488023498642e-4 * f;
        return 1.246332637532143e-4 * f / Math.Sqrt(h1 * h1 + h2 * h2);
    }

    // 将 A 计权的功率比换成 dB，功率用 10log10，极小值走此函数定义的下限。
    /// <summary>计算 IEC 61672-1 标准 A 计权在指定频率处的增益（dB），1 kHz 严格为 0.000 dB。</summary>
    public static double ComputeAWeightingDb(double frequencyHz)
    {
        double pow = ComputeWeightingPowerGain(frequencyHz, AnalyzerWeighting.AWeighting);
        return pow > 1e-18 ? 10.0 * Math.Log10(pow) : -120.0;
    }

    // 返回可直接乘在谱线功率上的倍率。平直设置全通，其余设置在此实现中对 0.5 Hz 及以下取零。
    /// <summary>
    /// 计算 APx500 标准计权与去加重滤波器功率增益 |H(f)|²（覆盖 A / C / CCIR-1k / CCIR-2k / De-emphasis 50µs / 75µs）。
    /// </summary>
    public static double ComputeWeightingPowerGain(double frequencyHz, AnalyzerWeighting weighting)
    {
        if (weighting == AnalyzerWeighting.Flat) return 1.0;
        if (frequencyHz <= 0.5) return 0.0;

        switch (weighting)
        {
            // A、C 的幅度曲线都先除以自己在 1 kHz 的幅度再平方，因此参考音的功率乘数为 1。
            case AnalyzerWeighting.AWeighting:
            {
                double r = EvalAWeightMagnitude(frequencyHz) / AWeightRef1k;
                return r * r;
            }
            case AnalyzerWeighting.CWeighting:
            {
                double r = EvalCWeightMagnitude(frequencyHz) / CWeightRef1k;
                return r * r;
            }
            // B 曲线由 C 的功率响应再乘一阶高通功率，最后的常数将新增级在 1 kHz 归一。
            case AnalyzerWeighting.BWeighting:
            {
                const double pole = 158.48932;
                double f2=frequencyHz*frequencyHz;
                return ComputeWeightingPowerGain(frequencyHz,AnalyzerWeighting.CWeighting)*
                    f2/(f2+pole*pole)*(1+pole*pole/1e6);
            }
            // A 和去加重串联，功率倍率相乘。50 或 75 微秒指定高频衰减开始的尺度。
            case AnalyzerWeighting.Deemphasis50usA:
            case AnalyzerWeighting.Deemphasis75usA:
                return ComputeWeightingPowerGain(frequencyHz,AnalyzerWeighting.AWeighting) *
                    ComputeWeightingPowerGain(frequencyHz,weighting==AnalyzerWeighting.Deemphasis50usA ?
                        AnalyzerWeighting.Deemphasis50us : AnalyzerWeighting.Deemphasis75us);
            // 两个 CCIR 选项使用同一曲线，把 1 或 2 kHz 各自定义为单位增益。
            case AnalyzerWeighting.Ccir1k:
            {
                double r = EvalCcir468Magnitude(frequencyHz) / CcirRef1k;
                return r * r;
            }
            case AnalyzerWeighting.Ccir2k:
            {
                double r = EvalCcir468Magnitude(frequencyHz) / CcirRef2k;
                return r * r;
            }
            // 一阶 RC 的复幅度为 1/(1+jωτ)，故功率比为 1/(1+(ωτ)²)。这里 Hz 乘 2π，微秒已换成秒。
            case AnalyzerWeighting.Deemphasis50us:
            {
                double wTau = 2.0 * Math.PI * frequencyHz * 50e-6;
                return 1.0 / (1.0 + wTau * wTau);
            }
            case AnalyzerWeighting.Deemphasis75us:
            {
                double wTau = 2.0 * Math.PI * frequencyHz * 75e-6;
                return 1.0 / (1.0 + wTau * wTau);
            }
            case AnalyzerWeighting.Ccitt:
            case AnalyzerWeighting.CMessage:
                return ApxAnsiWeighting.EvaluateAnalogPowerGain(frequencyHz, weighting);
            default:
                return 1.0;
        }
    }

    // 这是快照积分使用的 AES17 低通功率近似。截止以下取 1，以上用高次幂做陡降，没有跨包滤波状态。
    /// <summary>计算 APx500 AES17 砖墙低通滤波器功率响应（通带严格为 1.0，截止以上为 (f_c / f)^100，与原版 LowpassAes17FilterWrapper 逐位一致）。</summary>
    public static double ComputeAes17LowpassPowerGain(double frequencyHz, double cutoffHz)
    {
        if (cutoffHz <= 0.0 || frequencyHz <= cutoffHz) return 1.0;
        return Math.Pow(cutoffHz / frequencyHz, 100.0);
    }

    /// <summary>计算 APxRT 8 阶 0.01 dB 纹波椭圆低通滤波器功率响应 |H(f)|²（与 <c>ApxEllipticLowPass</c> 同原型、同预畸变）。</summary>
    public static double ComputeElliptic8LowpassPowerGain(double frequencyHz, double cutoffHz, int sampleRate = 0)
    {
        if (cutoffHz <= 0.0) return 1.0;
        if (sampleRate > 0 && frequencyHz >= sampleRate * 0.5) return 0.0;
        double u;
        if (sampleRate > 0)
        {
            double rel = ApxEllipticLowPass.RelativeCutoff(cutoffHz, sampleRate);
            double prewarped = Math.Tan(Math.PI * (0.5 * rel));
            u = Math.Tan(Math.PI * frequencyHz / sampleRate) / prewarped;
        }
        else
        {
            u = frequencyHz / cutoffHz;
        }

        double u2 = u * u;
        const double dcGainSq = 0.99942451938 * 0.99942451938;
        ReadOnlySpan<(double Re, double Im, double ZIm)> sections =
        [
            (-0.050061961859382879, 1.0544010430691861, 1.2937938824069803),
            (-0.17959150307597616, 0.98310522840240977, 1.4437942036807718),
            (-0.38398300339739067, 0.77530916216000145, 1.9850289833081445),
            (-0.60280069931460323, 0.31633913385572676, 5.2959231779150429)
        ];

        double pow = dcGainSq;
        for (int i = 0; i < sections.Length; i++)
        {
            var (pRe, pIm, zIm) = sections[i];
            double pRe2 = pRe * pRe;
            double pMag2 = pRe2 + pIm * pIm;
            double z2 = zIm * zIm;
            double num = z2 - u2;
            double d1 = u - pIm;
            double d2 = u + pIm;
            double den = (pRe2 + d1 * d1) * (pRe2 + d2 * d2);
            double dcRatio = (pMag2 * pMag2) / (z2 * z2);
            pow *= (num * num / den) * dcRatio;
        }
        return pow;
    }

    // 求某一 FFT 频点经倍频程带通后的功率比。Fs/2 及以上没有可独立表示的正频率，返回零。
    /// <summary>
    /// 计算 APx500 6 阶 Butterworth 带通滤波器（3 阶原型 LP→BP + 双线性预畸变）功率响应 |H(f)|²，
    /// 对齐 <c>FilterDesigns.CreateButterBandpassFilter(1, order=3, f0, 2^(-1/(2N)), fs)</c>。
    /// </summary>
    public static double ComputeButterBandpassPowerGain(double frequencyHz, double centerHz, int octaveFraction, int sampleRate)
    {
        if (frequencyHz <= 1e-6 || centerHz <= 1e-6 || sampleRate <= 0 || frequencyHz >= sampleRate * 0.5)
            return 0.0;
        int frac = Math.Clamp(octaveFraction, 1, 48);
        double f0 = Math.Clamp(centerHz, 5.0, sampleRate * 0.5 * 0.99);
        // 下边缘由 2 的分数次幂确定，再预畸变到模拟频率轴。利用两边缘的几何中心求带宽及 Q。
        double f1ToF0 = Math.Pow(2.0, -1.0 / (2.0 * frac));

        double w0 = Math.Tan(Math.PI * f0 / sampleRate);
        double w1 = Math.Tan(Math.PI * (f0 * f1ToF0) / sampleRate);
        double bw = (w0 * w0 / w1) - w1;
        if (bw <= 1e-12) return 0.0;
        double q = w0 / bw;

        // 归一化失谐量在中心为零。三阶低通原型映到带通后给出 1/(1+失谐量⁶)，中心功率增益为 1。
        double w = Math.Tan(Math.PI * frequencyHz / sampleRate);
        double detune = q * (w / w0 - w0 / w);
        double d2 = detune * detune;
        return 1.0 / (1.0 + d2 * d2 * d2);
    }

    // 合成某个频格的通带与计权功率响应，每一级的功率倍率相乘，供频域积分使用。
    /// <summary>APx Butterworth HP4 / LP8（或 AES17 低通）与标准计权滤波器的统一功率响应。</summary>
    public static double ComputeAnalyzerPowerGain(
        double frequencyHz,
        double highPassHz,
        double lowPassHz,
        AnalyzerWeighting weighting,
        int sampleRate = 0,
        LowPassFilterType lowPassType = LowPassFilterType.Butterworth8)
    {
        double gainSq = 1.0;

        // 高通用四阶 Butterworth 的功率公式。给出 Fs 时按双线性变换后的频率比计算，否则采用模拟频率比。
        if (highPassHz > 0.0)
        {
            if (frequencyHz <= 1e-6) return 0.0;
            double r = sampleRate > 0
                ? Math.Tan(Math.PI * Math.Min(highPassHz, sampleRate * 0.495) / sampleRate) / Math.Tan(Math.PI * frequencyHz / sampleRate)
                : highPassHz / frequencyHz;
            gainSq /= 1.0 + Math.Pow(r, 8);
        }

        // 八阶低通功率分母有十六次幂。选择 AES17 或 Elliptic8 时分别走对应的陡降模型。
        if (lowPassHz > 0.0)
        {
            if (lowPassType == LowPassFilterType.Aes17)
            {
                gainSq *= ComputeAes17LowpassPowerGain(frequencyHz, lowPassHz);
            }
            else if (lowPassType == LowPassFilterType.Elliptic8)
            {
                gainSq *= ComputeElliptic8LowpassPowerGain(frequencyHz, lowPassHz, sampleRate);
            }
            else
            {
                double r = sampleRate > 0
                    ? Math.Tan(Math.PI * frequencyHz / sampleRate) / Math.Tan(Math.PI * Math.Min(lowPassHz, sampleRate * 0.495) / sampleRate)
                    : frequencyHz / lowPassHz;
                gainSq /= 1.0 + Math.Pow(r, 16);
            }
        }

        if (weighting != AnalyzerWeighting.Flat)
            gainSq *= ComputeWeightingPowerGain(frequencyHz, weighting);

        return gainSq;
    }

    // 由 dBFS 加上指定参考量生成显示字符串。电压需满量程校准，功率需负载阻抗，声压还需麦克风灵敏度。
    /// <summary>将 AES17 dBFS 电平按指定电声单位格式化（支持 dBr 相对参考电平）。</summary>
    public static string FormatLevelInUnit(double dbfs, double fullScaleVoltageVrms, LevelDisplayUnit unit, double dbrReferenceDbfs = 0.0,
        double dbrBReferenceDbfs = 0.0, double loadOhms = 600, double micSensitivityMvPerPa = 10)
    {
        // 无效或极低电平在显示层使用各单位相应的零/负无穷文本。
        if (double.IsNaN(dbfs) || dbfs <= -150.0)
        {
            return unit switch
            {
                LevelDisplayUnit.Vrms => "0.000 mV",
                LevelDisplayUnit.Dbv => "-inf dBV",
                LevelDisplayUnit.Dbu => "-inf dBu",
                LevelDisplayUnit.Dbr => "-inf dBr",
                LevelDisplayUnit.DbrB => "-inf dBrB",
                LevelDisplayUnit.Dbm => "-inf dBm",
                LevelDisplayUnit.DbSpl => "-inf dB SPL",
                _ => "-inf dBFS"
            };
        }

        // 满量程正弦 Vrms 乘线性幅度比得伏特有效值；dBm 用 V²/R 换算毫瓦，SPL 由毫伏除以 mV/Pa 得帕再对 20 µPa 取比。
        double vrms = SignalGeneratorDsp.DbfsToLinearPeak(dbfs) * Math.Max(fullScaleVoltageVrms, 1e-6);
        return unit switch
        {
            LevelDisplayUnit.Dbfs => $"{dbfs:F2} dBFS",
            LevelDisplayUnit.Vrms => vrms >= 0.1 ? $"{vrms:F4} Vrms" : $"{vrms * 1000.0:F2} mVrms",
            LevelDisplayUnit.Dbv => $"{20.0 * Math.Log10(Math.Max(vrms, 1e-9)):+0.00;-0.00;0.00} dBV",
            LevelDisplayUnit.Dbu => $"{20.0 * Math.Log10(Math.Max(vrms / ZeroDbuVoltageVrms, 1e-9)):+0.00;-0.00;0.00} dBu",
            LevelDisplayUnit.DbrB => $"{dbfs - dbrBReferenceDbfs:F2} dBrB",
            LevelDisplayUnit.Dbm => $"{10 * Math.Log10(vrms * vrms / loadOhms * 1000):F2} dBm",
            LevelDisplayUnit.DbSpl => $"{20 * Math.Log10(vrms * 1000 / micSensitivityMvPerPa / 20e-6):F2} dB SPL",
            LevelDisplayUnit.Dbr => $"{dbfs - dbrReferenceDbfs:+0.00;-0.00;0.00} dBr",
            _ => $"{dbfs:F2} dBFS"
        };
    }

    // 从这段采样拟合并减去直流和一个正弦，使示波残余突出其余分量。频率或记录不适用时原样返回副本。
    /// <summary>
    /// 提取去除基波正弦与直流分量后的 THD+N 残余时域波形（对齐 APx500 Scope / FFT "Show Residual" 功能）。
    /// </summary>
    public static double[] ExtractThdResidualWaveform(ReadOnlySpan<double> samples, int sampleRate, double? expectedFreqHz = null)
    {
        int n = samples.Length;
        if (n < 64 || sampleRate <= 0) return samples.ToArray();

        // 调用方提供的目标频率只有在有效带内才用作初值，否则先在输入记录中找到最强谱峰。
        double freqHz = expectedFreqHz is > 5.0 and < 48000.0 && expectedFreqHz.Value < sampleRate * 0.49
            ? expectedFreqHz.Value
            : EstimateFundamentalHzQuick(samples, sampleRate);

        if (freqHz <= 5.0 || freqHz >= sampleRate * 0.49)
            return samples.ToArray();

        // Hz 乘记录时长得到本段内的周期数，即拟合器使用的频格单位。权重全为 1 意味着每个点同等参与。
        double initialBins = freqHz * n / sampleRate;
        var weights = new double[n];
        Array.Fill(weights, 1.0);
        return SineFit.Fit(samples, initialBins, weights, fixedFrequency: false).Residual(samples);
    }

    // 从尾部最多 4096 点粗估单音 Hz，为后续时域拟合提供起点。这里最强音被当作基频。
    private static double EstimateFundamentalHzQuick(ReadOnlySpan<double> samples, int sampleRate)
    {
        int fftSize = ChooseLargestPowerOfTwo(samples.Length, 4096);
        if (fftSize < 128) return 0.0;
        var (_, _, half, _) = FftEngine.ComputeSingleSidedSpectrum(samples[^fftSize..], fftSize, FftWindowType.Hann, removeDcOffset: true);
        int peakBin = 1;
        double peakMag = 0.0;
        for (int k = 1; k < half.Length - 1; k++)
        {
            double m = half[k].Magnitude;
            if (m > peakMag) { peakMag = m; peakBin = k; }
        }
        // 峰两旁的幅度不等说明峰心偏向一侧。用三点抛物线估半格内修正，再乘 Fs/N 换回 Hz。
        if (peakMag <= 1e-9) return 0.0;
        double a = half[peakBin - 1].Magnitude;
        double b = peakMag;
        double c = half[peakBin + 1].Magnitude;
        double denom = a - 2.0 * b + c;
        double delta = Math.Abs(denom) > 1e-18 ? Math.Clamp(0.5 * (a - c) / denom, -0.5, 0.5) : 0.0;
        return (peakBin + delta) * sampleRate / fftSize;
    }

    // 在给定双声道记录上生成一次 Bench 快照。每项从这批样本计算，和连续包中保留检波历史的实时面板入口不同。
    /// <summary>执行 Bench Mode 单帧综合仪表分析（含 Bandpass Level 与 IMD Ratio）。</summary>
    public static BenchMeterSnapshot AnalyzeBenchSnapshot(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int sampleRate,
        GeneratorConfig generator,
        AnalyzerFilterConfig filterConfig,
        double inputFullScaleVoltageVrms,
        double? outputFullScaleVoltageVrms = null)
    {
        double outVfs = outputFullScaleVoltageVrms ?? inputFullScaleVoltageVrms;
        var lg = AnalyzeLevelAndGain(
            left, right, sampleRate,
            generator.LevelDbfs, generator.EffectiveChannel2LevelDbfs,
            generator.IsEnabled, inputFullScaleVoltageVrms, outVfs);
        int frequencyWindow = ChooseLargestPowerOfTwo(Math.Min(left.Length, right.Length), 16384);
        // 用去直流的交越间隔复核频谱测频。以这段 FFT 的一格宽度限制可接受修正，避免跟到另一音调。
        ChannelLevelResult Refine(ChannelLevelResult level, ReadOnlySpan<double> samples) => level.MeasuredFrequencyHz <= 0 ? level : level with
        {
            MeasuredFrequencyHz = ApxFrequencyPhaseDetector.RefineSnapshotFrequency(samples[^Math.Min(samples.Length, 16384)..],
                sampleRate, level.DcLevelFs, Math.Pow(10, level.RmsDbfs / 20), level.MeasuredFrequencyHz,
                sampleRate / (double)frequencyWindow)
        };
        lg = lg with { Ch1 = Refine(lg.Ch1, left), Ch2 = Refine(lg.Ch2, right) };

        var thd = AnalyzeConfiguredThdN(left, right, sampleRate, generator, filterConfig);

        double weightedCh1 = ComputeFullSignalWeightedRmsDbfs(left, sampleRate, filterConfig, lg.Ch1.RmsDbfs);
        double weightedCh2 = ComputeFullSignalWeightedRmsDbfs(right, sampleRate, filterConfig, lg.Ch2.RmsDbfs);
        // 开启发生器后，增益读数由未加权 RMS 修正为相应的加权电平，通道间差值也使用同一对读数。
        if (generator.IsEnabled)
        {
            lg = lg with
            {
                Ch1 = lg.Ch1 with { GainDb = lg.Ch1.GainDb + weightedCh1 - lg.Ch1.RmsDbfs },
                Ch2 = lg.Ch2 with { GainDb = lg.Ch2.GainDb + weightedCh2 - lg.Ch2.RmsDbfs },
                ChannelGainDeltaDb = weightedCh2 - weightedCh1
            };
        }

        // 对每个声道按设置选带通中心。固定值、实测频率和发生器 Hz 的优先级决定窄带仪表会看到哪段能量。
        double Center(double measured) => filterConfig.BandpassTuneMode switch
        {
            NotchTuningMode.Fixed when filterConfig.BandpassCenterHz > 10.0 => filterConfig.BandpassCenterHz,
            NotchTuningMode.Measured => measured > 10.0 ? measured : generator.FrequencyHz,
            _ when filterConfig.BandpassCenterHz > 10.0 => filterConfig.BandpassCenterHz,
            _ => (generator.IsEnabled && (generator.Waveform == GeneratorWaveform.Sine || generator.Waveform == GeneratorWaveform.Square))
                ? generator.FrequencyHz
                : (measured > 10.0 ? measured : generator.FrequencyHz)
        };
        var bp = AnalyzeBandpassLevel(left, right, sampleRate, Center(lg.Ch1.MeasuredFrequencyHz),
            filterConfig.BandpassQ, filterConfig.BandpassSelectivity, Center(lg.Ch2.MeasuredFrequencyHz));
        // APx Bench reports the d2 product for CCIF and DFD (User's Manual, p. 775).
        var imd = AnalyzeImdRatio(left, right, sampleRate, generator.Waveform, filterConfig.ImdStandard, ImdMeasurementMode.D2);

        return new BenchMeterSnapshot(lg, thd, weightedCh1, weightedCh2, bp, imd);
    }

    // 对整段 PCM 估计经主通路带宽和计权后的 RMS，结果按满幅正弦有效值换成 dBFS。
    /// <summary>计算施加 HP4 / LP8(或 AES17) 带宽与任意标准计权后的全信号 RMS 电平（dBFS）。</summary>
    public static double ComputeFullSignalWeightedRmsDbfs(
        ReadOnlySpan<double> samples,
        int sampleRate,
        AnalyzerFilterConfig filterConfig,
        double? precomputedUnfilteredAcRmsDbfs = null)
    {
        bool isCompletelyBypassed =
            filterConfig.Weighting == AnalyzerWeighting.Flat &&
            filterConfig.HighPassHz <= 0.0 &&
            filterConfig.LowPassHz <= 0.0;

        // 无高通且计权允许直流时，直流能量计入读数。短记录不足以频域积分时按是否通过直流选择时域回退。
        bool passesDc = filterConfig.HighPassHz <= 0 && filterConfig.Weighting is
            AnalyzerWeighting.Flat or AnalyzerWeighting.Deemphasis50us or AnalyzerWeighting.Deemphasis75us;
        if (isCompletelyBypassed || (samples.Length < 64 && passesDc))
            return FullRmsDbfs(samples);

        if (samples.Length < 64)
            return precomputedUnfilteredAcRmsDbfs ?? ComputeUnfilteredAcRmsDbfs(samples);

        // 快照的时域滤波从空状态开始。启用高通时先去记录的窗加权直流，减小偏置产生的启动暂态。
        double dc = filterConfig.HighPassHz > 0 ? ComputeWindowedDcMean(samples) : 0;
        var filtered = new double[samples.Length];
        for (int i = 0; i < samples.Length; i++)
            filtered[i] = samples[i] - dc;

        if (filterConfig.HighPassHz > 0.0)
            ApplyButterworthInPlace(filtered, sampleRate, filterConfig.HighPassHz, order: 4, isHighPass: true);

        // 八阶 Butterworth / Elliptic8 低通在时域执行，AES17 在后面的频域路径乘功率曲线，避免重复施加同一低通。
        bool applyTimeDomainLp = filterConfig.LowPassHz > 0.0
            && filterConfig.LowPassType is LowPassFilterType.Butterworth8 or LowPassFilterType.Elliptic8;
        if (applyTimeDomainLp)
        {
            if (filterConfig.LowPassType == LowPassFilterType.Elliptic8)
            {
                double rel = ApxEllipticLowPass.RelativeCutoff(filterConfig.LowPassHz, sampleRate);
                var (ellipLp, _) = ApxEllipticLowPass.CreateStereo(rel, sampleRate);
                ellipLp.ProcessInPlace(filtered);
            }
            else
            {
                ApplyButterworthInPlace(filtered, sampleRate, filterConfig.LowPassHz, order: 8, isHighPass: false);
            }
        }

        // 本次新建的高通或低通要经过启动过程，最多跳过首四分之一记录、上限 4096 点，再对余下部分测 RMS。
        int settleSkip = (filterConfig.HighPassHz > 0.0 || applyTimeDomainLp)
            ? Math.Min(filtered.Length / 4, 4096)
            : 0;
        var steady = filtered.AsSpan(settleSkip);

        bool needFreqDomainWeighting = filterConfig.Weighting != AnalyzerWeighting.Flat ||
                                       (filterConfig.LowPassHz > 0.0 && filterConfig.LowPassType == LowPassFilterType.Aes17);

        // 无额外频域权重时，把样本乘 Hann 窗后平方累加。除以窗平方和保持功率标定，乘 √2 对齐正弦 dBFS 基准。
        if (!needFreqDomainWeighting)
        {
            double weightedSq = 0.0;
            double weightSqSum = 0.0;
            double twoPiOverN = 2.0 * Math.PI / steady.Length;
            for (int i = 0; i < steady.Length; i++)
            {
                double w = 0.5 - 0.5 * Math.Cos(twoPiOverN * i);
                double xw = steady[i] * w;
                weightedSq += xw * xw;
                weightSqSum += w * w;
            }
            double rmsAes17 = Math.Sqrt(weightedSq / Math.Max(weightSqSum, 1e-12)) * Sqrt2;
            return SignalGeneratorDsp.LinearToDbfs(rmsAes17);
        }

        // 需要按频率加权时改做 FFT。若剩余长度仍不足 512 点，返回按直流通断计算的简化电平。
        int fftSize = ChooseLargestPowerOfTwo(steady.Length, 16384);
        if (fftSize < 512)
            return passesDc ? FullRmsDbfs(steady) : ComputeUnfilteredAcRmsDbfs(steady);

        // 每个正频格的 |X|² 以窗均方和 N² 校正，系数 4 同时计入负频率一侧和满幅正弦功率基准。
        var (_, _, halfSpec, window) = FftEngine.ComputeSingleSidedSpectrum(steady, fftSize, FftWindowType.BlackmanHarris4, removeDcOffset: true);
        double binHz = (double)sampleRate / fftSize;
        double powerScale = 4.0 / ((double)fftSize * fftSize * window.NoisePowerGain);

        // 已去均值的谱只做交流积分。若直流应通过，则另加 2·DC²，因满幅正弦在样本单位的均方为 1/2。
        double dcLevel = passesDc ? ComputeWindowedDcMean(steady) : 0;
        double powerSum = 2 * dcLevel * dcLevel;
        for (int k = 1; k < halfSpec.Length - 1; k++)
        {
            double f = k * binHz;
            double mag = halfSpec[k].Magnitude;
            double gain = ComputeWeightingPowerGain(f, filterConfig.Weighting);
            if (filterConfig.LowPassHz > 0.0 && filterConfig.LowPassType == LowPassFilterType.Aes17)
                gain *= ComputeAes17LowpassPowerGain(f, filterConfig.LowPassHz);
            powerSum += mag * mag * powerScale * gain;
        }

        return powerSum > 1e-18 ? 10.0 * Math.Log10(powerSum) : -180.0;
    }

    // 两路分别测指定频带内的电平。测量中心可以分别跟踪，故返回值保留实际采用的两个 Hz。
    /// <summary>
    /// 窄带带通电平（Bandpass Level）测量：支持 APx500 6 阶 Butterworth（1/3、1/6、1/12、1/24 倍频程）及二阶 Q 带通。
    /// </summary>
    public static BandpassLevelResult AnalyzeBandpassLevel(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int sampleRate,
        double centerHz,
        double q = 10.0,
        BandpassSelectivity selectivity = BandpassSelectivity.SecondOrderQ10, double? ch2CenterHz = null)
    {
        double fc = Math.Clamp(centerHz, 10.0, sampleRate * 0.45);
        double qSafe = Math.Clamp(q, 0.7, 50.0);
        double ch1 = MeasureSingleChannelBandpassDbfs(left, sampleRate, fc, qSafe, selectivity);
        double fc2 = Math.Clamp(ch2CenterHz ?? fc, 10.0, sampleRate * .45);
        double ch2 = MeasureSingleChannelBandpassDbfs(right, sampleRate, fc2, qSafe, selectivity);
        return new BandpassLevelResult(fc, ch1, ch2) { Ch2CenterFrequencyHz = fc2 };
    }

    // 对当前记录的正频率能量逐格乘带通功率响应并求和。结果反映有限窗内的频带功率，无递推历史。
    private static double MeasureSingleChannelBandpassDbfs(
        ReadOnlySpan<double> samples, int sampleRate, double fc, double q, BandpassSelectivity selectivity)
    {
        int fftSize = ChooseLargestPowerOfTwo(samples.Length, 16384);
        if (fftSize < 512) return -180.0;

        var (_, _, halfSpec, window) = FftEngine.ComputeSingleSidedSpectrum(samples, fftSize, FftWindowType.BlackmanHarris4, removeDcOffset: true);
        double binHz = (double)sampleRate / fftSize;
        double powerScale = 4.0 / ((double)fftSize * fftSize * window.NoisePowerGain);

        double powerSum = 0.0;
        double q2 = q * q;
        int octaveFrac = (int)selectivity;
        for (int k = 1; k < halfSpec.Length - 1; k++)
        {
            double f = k * binHz;
            // 分数倍频程采用预畸变的多阶曲线，固定 Q 采用二阶响应。中心处 f/fc−fc/f=0，峰值增益为 1。
            double bpGainSq = octaveFrac > 0
                ? ComputeButterBandpassPowerGain(f, fc, octaveFrac, sampleRate)
                : 1.0 / (1.0 + q2 * Math.Pow((f / fc) - (fc / f), 2));
            double mag = halfSpec[k].Magnitude;
            powerSum += mag * mag * powerScale * bpGainSq;
        }

        return powerSum > 1e-18 ? 10.0 * Math.Log10(powerSum) : -180.0;
    }

    // 左右通道按相同激励提示和标准各分析一次，输出各自的有效性、百分比及相对于参考音的 dB。
    /// <summary>
    /// 互调失真（IMD Ratio）测量：支持任意双音自动寻峰的 SMPTE（包络解调）、CCIF（单载波参考）与 DFD（双载波和参考）及 d2/d3/d23/d25 阶次组合。
    /// </summary>
    public static ImdResult AnalyzeImdRatio(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int sampleRate,
        GeneratorWaveform waveformHint = GeneratorWaveform.ImdSmpte,
        ImdMeasurementStandard standard = ImdMeasurementStandard.Auto,
        ImdMeasurementMode mode = ImdMeasurementMode.D23)
    {
        var ch1 = MeasureSingleChannelImd(left, sampleRate, waveformHint, standard, mode);
        var ch2 = MeasureSingleChannelImd(right, sampleRate, waveformHint, standard, mode);
        return new ImdResult(ch1, ch2);
    }

    // 用 FFT 找到输入的两个主要测试音。再根据测试类型测包络调制或指定组合频率上的失真分量。
    private static ImdChannelResult MeasureSingleChannelImd(
        ReadOnlySpan<double> samples,
        int sampleRate,
        GeneratorWaveform waveformHint,
        ImdMeasurementStandard standard,
        ImdMeasurementMode mode)
    {
        int fftSize = ChooseLargestPowerOfTwo(samples.Length, 65536);
        // DIM 的输入和权重不同，直接走专用分析。其他双音模式先检查长度，防止过粗频格混淆邻近产物。
        if (standard == ImdMeasurementStandard.DimEmulation || (standard == ImdMeasurementStandard.Auto && DimStimulus.IsDim(waveformHint)))
            return DimEmulation.Analyze(samples, sampleRate, fftSize);
        if (fftSize < 1024)
            return new ImdChannelResult(false, standard.ToString(), double.NaN, double.NaN, "Insufficient samples");

        var (linPeak, _, halfSpec, window) = FftEngine.ComputeSingleSidedSpectrum(samples, fftSize, FftWindowType.BlackmanHarris4, removeDcOffset: true);
        double binHz = (double)sampleRate / fftSize;
        double powerScale = 4.0 / ((double)fftSize * fftSize * window.NoisePowerGain);

        // 每个目标频率合计主瓣附近若干格的功率。越出有效带的组合频率返回零，避免把折叠或无效频率当作产物。
        double TonePowerFrom(Complex[] spec, double fHz, int halfSpanBins = 4)
        {
            if (fHz <= 5.0 || fHz >= sampleRate * 0.485) return 0.0;
            int center = (int)Math.Round(fHz / binHz);
            int lo = Math.Max(1, center - halfSpanBins);
            int hi = Math.Min(spec.Length - 2, center + halfSpanBins);
            double sum = 0.0;
            for (int k = lo; k <= hi; k++)
            {
                double m = spec[k].Magnitude;
                sum += m * m * powerScale;
            }
            return sum;
        }
        double TonePower(double fHz, int halfSpanBins = 4) => TonePowerFrom(halfSpec, fHz, halfSpanBins);

        // 自动检测两个最强非相邻主峰（支持任意频率 SMPTE / CCIF / DFD 双音，对齐 APx500 PeakBin + PeakBin2）
        int minBin = Math.Max(2, (int)Math.Ceiling(20.0 / binHz));
        int maxBin = Math.Min(halfSpec.Length - 3, (int)Math.Floor(sampleRate * 0.47 / binHz));
        int peak1 = -1;
        double peak1Val = 0.0;
        // 先找带内最大谱峰，第二音必须是相隔一定频格的局部峰，减小将同一主瓣的边缘认作第二音的机会。
        for (int k = minBin; k <= maxBin; k++)
        {
            if (linPeak[k] > peak1Val) { peak1Val = linPeak[k]; peak1 = k; }
        }
        int peak2 = -1;
        double peak2Val = 0.0;
        // Allow the documented 80 Hz DFD spacing when the FFT resolves both peaks.
        int minSepBins = 6;
        for (int k = minBin; k <= maxBin; k++)
        {
            if (Math.Abs(k - peak1) < minSepBins || linPeak[k]<linPeak[k-1] || linPeak[k]<=linPeak[k+1]) continue;
            if (linPeak[k] > peak2Val) { peak2Val = linPeak[k]; peak2 = k; }
        }

        if (peak1 < 0 || peak2 < 0 || peak1Val < 1e-4 || peak2Val < 1e-4)
            return new ImdChannelResult(false, standard == ImdMeasurementStandard.Auto ? "IMD" : standard.ToString(), double.NaN, double.NaN, "Need two prominent tones");

        // 按频率给两音排序并用峰顶插值求 Hz。互调产物如 f高−f低 依赖真实两音之差，格间修正也会影响其位置。
        int loBin = Math.Min(peak1, peak2);
        int hiBin = Math.Max(peak1, peak2);
        (double fLow, _) = FftEngine.RefinePeakParabolic(linPeak, loBin, sampleRate, fftSize);
        (double fHigh, _) = FftEngine.RefinePeakParabolic(linPeak, hiBin, sampleRate, fftSize);

        // 决定采用 SMPTE、CCIF 还是 DFD
        ImdMeasurementStandard resolved = standard;
        // 显式标准直接采用；自动模式优先用发生器提示，其余按两峰频率比推定相距较远或较近的双音。
        if (resolved == ImdMeasurementStandard.Auto)
        {
            if (waveformHint == GeneratorWaveform.ImdCcif) resolved = ImdMeasurementStandard.Ccif;
            else if (waveformHint == GeneratorWaveform.ImdSmpte) resolved = ImdMeasurementStandard.Smpte;
            else resolved = (fHigh / Math.Max(fLow, 1.0) >= 3.5) ? ImdMeasurementStandard.Smpte : ImdMeasurementStandard.Ccif;
        }

        // SMPTE 需要分离的低、高音。取高频载波的包络波动相对其平均幅度来表征调制。
        if (resolved == ImdMeasurementStandard.Smpte)
        {
            double pLow = TonePower(fLow);
            double pHigh = TonePower(fHigh);
            if (pLow < 1e-8 || pHigh < 1e-8 || fHigh / Math.Max(fLow, 1.0) < 2.5)
                return new ImdChannelResult(false, "SMPTE", double.NaN, double.NaN, "Need low+high SMPTE tones");

            // 动态围绕实际检测到的高频载波 fHigh ± 500 Hz 做解析包络解调（对齐 APx500 Smpte.Compute）
            double ratio = MeasureSmpteEnvelopeRatio(halfSpec, fftSize, sampleRate, fHigh);
            string lbl = $"SMPTE {fLow:F0}+{fHigh / 1000.0:0.#}k";
            return new ImdChannelResult(true, lbl, ratio * 100.0, 20.0 * Math.Log10(Math.Max(ratio, 1e-9)));
        }
        else if (resolved == ImdMeasurementStandard.Mod)
        {
            // APx500 IMDType.MOD (Mod.Compute / Dfd-helper): 时域扣除两个主载波后再作第二次 FFT 积分边带 fHigh ± k·fLow (k=1..4)，以高频载波功率为参考。
            double pLow = TonePower(fLow);
            double pHigh = TonePower(fHigh);
            if (pLow < 1e-8 || pHigh < 1e-8 || fHigh / Math.Max(fLow, 1.0) < 2.5)
                return new ImdChannelResult(false, "MOD", double.NaN, double.NaN, "Need low+high MOD tones");

            var analysisSamples = samples[^fftSize..];
            var fitLow = SineFit.Fit(analysisSamples, fLow / binHz, window.Coefficients, fixedFrequency: false);
            var afterLow = fitLow.Residual(analysisSamples);
            var fitHigh = SineFit.Fit(afterLow, fHigh / binHz, window.Coefficients, fixedFrequency: false);
            var residual = fitHigh.Residual(afterLow);
            var (_, _, resSpec, _) = FftEngine.ComputeSingleSidedSpectrum(residual, fftSize, FftWindowType.BlackmanHarris4, removeDcOffset: false);

            double fLowExact = fitLow.FrequencyBins * binHz;
            double fHighExact = fitHigh.FrequencyBins * binHz;

            // APx500 IMDType.MOD (Mod.Compute): 成对边带按幅度相加（使 m% 调幅正弦的 d2 读数精确等于 m%），再按阶次 RSS 合成。
            double d2Amp = Math.Sqrt(TonePowerFrom(resSpec, fHighExact - fLowExact)) + Math.Sqrt(TonePowerFrom(resSpec, fHighExact + fLowExact));
            double d3Amp = Math.Sqrt(TonePowerFrom(resSpec, fHighExact - 2.0 * fLowExact)) + Math.Sqrt(TonePowerFrom(resSpec, fHighExact + 2.0 * fLowExact));
            double d4Amp = Math.Sqrt(TonePowerFrom(resSpec, fHighExact - 3.0 * fLowExact)) + Math.Sqrt(TonePowerFrom(resSpec, fHighExact + 3.0 * fLowExact));
            double d5Amp = Math.Sqrt(TonePowerFrom(resSpec, fHighExact - 4.0 * fLowExact)) + Math.Sqrt(TonePowerFrom(resSpec, fHighExact + 4.0 * fLowExact));
            double imdPower = mode switch
            {
                ImdMeasurementMode.D2 => d2Amp * d2Amp,
                ImdMeasurementMode.D3 => d3Amp * d3Amp,
                ImdMeasurementMode.D23 => d2Amp * d2Amp + d3Amp * d3Amp,
                ImdMeasurementMode.D25 => d2Amp * d2Amp + d3Amp * d3Amp + d4Amp * d4Amp + d5Amp * d5Amp,
                _ => d2Amp * d2Amp + d3Amp * d3Amp
            };
            double ratio = Math.Sqrt(Math.Max(imdPower, 0.0) / pHigh);
            string lbl = $"MOD {fLowExact:F0}+{fHighExact / 1000.0:0.#}k ({mode.ToString().ToLowerInvariant()})";
            return new ImdChannelResult(true, lbl, ratio * 100.0, 20.0 * Math.Log10(Math.Max(ratio, 1e-15)));
        }
        else if (resolved == ImdMeasurementStandard.Iec60118)
        {
            // APx500 Imd.MeasureIec60118: Dfd.Compute([D2, D3minus]) * 2.0 (fHigh - fLow 与 2·fLow - fHigh，以高载波幅值为参考)
            double pLow = TonePower(fLow);
            double pHigh = TonePower(fHigh);
            if (pLow < 1e-8 || pHigh < 1e-8)
                return new ImdChannelResult(false, "IEC60118", double.NaN, double.NaN, "Need dual tones");

            var analysisSamples = samples[^fftSize..];
            var fitLow = SineFit.Fit(analysisSamples, fLow / binHz, window.Coefficients, fixedFrequency: false);
            var afterLow = fitLow.Residual(analysisSamples);
            var fitHigh = SineFit.Fit(afterLow, fHigh / binHz, window.Coefficients, fixedFrequency: false);
            var residual = fitHigh.Residual(afterLow);
            var (_, _, resSpec, _) = FftEngine.ComputeSingleSidedSpectrum(residual, fftSize, FftWindowType.BlackmanHarris4, removeDcOffset: false);

            double fLowExact = fitLow.FrequencyBins * binHz;
            double fHighExact = fitHigh.FrequencyBins * binHz;
            double d2Pow = TonePowerFrom(resSpec, fHighExact - fLowExact);
            double d3MinusPow = TonePowerFrom(resSpec, 2.0 * fLowExact - fHighExact);
            double refAmp = Math.Sqrt(pHigh);
            double ratio = Math.Sqrt(Math.Max(d2Pow + d3MinusPow, 0.0)) / Math.Max(refAmp, 1e-12);
            string lbl = $"IEC60118 {fLowExact / 1000.0:0.#}+{fHighExact / 1000.0:0.#}k";
            return new ImdChannelResult(true, lbl, ratio * 100.0, 20.0 * Math.Log10(Math.Max(ratio, 1e-15)));
        }
        else
        {
            double pLow = TonePower(fLow);
            double pHigh = TonePower(fHigh);
            if (pLow < 1e-8 || pHigh < 1e-8)
                return new ImdChannelResult(false, resolved.ToString().ToUpperInvariant(), double.NaN, double.NaN, "Need dual high-frequency tones");

            // 差频与两倍差频分别对应偶次产物；基频附近成对的三、五次分量先按本实现的幅度规则合并。
            double df = fHigh - fLow;
            double d2Pow = TonePower(df);
            double d4Pow = TonePower(2.0 * df);
            double d3Amp = Math.Sqrt(TonePower(2.0 * fLow - fHigh)) + Math.Sqrt(TonePower(2.0 * fHigh - fLow));
            double d5Amp = Math.Sqrt(TonePower(3.0 * fLow - 2.0 * fHigh)) + Math.Sqrt(TonePower(3.0 * fHigh - 2.0 * fLow));

            // 根据 D2、D3、D23、D25 选择参与积分的阶次，功率总和开方后才与参考音做幅度比较。
            double imdPower = mode switch
            {
                ImdMeasurementMode.D2 => d2Pow,
                ImdMeasurementMode.D3 => d3Amp * d3Amp,
                ImdMeasurementMode.D23 => d2Pow + d3Amp * d3Amp,
                ImdMeasurementMode.D25 => d2Pow + d4Pow + d3Amp * d3Amp + d5Amp * d5Amp,
                _ => d2Pow + d3Amp * d3Amp
            };

            // APx500 CCIF 以高载波幅值为参考；DFD（IEC 60268-3）在同式基础上再乘 0.5（以双载波和 2·A_high 为参考）
            // 分母取高音功率。此处 DFD 的参考约定还将幅度比乘 1/2，最后同一比值分别以百分比、dB 表达。
            double ratio = Math.Sqrt(Math.Max(imdPower, 1e-24) / pHigh);
            if (resolved == ImdMeasurementStandard.Dfd) ratio *= 0.5;

            string tag = resolved == ImdMeasurementStandard.Dfd ? "DFD" : "CCIF";
            string lbl = $"{tag} {fLow / 1000.0:0.#}+{fHigh / 1000.0:0.#}k ({mode.ToString().ToLowerInvariant()})";
            return new ImdChannelResult(true, lbl, ratio * 100.0, 20.0 * Math.Log10(Math.Max(ratio, 1e-9)));
        }
    }

    // 用包络解调估计高频音被低频音调制多少。输入是已加窗的单边复谱，保留载波附近边带的相位。
    private static double MeasureSmpteEnvelopeRatio(Complex[] spectrum, int fftSize, int sampleRate, double carrierHz)
    {
        double binHz = (double)sampleRate / fftSize;
        var analytic = new Complex[fftSize];
        double halfBw = 500.0;
        // 只复制载波周围的正频率并加倍，逆变换得解析信号。其复数模随时间的变化给出幅度包络。
        int first = Math.Max(1, (int)Math.Floor((carrierHz - halfBw) / binHz) - 4);
        int last = Math.Min(spectrum.Length - 2, (int)Math.Ceiling((carrierHz + halfBw) / binHz) + 4);
        for (int k = first; k <= last; k++) analytic[k] = 2.0 * spectrum[k];
        FftEngine.InverseInPlace(analytic);
        var envelope = new double[fftSize];
        for (int i = 0; i < fftSize; i++) envelope[i] = analytic[i].Magnitude;
        var (_, _, envelopeSpectrum, _) = FftEngine.ComputeSingleSidedSpectrum(
            envelope, fftSize, FftWindowType.BlackmanHarris4, removeDcOffset: false);

        // The doubly windowed envelope occupies several DC bins. APx integrates
        // bins 0..3 for the carrier and starts modulation integration at bin 8.
        // 包络的直流主瓣代表载波基线，较远频格包含调制。留出过渡频格以降低加窗泄漏对调制积分的影响。
        double dcPower = 0.5 * envelopeSpectrum[0].Magnitude * envelopeSpectrum[0].Magnitude;
        for (int k = 1; k <= 3; k++) dcPower += envelopeSpectrum[k].Magnitude * envelopeSpectrum[k].Magnitude;
        double modulationPower = 0.0;
        for (int k = 8; k < envelopeSpectrum.Length - 1; k++)
            modulationPower += envelopeSpectrum[k].Magnitude * envelopeSpectrum[k].Magnitude;
        return dcPower > 1e-24 ? Math.Sqrt(2.0 * modulationPower / dcPower) : double.NaN;
    }

    // 简化入口假设两路发生器电平相同，输入和输出满量程相同；完整入口允许分别提供这些校准量。
    /// <summary>执行 Level &amp; Gain 测量分析（向后兼容单电平单标定签名）。</summary>
    public static LevelAndGainResult AnalyzeLevelAndGain(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int sampleRate,
        double generatorLevelDbfs,
        bool generatorEnabled,
        double fullScaleVoltageVrms) =>
        AnalyzeLevelAndGain(left, right, sampleRate, generatorLevelDbfs, generatorLevelDbfs, generatorEnabled, fullScaleVoltageVrms, fullScaleVoltageVrms);

    // 对齐两通道的 FFT 长度，再分别求电平；相位比较必须在同一频率格上进行。
    /// <summary>执行 Level &amp; Gain 测量分析（支持独立双通道发生器电平与独立输入/输出满幅电压标定）。</summary>
    public static LevelAndGainResult AnalyzeLevelAndGain(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int sampleRate,
        double genCh1LevelDbfs,
        double genCh2LevelDbfs,
        bool generatorEnabled,
        double inputFullScaleVoltageVrms,
        double outputFullScaleVoltageVrms)
    {
        int fftSize = ChooseLargestPowerOfTwo(Math.Min(left.Length, right.Length), 16384);
        double calOffsetDb = ComputeCalibrationGainOffsetDb(inputFullScaleVoltageVrms, outputFullScaleVoltageVrms);

        var ch1 = MeasureSingleChannelLevel(left, sampleRate, fftSize, genCh1LevelDbfs, calOffsetDb, generatorEnabled, inputFullScaleVoltageVrms, out var spec1, out int peakBin1);
        var ch2 = MeasureSingleChannelLevel(right, sampleRate, fftSize, genCh2LevelDbfs, calOffsetDb, generatorEnabled, inputFullScaleVoltageVrms, out var spec2, out _);

        double phaseDeg = 0.0;
        // 复交叉积 X₂·conj(X₁) 的相角为第二路减第一路的相位。两路都超过电平门限才更新该估计。
        if (spec1 is not null && spec2 is not null && peakBin1 > 0 && peakBin1 < spec1.Length && ch1.RmsDbfs > -80.0 && ch2.RmsDbfs > -80.0)
        {
            Complex cross = spec2[peakBin1] * Complex.Conjugate(spec1[peakBin1]);
            phaseDeg = Math.Atan2(cross.Imaginary, cross.Real) * (180.0 / Math.PI);
        }

        double deltaDb = ch2.RmsDbfs - ch1.RmsDbfs;
        return new LevelAndGainResult(ch1, ch2, phaseDeg, deltaDb);
    }

    // 从一个声道提取绝对峰值、偏置、交流 RMS、增益和粗测频率，复频谱交给上层求通道间相位。
    private static ChannelLevelResult MeasureSingleChannelLevel(
        ReadOnlySpan<double> samples,
        int sampleRate,
        int fftSize,
        double generatorLevelDbfs,
        double ioCalibrationGainOffsetDb,
        bool generatorEnabled,
        double inputFullScaleVoltageVrms,
        out Complex[]? halfSpec,
        out int peakBin)
    {
        halfSpec = null;
        peakBin = 0;

        if (samples.Length == 0)
            return new ChannelLevelResult(-180.0, 0.0, -180.0, 0.0, 0.0, false, 0.0, 0.0, -180.0);

        // 先在包含直流的原波形找最高绝对值，再用窗加权均值估计偏置，以区分过载和交流峰值。
        double rawPeak = 0.0;
        for (int i = 0; i < samples.Length; i++)
        {
            double abs = Math.Abs(samples[i]);
            if (abs > rawPeak) rawPeak = abs;
        }
        double dcMean = ComputeWindowedDcMean(samples);

        double sumSq = 0.0;
        double acPeak = 0.0;
        // 减偏置后求平方和与最大绝对值，分别给出交流有效值和峰值，后者与 RMS 的比值形成波峰因数。
        for (int i = 0; i < samples.Length; i++)
        {
            double ac = samples[i] - dcMean;
            sumSq += ac * ac;
            double absAc = Math.Abs(ac);
            if (absAc > acPeak) acPeak = absAc;
        }
        double rmsLinear = Math.Sqrt(sumSq / samples.Length);

        // 正弦满幅峰值为 1、RMS 为 1/√2，乘 √2 后满幅正弦读 0 dBFS。伏特值还取决于输入量程。
        double rmsAes17Linear = rmsLinear * Sqrt2;
        double rmsDbfs = SignalGeneratorDsp.LinearToDbfs(rmsAes17Linear);
        double acPeakDbfs = SignalGeneratorDsp.LinearToDbfs(acPeak);
        double rawPeakDbfs = SignalGeneratorDsp.LinearToDbfs(rawPeak);
        double rmsVrms = rmsAes17Linear * inputFullScaleVoltageVrms;
        double gainDb = generatorEnabled ? (rmsDbfs - generatorLevelDbfs + ioCalibrationGainOffsetDb) : 0.0;
        bool isClipping = rawPeak >= 0.999;
        double crestFactorDb = rmsLinear > 1e-9 ? 20.0 * Math.Log10(Math.Max(acPeak / rmsLinear, 1.0)) : 0.0;

        double measuredFreq = 0.0;
        // 仅在长度和电平均足够时寻找主音。搜索排除直流与奈奎斯特附近，最高格再用邻点插值得 Hz。
        if (fftSize >= 256 && rmsDbfs > -90.0)
        {
            var (linPeak, _, spec, _) = FftEngine.ComputeSingleSidedSpectrum(samples, fftSize, FftWindowType.Hann);
            halfSpec = spec;

            int minBin = Math.Max(2, (int)Math.Floor(10.0 * fftSize / sampleRate));
            int maxBin = Math.Min(linPeak.Length - 2, (int)Math.Ceiling((sampleRate * 0.48) * fftSize / sampleRate));

            double bestVal = 0.0;
            int bestIdx = minBin;
            for (int k = minBin; k <= maxBin; k++)
            {
                if (linPeak[k] > bestVal) { bestVal = linPeak[k]; bestIdx = k; }
            }

            peakBin = bestIdx;
            (measuredFreq, _) = FftEngine.RefinePeakParabolic(linPeak, bestIdx, sampleRate, fftSize);
        }

        return new ChannelLevelResult(rmsDbfs, rmsVrms, acPeakDbfs, gainDb, measuredFreq, isClipping, dcMean, crestFactorDb, rawPeakDbfs);
    }

    // 将分析设置翻译成陷波初值及严格固定标志，再传给双通道 THD+N。发生器 Hz 有效也不等于输入实测 Hz。
    public static ThdNResult AnalyzeConfiguredThdN(ReadOnlySpan<double> left, ReadOnlySpan<double> right,
        int sampleRate, GeneratorConfig generator, AnalyzerFilterConfig filterConfig)
    {
        // 根据陷波调谐模式决定 THD+N 目标基波频率
        double? targetNotchHz = filterConfig.NotchTuningMode switch
        {
            NotchTuningMode.Generator when generator.IsEnabled && (generator.Waveform == GeneratorWaveform.Sine || generator.Waveform == GeneratorWaveform.Square)
                => generator.FrequencyHz,
            NotchTuningMode.Fixed when filterConfig.FixedNotchFrequencyHz > 10.0
                => filterConfig.FixedNotchFrequencyHz,
            _ => null // Measured 模式自动寻峰
        };

        return AnalyzeThdPlusN(
            left, right, sampleRate,
            targetNotchHz,
            bwLowHz: filterConfig.HighPassHz,
            bwHighHz: filterConfig.LowPassHz,
            residualWeighting: filterConfig.ResidualWeighting,
            strictFixedNotch: filterConfig.NotchTuningMode == NotchTuningMode.Fixed,
            signalWeighting: filterConfig.Weighting,
            lowPassType: filterConfig.LowPassType,
            thdReference: filterConfig.ThdReference);
    }

    // 使用两路共有的 FFT 长度和相同的频带约定，分别估计谐波、残余和相对电平。
    /// <summary>
    /// 执行双通道 THD+N 测量分析（支持严格固定频率陷波、独立输入/残余计权、Butterworth/AES17 低通与 APx 总 RMS / IEEE 基波参考）。
    /// </summary>
    public static ThdNResult AnalyzeThdPlusN(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int sampleRate,
        double? expectedFundamentalHz = null,
        double bwLowHz = 20.0,
        double bwHighHz = 20000.0,
        AnalyzerWeighting residualWeighting = AnalyzerWeighting.Flat,
        bool strictFixedNotch = false,
        AnalyzerWeighting signalWeighting = AnalyzerWeighting.Flat,
        LowPassFilterType lowPassType = LowPassFilterType.Butterworth8,
        ThdReferenceMode thdReference = ThdReferenceMode.TotalRmsIec)
    {
        double effectiveHigh = bwHighHz > 0.0 ? Math.Min(bwHighHz, sampleRate * 0.48) : sampleRate * 0.48;
        double effectiveLow = Math.Max(0.0, bwLowHz);
        int fftSize = ChooseLargestPowerOfTwo(Math.Min(left.Length, right.Length), 65536);

        var ch1 = MeasureSingleChannelThdN(left, sampleRate, fftSize, expectedFundamentalHz, effectiveLow, bwHighHz, effectiveHigh, residualWeighting, strictFixedNotch, signalWeighting, lowPassType, thdReference);
        var ch2 = MeasureSingleChannelThdN(right, sampleRate, fftSize, expectedFundamentalHz, effectiveLow, bwHighHz, effectiveHigh, residualWeighting, strictFixedNotch, signalWeighting, lowPassType, thdReference);

        return new ThdNResult(ch1, ch2, effectiveLow, effectiveHigh);
    }

    // 单声道步骤为定位基频、估计输入功率、拟合并减去正弦、积分残余和各次谐波，最终按所选参考求比。
    private static ThdNChannelResult MeasureSingleChannelThdN(
        ReadOnlySpan<double> samples,
        int sampleRate,
        int fftSize,
        double? expectedFundamentalHz,
        double hpCutoffHz,
        double lpCutoffHz,
        double displayBwHighHz,
        AnalyzerWeighting residualWeighting,
        bool strictFixedNotch,
        AnalyzerWeighting signalWeighting,
        LowPassFilterType lowPassType,
        ThdReferenceMode thdReference)
    {
        if (fftSize < 512)
            return CreateInvalidThdN("Insufficient samples");

        var (linPeak, _, halfSpec, window) = FftEngine.ComputeSingleSidedSpectrum(samples, fftSize, FftWindowType.BlackmanHarris4);

        double binHz = (double)sampleRate / fftSize;
        int minSearchBin = Math.Max(2, (int)Math.Ceiling(Math.Max(hpCutoffHz > 0 ? hpCutoffHz * 0.5 : 10.0, 10.0) / binHz));
        int maxSearchBin = Math.Min(halfSpec.Length - 2, (int)Math.Floor(displayBwHighHz / binHz));

        int fundBin;
        double fundFreqHz;

        // 严格固定频率时保留用户 Hz，最近整数格只用于定位。可跟踪时则在允许带宽内寻找真正的谱峰。
        if (strictFixedNotch && expectedFundamentalHz is > 10.0 && expectedFundamentalHz < displayBwHighHz)
        {
            // 固定陷波模式：强制在用户指定的中心频率处陷波（便于观察偏频或固定频率残余）
            fundBin = Math.Clamp((int)Math.Round(expectedFundamentalHz.Value / binHz), 1, halfSpec.Length - 2);
            fundFreqHz = expectedFundamentalHz.Value;
        }
        else
        {
            int searchStart = minSearchBin;
            int searchEnd = maxSearchBin;
            // 若已有预期频率，围绕它限定峰值搜索范围，降低较强的无关远端音调夺取基频的可能。
            if (expectedFundamentalHz is > 10.0 && expectedFundamentalHz < displayBwHighHz)
            {
                int center = (int)Math.Round(expectedFundamentalHz.Value / binHz);
                int span = Math.Max(6, (int)Math.Ceiling(center * 0.05));
                searchStart = Math.Max(minSearchBin, center - span);
                searchEnd = Math.Min(maxSearchBin, center + span);
            }

            fundBin = searchStart;
            double maxAmp = 0.0;
            for (int k = searchStart; k <= searchEnd; k++)
            {
                if (linPeak[k] > maxAmp) { maxAmp = linPeak[k]; fundBin = k; }
            }
            // 用主瓣内频格位置的功率加权平均估基频；单音落在两格之间时能量会分布到两侧。
            double sumPower = 0.0, sumBinPower = 0.0;
            int halfWidth = Math.Min(4, Math.Min(fundBin, halfSpec.Length - 1 - fundBin));
            for (int k = fundBin - halfWidth; k <= fundBin + halfWidth; k++)
            {
                double power = halfSpec[k].Magnitude * halfSpec[k].Magnitude;
                sumPower += power; sumBinPower += k * power;
            }
            fundFreqHz = sumPower > 1e-24 ? sumBinPower / sumPower * binHz : fundBin * binHz;
        }

        double powerScale = 4.0 / ((double)fftSize * fftSize * window.NoisePowerGain);
        const int fundamentalHalfWidth = 4;
        int fundLow = Math.Max(1, fundBin - fundamentalHalfWidth);
        int fundHigh = Math.Min(halfSpec.Length - 2, fundBin + fundamentalHalfWidth);

        double fundamentalPower = 0.0;
        double residualUnweightedPower = 0.0;
        double residualWeightedPower = 0.0;
        double totalRawInBandPower = 0.0;

        int integrationMaxBin = halfSpec.Length - 2;
        // 原始谱经过模拟的主通路带宽和计权后求总功率，基频主瓣与其余频格分别累计。
        for (int k = 1; k <= integrationMaxBin; k++)
        {
            double f = k * binHz;
            double mag = halfSpec[k].Magnitude;
            double rawBinPower = mag * mag * powerScale;
            double bandFilteredPower = rawBinPower * ComputeAnalyzerPowerGain(f, hpCutoffHz, lpCutoffHz, signalWeighting, sampleRate, lowPassType);
            totalRawInBandPower += bandFilteredPower;

            if (k >= fundLow && k <= fundHigh)
            {
                fundamentalPower += bandFilteredPower;
            }
            else
            {
                residualUnweightedPower += bandFilteredPower;
            }
        }

        double fundLevelDbfs = fundamentalPower > 1e-18 ? 10.0 * Math.Log10(fundamentalPower) : -180.0;
        double totalLevelDbfs = totalRawInBandPower > 1e-18 ? 10.0 * Math.Log10(totalRawInBandPower) : -180.0;

        // 自动寻峰时兼顾绝对电平、主瓣占比和相对背景的突出程度。纯噪声也有最高格，单凭最高格不足以认定基频。
        if (!strictFixedNotch)
        {
            double avgNoisePerBin = residualUnweightedPower / Math.Max(1, integrationMaxBin - (fundHigh - fundLow + 1));
            double peakBinPower = halfSpec[fundBin].Magnitude * halfSpec[fundBin].Magnitude * powerScale;
            double fundamentalEnergyShare = totalRawInBandPower > 1e-18 ? fundamentalPower / totalRawInBandPower : 0.0;
            bool hasProminentTone = fundamentalEnergyShare >= 0.05 && peakBinPower > Math.Max(1e-10, avgNoisePerBin * 20.0);
            if (fundLevelDbfs < -90.0 || !hasProminentTone)
                return CreateInvalidThdN(fundLevelDbfs < -90.0 ? "No Signal" : "No Fundamental");
        }
        // 固定陷波无需重新认定主音，但总电平仍须越过最低门限，才能报告可用读数。
        else if (totalLevelDbfs < -110.0)
        {
            return CreateInvalidThdN("No Signal");
        }

        // APx subtracts a reconstructed sine before transforming the residual.
        // Blanking bins in the original spectrum leaves off-bin fundamental
        // sidelobes and can hide a nearby low-frequency harmonic.
        var analysisSamples = samples[^fftSize..];
        // 设基波落在第 42.67 个频格。有限窗口会把它摊到邻格，只删一个格仍会把强基波漏算进残余。
        // 在这段样本上拟合正弦并逐点相减，再去统计差值的频谱，才能把这项泄漏压下去。
        // 在与 FFT 相同的尾段拟合基音与 DC，再对时域残余重新作 FFT。固定中心时禁止拟合器移动频率。
        var fit = SineFit.Fit(analysisSamples, fundFreqHz / binHz, window.Coefficients, strictFixedNotch);
        fundFreqHz = fit.FrequencyBins * binHz;
        var residual = fit.Residual(analysisSamples);
        var (_, _, residualSpectrum, _) = FftEngine.ComputeSingleSidedSpectrum(residual, fftSize, FftWindowType.BlackmanHarris4);
        residualWeightedPower = 0.0;
        // 残余的每一格依次计入陷波响应、主通路响应与专属残余计权，功率权重相乘后才积分。
        for (int k = 1; k <= integrationMaxBin; k++)
        {
            double power = residualSpectrum[k].Magnitude * residualSpectrum[k].Magnitude * powerScale;
            residualWeightedPower += power * ComputeThdNotchPowerGain(k, fit.FrequencyBins) *
                ComputeAnalyzerPowerGain(k * binHz, hpCutoffHz, lpCutoffHz, signalWeighting, sampleRate, lowPassType) *
                ComputeAnalyzerPowerGain(k * binHz, 0, 0, residualWeighting, sampleRate);
        }

        // THD+N 以总信号有效值为参考，功率比开方才是面板百分比的线性量；SINAD 则取其 dB 的相反数。
        double totalSignalPower = Math.Max(totalRawInBandPower, fundamentalPower);
        double thdNPower = Math.Max(residualWeightedPower, 1e-30);
        double thdNLevelDbfs = 10.0 * Math.Log10(thdNPower);
        double thdNRatio = Math.Sqrt(thdNPower / totalSignalPower);
        double thdNPercent = thdNRatio * 100.0;
        double thdNDb = 20.0 * Math.Log10(Math.Max(thdNRatio, 1e-15));
        double sinadDb = -thdNDb;

        var harmonics = new List<HarmonicComponent>(19);
        double totalHarmonicWeightedPower = 0.0;
        const int harmonicHalfWidth = 4;

        // 谐波出现在整数倍基频处。根据带宽和可用频格停止枚举，每阶用中心附近的主瓣功率估计。
        for (int order = 2; order <= 20; order++)
        {
            double hFreq = fundFreqHz * order;
            if (hFreq > displayBwHighHz) break;
            int hBin = (int)Math.Round(hFreq / binHz);
            if (hBin >= integrationMaxBin) break;

            int hStart = Math.Max(1, hBin - harmonicHalfWidth);
            int hEnd = Math.Min(integrationMaxBin, hBin + harmonicHalfWidth);

            double hRawPower = 0.0;
            double hWeightedPower = 0.0;
            for (int k = hStart; k <= hEnd; k++)
            {
                double f = k * binHz;
                double m = residualSpectrum[k].Magnitude;
                double p = m * m * powerScale;
                // 单阶显示含主通路滤波；形成合计失真时再加残余计权，因此两种读数对同一阶的权重可能不同。
                double signalPower = p * ComputeAnalyzerPowerGain(f, hpCutoffHz, lpCutoffHz, signalWeighting, sampleRate, lowPassType);
                hRawPower += signalPower;
                hWeightedPower += signalPower * ComputeAnalyzerPowerGain(f, 0, 0, residualWeighting, sampleRate);
            }

            // APx integrates harmonic bands without subtracting a guessed local
            // noise floor. Keep harmonic results independent of the THD+N notch.
            double hNetUnweighted = Math.Max(hRawPower, 1e-30);
            double hNetWeighted = Math.Max(hWeightedPower, 1e-30);

            totalHarmonicWeightedPower += hNetWeighted;

            double hLevelDbfs = 10.0 * Math.Log10(hNetUnweighted);
            harmonics.Add(new HarmonicComponent(order, hFreq, hLevelDbfs, hLevelDbfs - fundLevelDbfs));
        }

        // APx 默认以总 RMS 为分母（TotalRmsIec）；IEEE 标准则以基波 RMS 为分母（FundamentalIeee）
        // 同一组合信号可以回答两个问题。谐波相对基波有多大，用基波功率作分母；
        // 谐波占整个信号多少，用总功率作分母。这里的 THD 分子只累计谐波，宽带噪声则会进入上面的 THD+N。
        // THD 可按基频或总量作分母；噪声估计取残余功率扣除谐波功率，必须在线性功率域相减。
        double thdDenom = thdReference == ThdReferenceMode.FundamentalIeee ? fundamentalPower : totalSignalPower;
        double thdRatio = thdDenom > 1e-18 ? Math.Sqrt(Math.Max(totalHarmonicWeightedPower, 1e-30) / thdDenom) : double.NaN;
        double thdPercent = double.IsNaN(thdRatio) ? double.NaN : thdRatio * 100.0;
        double thdDb = double.IsNaN(thdRatio) ? double.NaN : 20.0 * Math.Log10(Math.Max(thdRatio, 1e-9));
        double noiseFloorDbfs = 10.0 * Math.Log10(Math.Max(thdNPower - totalHarmonicWeightedPower, 1e-30));

        return new ThdNChannelResult(
            fundFreqHz, fundLevelDbfs, thdNPercent, thdNDb, sinadDb, thdPercent, thdDb,
            noiseFloorDbfs, harmonics, HasValidSignal: true, InvalidReason: null, ThdNLevelDbfs: thdNLevelDbfs);
    }

    // 以 bin/基频格数给陷波响应定位。只在半倍至两倍中心之间使用形状模型，中心附近七格强制清零。
    /// <summary>APx500 ProcessingWorksheet residual notch power response (outside the central blanked bins).</summary>
    public static double ComputeThdNotchPowerGain(int bin, double fundamentalBin)
    {
        if (fundamentalBin <= 0 || bin < Math.Ceiling(fundamentalBin * 0.5) || bin > Math.Floor(fundamentalBin * 2.0))
            return 1.0;
        if (Math.Abs(bin - Math.Round(fundamentalBin)) <= 3) return 0.0;
        double relativeFrequency = bin / fundamentalBin;
        // 用归一化角频率求三节传递函数的模并平方，乘到各格的残余功率上，不改变远离中心处的功率。
        double omega = relativeFrequency * Math.Tau;
        double s2 = -relativeFrequency * relativeFrequency * Math.Tau * Math.Tau;
        double gain = Math.Pow(Math.Abs(s2 + 39.47860224), 3);
        gain /= new Complex(s2 + 19.5226, -1.4414 * omega).Magnitude;
        gain /= new Complex(s2 + 79.8613, -2.9152 * omega).Magnitude;
        gain /= new Complex(s2 + 39.4855, -5.65 * omega).Magnitude;
        return gain * gain;
    }

    // 缺失基音或记录太短时返回 NaN 与原因，谐波列表为空，调用方可据此区分无效测量和极低失真。
    private static ThdNChannelResult CreateInvalidThdN(string reason) => new(
        double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
        Array.Empty<HarmonicComponent>(), HasValidSignal: false, InvalidReason: reason, ThdNLevelDbfs: double.NaN);

    // 从两路同步缓冲中选可容纳完整 FFT 的窗口。绝对样本编号约束可重复触发的位置，本地偏移用于切片。
    /// <summary>
    /// 按触发模式、布防基线样本序号与触发延时严格截取 FFT 分析窗口。
    /// <para>
    /// - <b>RisingEdge 模式</b>：仅接受发生在 <paramref name="minTriggerSampleIndex"/>（布防时刻）之后的上升沿，
    ///   且必须满足触发点 + <paramref name="triggerDelaySamples"/> + <paramref name="fftSize"/> 均已到达；
    ///   若未满足条件则返回 <c>Triggered = false</c> 与空数组，绝不退回未触发数据污染单次采集。
    /// </para>
    /// </summary>
    public static (double[] LeftWindow, double[] RightWindow, bool Triggered, long TriggerSampleIndex) ExtractTriggeredFftWindow(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        long bufferEndSampleIndex,
        long minTriggerSampleIndex,
        int fftSize,
        FftTriggerMode triggerMode,
        double triggerThresholdLinear = 0.05,
        int triggerDelaySamples = 0,
        InputChannelSelection triggerChannel = InputChannelSelection.Channel1,
        double[]? generatorReference = null, GeneratorWaveform waveform = GeneratorWaveform.Sine, long[]? fileStarts = null)
    {
        int n = Math.Min(left.Length, right.Length);
        int delay = Math.Max(0, triggerDelaySamples);
        if (n < fftSize)
            return (Array.Empty<double>(), Array.Empty<double>(), false, -1);

        // 自由运行取最新完整窗口，但也要满足距上一触发的间隔与延时约束。
        if (triggerMode == FftTriggerMode.FreeRun)
        {
            int start = n - fftSize;
            if (bufferEndSampleIndex - fftSize < minTriggerSampleIndex + delay)
                return (Array.Empty<double>(), Array.Empty<double>(), false, -1);
            return (left.Slice(start, fftSize).ToArray(), right.Slice(start, fftSize).ToArray(), true, bufferEndSampleIndex);
        }

        // 正过零模式把延迟加到最早允许触发的编号；其他边沿模式在找到触发后才把窗口向后平移。
        if (triggerMode == FftTriggerMode.PositiveZeroCrossing)
        {
            minTriggerSampleIndex += delay;
            delay = 0;
        }
        int maxTriggerIdx = n - fftSize - delay;
        long first = bufferEndSampleIndex - n;
        // 发生器触发需要与输入对齐的参考。WAV 用文件起点标记，其余受支持的周期波形用参考上升过零。
        if (triggerMode == FftTriggerMode.Generator)
        {
            if (generatorReference?.Length != n) return ([], [], false, -1);
            if (waveform == GeneratorWaveform.WavPlayback && fileStarts != null)
                foreach (long marker in fileStarts)
                    if (marker >= minTriggerSampleIndex && marker >= first && marker <= first + maxTriggerIdx)
                    {
                        int offset = (int)(marker - first) + delay;
                        return (left.Slice(offset, fftSize).ToArray(), right.Slice(offset, fftSize).ToArray(), true, marker);
                    }
            if (waveform is not (GeneratorWaveform.Sine or GeneratorWaveform.Square)) return ([], [], false, -1);
        }
        if (maxTriggerIdx < 1)
            return (Array.Empty<double>(), Array.Empty<double>(), false, -1);

        int minLocalIdx = (int)Math.Max(1, minTriggerSampleIndex - first);

        // 在布防之后的有效区间内寻找最早（单次触发语义）或满足完整延时+窗口条件的上升沿
        // 对选定声道或发生器参考扫描首次合格边沿。为比较前一个样本从下标 1 起，同时预留后续完整 FFT 长度。
        ReadOnlySpan<double> trigger = triggerMode == FftTriggerMode.Generator ? generatorReference :
            triggerChannel == InputChannelSelection.Channel2 ? right : left;
        for (int i = minLocalIdx; i <= maxTriggerIdx; i++)
        {
            bool crossing = IsFftTriggerEdge(triggerMode, trigger[i - 1], trigger[i], triggerThresholdLinear);
            if (crossing)
            {
                int start = i + delay;
                long trigGlobalIdx = first + i;
                return (left.Slice(start, fftSize).ToArray(), right.Slice(start, fftSize).ToArray(), true, trigGlobalIdx);
            }
        }

        return (Array.Empty<double>(), Array.Empty<double>(), false, -1);
    }

    // 两样本跨越阈值才触发。上升、下降和过零模式对等于阈值的端点采用这里各自的比较规则。
    internal static bool IsFftTriggerEdge(FftTriggerMode mode, double previous, double current, double threshold) => mode switch
    {
        FftTriggerMode.Generator or FftTriggerMode.PositiveZeroCrossing => previous <= 0 && current > 0,
        FftTriggerMode.FallingEdge => previous > threshold && current <= threshold,
        _ => previous < threshold && current >= threshold
    };

    // 单记录入口以数组长度充当缓冲末端编号，只返回窗口和触发状态，适用于无需跨缓冲去重的调用。
    /// <summary>向后兼容的简化触发窗口截取重载。</summary>
    public static (double[] LeftWindow, double[] RightWindow, bool Triggered) ExtractTriggeredFftWindow(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int fftSize,
        FftTriggerMode triggerMode,
        double triggerThresholdLinear = 0.05,
        int triggerDelaySamples = 0)
    {
        var (l, r, trig, _) = ExtractTriggeredFftWindow(
            left, right, left.Length, 0, fftSize, triggerMode, triggerThresholdLinear, triggerDelaySamples);
        return (l, r, trig);
    }

    // 生成频谱显示、峰值标记和第一声道的派生读数。实际 FFT 长度决定横轴每格的 Hz。
    /// <summary>执行 Signal Analyzer 实时 FFT 频谱分析，同时输出 Top-5 标峰与派生指标（SFDR、噪声密度、PAR）。</summary>
    public static SpectrumAnalysisResult AnalyzeSpectrum(
        ReadOnlySpan<double> left,
        ReadOnlySpan<double> right,
        int sampleRate,
        int fftSize,
        FftWindowType windowType)
    {
        int available = Math.Min(left.Length, right.Length);
        // 请求长度有足够样本且处于支持范围内时直接采用，否则退到能容纳的 2 的幂长度。
        int effectiveFft = available >= fftSize && FftEngine.IsBenchLength(fftSize)
            ? fftSize : ChooseLargestPowerOfTwo(available, fftSize);
        if (effectiveFft < 256)
        {
            return new SpectrumAnalysisResult(
                Array.Empty<double>(), Array.Empty<double>(), Array.Empty<double>(),
                Array.Empty<SpectrumPeakMarker>(), 0, sampleRate, windowType, 0);
        }

        var (lin1, db1, halfSpec1, window) = FftEngine.ComputeBenchSpectrum(left, effectiveFft, windowType);
        var (lin2, db2, _, _) = FftEngine.ComputeBenchSpectrum(right, effectiveFft, windowType);

        int halfLen = db1.Length;
        double binHz = (double)sampleRate / effectiveFft;
        var freqs = new double[halfLen];
        for (int k = 0; k < halfLen; k++) freqs[k] = k * binHz;

        // 寻峰使用线性幅度；随后才把两路线性数组原地平方，作为显示结果里的峰值功率。
        var peaks = FindSpectrumPeaks(lin1, db1, sampleRate, effectiveFft);
        var derived = ComputeFftDerivedMetrics(halfSpec1, db1, peaks, sampleRate, effectiveFft, window);
        for (int i = 0; i < halfLen; i++)
        {
            lin1[i] *= lin1[i];
            lin2[i] *= lin2[i];
        }
        return new SpectrumAnalysisResult(freqs, db1, db2, peaks, effectiveFft, sampleRate, windowType, binHz, derived)
        {
            Ch1PeakPower = lin1,
            Ch2PeakPower = lin2
        };
    }

    // 找第一声道至多五个峰标记。先从 15 Hz 以上收集局部极大值，再按强弱排序并排除重复。
    public static IReadOnlyList<SpectrumPeakMarker> FindSpectrumPeaks(double[] lin1, double[] db1, int sampleRate, int effectiveFft)
    {
        int halfLen = db1.Length;
        double binHz = (double)sampleRate / effectiveFft;
        var candidates = new List<(int Bin, double Db)>();
        int startBin = Math.Max(1, (int)Math.Ceiling(15.0 / binHz));
        for (int k = startBin; k < halfLen - 1; k++)
        {
            if (db1[k] > -180 && db1[k] >= db1[k - 1] && db1[k] > db1[k + 1])
                candidates.Add((k, db1[k]));
        }

        candidates.Sort((a, b) => b.Db.CompareTo(a.Db));
        var peaks = new List<SpectrumPeakMarker>(5);
        var usedBins = new List<int>(5);

        foreach (var (bin, _) in candidates)
        {
            if (peaks.Count >= 5) break;
            // Close maxima separated by a measured valley are resolvable, even
            // when both occupy what used to be a single ten-bin marker region.
            // 很近的两个峰若中间谷底浅于 6 dB 就视为同一峰。谷底够深才将两个相邻峰都保留为候选音调。
            bool samePeak = usedBins.Any(other =>
            {
                if (Math.Abs(other-bin) >= 10) return false;
                double valley = Math.Min(db1[bin],db1[other]);
                for (int k = Math.Min(bin,other)+1; k < Math.Max(bin,other); k++)
                    valley = Math.Min(valley,db1[k]);
                return Math.Min(db1[bin],db1[other])-valley < 6;
            });
            if (samePeak) continue;

            // 对最终保留的峰才做格间插值，标签报告估计的 Hz 和 dBFS。
            var (refinedFreq, refinedPeak) = FftEngine.RefinePeakParabolic(lin1, bin, sampleRate, effectiveFft);
            double refinedDb = SignalGeneratorDsp.LinearToDbfs(refinedPeak);
            string label = $"{refinedFreq:G8} Hz ({refinedDb:F3} dBFS)";

            peaks.Add(new SpectrumPeakMarker(refinedFreq, refinedDb, label));
            usedBins.Add(bin);
        }

        return peaks;
    }

    // 由第一声道的频带功率、逐格中位数与已选主峰估计带内电平、噪声密度和杂散动态范围。
    private static FftDerivedMetrics ComputeFftDerivedMetrics(
        Complex[] halfSpec,
        double[] magDbfs,
        IReadOnlyList<SpectrumPeakMarker> peaks,
        int sampleRate,
        int fftSize,
        FftEngine.WindowData window)
    {
        double binHz = (double)sampleRate / fftSize;
        double powerScale = 4.0 / ((double)fftSize * fftSize * window.NoisePowerGain);
        int lo = Math.Max(2, (int)Math.Ceiling(20.0 / binHz));
        int hi = Math.Min(halfSpec.Length - 2, (int)Math.Floor(Math.Min(20000.0, sampleRate * 0.48) / binHz));

        double totalPower = 0.0;
        var sortedDbs = new List<double>(Math.Max(1, hi - lo + 1));
        // 按窗功率校正积分 20 Hz 至可用高端内的能量，同时收集各格电平，给噪声估计使用。
        for (int k = lo; k <= hi; k++)
        {
            double m = halfSpec[k].Magnitude;
            totalPower += m * m * powerScale;
            sortedDbs.Add(magDbfs[k]);
        }

        double bandPowerDbfs = totalPower > 1e-18 ? 10.0 * Math.Log10(totalPower) : -180.0;
        // 中位数比均值更不易被少量强谱线抬高。再扣除窗的等效噪声带宽（Hz）的 10log10，换算每 Hz 密度估计。
        sortedDbs.Sort();
        double medianBinDb = sortedDbs.Count > 0 ? sortedDbs[sortedDbs.Count / 2] : -180.0;
        // 噪声谱密度 = 中位 bin 电平 − 10·log10(ENBW_Hz)
        double enbwHz = Math.Max(binHz * window.EnbwBins, 1e-6);
        double noiseDensity = medianBinDb - 10.0 * Math.Log10(enbwHz);

        // 两个峰的电平差给出此处的 SFDR 估计，缺少次峰时改用中位格作参考，含义随之改变。
        double mainPeakDb = peaks.Count > 0 ? peaks[0].MagnitudeDbfs : bandPowerDbfs;
        double parDb = Math.Max(0.0, mainPeakDb - bandPowerDbfs);
        double sfdrDbc = peaks.Count >= 2 ? (peaks[0].MagnitudeDbfs - peaks[1].MagnitudeDbfs) : (mainPeakDb - medianBinDb);

        return new FftDerivedMetrics(bandPowerDbfs, noiseDensity, parDb, Math.Max(0.0, sfdrDbc));
    }

    // 已知目标 Hz 时可直接投影到该频率的正弦与余弦，得到峰值及相对于记录起点的相角。
    /// <summary>锁相正交解调器：提取单个目标频率的精确幅度与相位（延迟 τ 对应 −360° × f × τ）。</summary>
    public static (double PeakLinear, double LevelDbfs, double PhaseDeg) ExtractSingleToneLockIn(
        ReadOnlySpan<double> samples,
        int sampleRate,
        double targetFrequencyHz)
    {
        int n = samples.Length;
        if (n < 64) return (0.0, -180.0, 0.0);

        double mean = ComputeWindowedDcMean(samples);
        double re = 0.0, im = 0.0, winSum = 0.0;
        double omega = 2.0 * Math.PI * targetFrequencyHz / sampleRate;
        double winStep = 2.0 * Math.PI / n;

        // Hann 窗抑制邻频泄漏。虚部累积负正弦，与 FFT 的负指数约定一致；目标越接近其他强音，投影越易混入其分量。
        for (int i = 0; i < n; i++)
        {
            double w = 0.5 - 0.5 * Math.Cos(winStep * i);
            double x = (samples[i] - mean) * w;
            double angle = omega * i;
            re += x * Math.Cos(angle);
            im -= x * Math.Sin(angle);
            winSum += w;
        }

        // 对远离直流和奈奎斯特、负频镜像泄漏可忽略的实正弦，2/窗和将单边投影恢复为峰值尺度。
        double scale = 2.0 / Math.Max(winSum, 1e-12);
        double reNorm = re * scale;
        double imNorm = im * scale;
        double peakLinear = Math.Sqrt(reNorm * reNorm + imNorm * imNorm);
        return (peakLinear, SignalGeneratorDsp.LinearToDbfs(peakLinear), Math.Atan2(imNorm, reNorm) * (180.0 / Math.PI));
    }

    // 保留直流的全波形 RMS。逐样本平方平均、开方，再按满幅正弦有效值换成 dBFS。
    private static double FullRmsDbfs(ReadOnlySpan<double> samples)
    {
        if (samples.IsEmpty) return -180;
        double power = 0;
        foreach (double sample in samples) power += sample * sample;
        return SignalGeneratorDsp.LinearToDbfs(Math.Sqrt(power / samples.Length) * Sqrt2);
    }

    // 先估去直流电平，再对其余样本求交流 RMS。恒定输入在理想算术下产生零交流功率。
    private static double ComputeUnfilteredAcRmsDbfs(ReadOnlySpan<double> samples)
    {
        if (samples.Length == 0) return -180.0;
        double mean = ComputeWindowedDcMean(samples);
        double sumSq = 0.0;
        for (int i = 0; i < samples.Length; i++)
        {
            double ac = samples[i] - mean;
            sumSq += ac * ac;
        }
        return SignalGeneratorDsp.LinearToDbfs(Math.Sqrt(sumSq / samples.Length) * Sqrt2);
    }

    // 在本次快照里串联二阶节实现偶数阶 Butterworth 高/低通。每次调用都从零历史开始。
    private static void ApplyButterworthInPlace(double[] buffer, int sampleRate, double cutoffHz, int order, bool isHighPass)
    {
        double k = Math.Tan(Math.PI * Math.Clamp(cutoffHz, 5.0, sampleRate * 0.495) / sampleRate);
        double k2 = k * k;
        for (int section = 0; section < order / 2; section++)
        {
            // 不同极点对使用不同 Q。高通分子在直流相消，低通分子在奈奎斯特相消，分母决定衰减和暂态。
            double inverseQ = 2.0 * Math.Cos((2 * section + 1) * Math.PI / (2 * order));
            double norm = 1.0 / (1.0 + inverseQ * k + k2);
            double b0 = isHighPass ? norm : k2 * norm;
            double b1 = isHighPass ? -2.0 * norm : 2.0 * b0;
            double b2 = b0;
            double a1 = 2.0 * (k2 - 1.0) * norm;
            double a2 = (1.0 - inverseQ * k + k2) * norm;

            double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
            // 按差分方程用当前及前两次输入输出求新值，再滚动延迟项。前一节写入缓冲的输出即下一节输入。
            for (int i = 0; i < buffer.Length; i++)
            {
                double x0 = buffer[i];
                double y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
                x2 = x1; x1 = x0;
                y2 = y1; y1 = y0;
                buffer[i] = y0;
            }
        }
    }

    // 短记录直接求算术均值，较长记录用 Hann 权重压低端点对偏置估计的影响。
    private static double ComputeWindowedDcMean(ReadOnlySpan<double> samples)
    {
        if (samples.Length == 0) return 0.0;
        if (samples.Length < 16)
        {
            double s = 0.0;
            for (int i = 0; i < samples.Length; i++) s += samples[i];
            return s / samples.Length;
        }

        double weightedSum = 0.0, winSum = 0.0;
        double twoPiOverN = 2.0 * Math.PI / samples.Length;
        // 每点按窗加权并除以窗和，所以输入若为常量，估得的直流值仍是该常量。
        for (int i = 0; i < samples.Length; i++)
        {
            double w = 0.5 - 0.5 * Math.Cos(twoPiOverN * i);
            weightedSum += samples[i] * w;
            winSum += w;
        }
        return weightedSum / Math.Max(winSum, 1e-12);
    }

    // 选不超过可用点数及请求上限的最大 2 的幂，供要求基二长度的快照路径使用；初始下限为 1。
    private static int ChooseLargestPowerOfTwo(int availableLength, int maxDesired)
    {
        int limit = Math.Min(availableLength, maxDesired);
        int size = 1;
        while ((size << 1) <= limit) size <<= 1;
        return size;
    }
}

