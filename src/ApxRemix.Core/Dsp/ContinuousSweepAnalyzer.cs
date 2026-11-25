// 连续扫频只发一段变频激励。这里结合激励与整段响应做反卷积，再从冲激响应提取所需频响；若要按到达时间排除晚反射，可以在冲激响应上选窗。
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// Continuous Sweep（Farina 对数 Chirp 反卷积）与 Acoustic Response（麦克风 SPL 标定 + IR 时间窗去反射）引擎。
/// <para>
/// 支持三类明确隔离的测量路径：
/// 1. <b>实测采集反卷积</b>（<see cref="AnalyzeDeconvolution"/>）：对真实播放+采集的长 Chirp 响应校验信号强度与削顶后反卷积；
/// 2. <b>开环文件导入分析</b>：先导出标准激励 Chirp WAV，再导入外部录制响应 WAV 校验采样率并反卷积；
/// 3. <b>显式虚拟演示</b>（<see cref="RunVirtualContinuousSweep(ContinuousSweepConfig, VirtualDutConfig, int, int)"/>）：仅在用户明确选择 Demo 时运行。
/// </para>
/// </summary>
public static class ContinuousSweepAnalyzer
{
    // Preserve the local chirp analyzer's upper passband. ApChirp's tapered
    // section fits above this point and is independently bounded at .49 Fs.
    // 以 0.47Fs 作为本地分析上限，给上端渐弱段以及奈奎斯特界限留出余量。
    public static double MaxAnalysisFrequency(int sampleRate) => sampleRate * .47;

    public static double CheckStopFrequency(double hz, int sampleRate)
    {
        // 此处校验截止 Hz。超过 0.47Fs 或未给出正的有限频率就拒绝配置，调用者不会得到一个被自动修剪的截止值。
        double max = MaxAnalysisFrequency(sampleRate);
        if (!double.IsFinite(hz) || hz <= 0 || hz > max)
            throw new ArgumentOutOfRangeException(nameof(hz),
                $"Sweep stop {hz:G8} Hz is outside the local analysis range (maximum {max:G8} Hz at {sampleRate} samples/s).");
        return hz;
    }

    /// <summary>
    /// 生成与原版 APx500 <c>AudioPrecision.Processing.Signals.ApChirp</c> 逐样本对齐的扫频激励波形。
    /// <para>
    /// 使用原版精确公式：<c>Phi0 = F1·T3 / ln(F3/F1) · (1 − (F3/F1)^(T0/T3))</c>，
    /// 瞬时频率 <c>f(t) = F1·(F3/F1)^(t/T3)</c>，并在 <c>[T0, 0]</c> 与 <c>[Freq2Time(F2), T3]</c>
    /// 施加原版 <c>CosineSquaredWindow</c>（即 <c>[0.5·(1 ± cos(π·u))]² = sin⁴/cos⁴</c>）过渡窗。
    /// </para>
    /// </summary>
    public static double[] GenerateApChirp(
        double t0,
        double t3,
        double f1,
        double f2,
        double f3,
        int sampleRate,
        double levelDbfs,
        out double[] timeAxis)
    {
        // 时间以扫频主段起点为零，负的 t0 留给前沿。频率比将进入对数，两个正的、互异的端点才能定义指数扫速。
        if (t0 > 0.0) throw new ArgumentException("ApChirp T0 必须 <= 0。", nameof(t0));
        if (t3 <= 0.0) throw new ArgumentException("ApChirp T3 必须 > 0。", nameof(t3));
        if (sampleRate <= 0) throw new ArgumentException("采样率必须 > 0。", nameof(sampleRate));
        if (f1 <= 0.0 || f3 <= 0.0 || f3 == f1) throw new ArgumentException("ApChirp F1/F3 必须为正且不相等。");
        return GenerateApChirp(t0, t3, f1, f2, f3, sampleRate, levelDbfs, t3, null, out timeAxis);
    }

    // Extended excitation keeps the carrier's sweep rate while choosing an independent end time and taper.
    internal static double[] GenerateApChirpSignal(double t0, double t3, double f1, double f3, int sampleRate,
        double signalEndTime, double rightTaperStart, out double[] timeAxis)
    {
        // t3 决定载波扫速，signalEndTime 单独决定记录何时结束。把可播放时长与末端渐弱起点分开，延长激励时仍可保持相同扫速。
        if (!(t3 > 0.0 && t0 <= 0.0 && sampleRate > 0 && f1 > 0.0 && f3 > 0.0 && f3 != f1))
            throw new ArgumentException("Invalid APx chirp carrier.");
        if (!(signalEndTime > t0 && rightTaperStart >= 0.0 && rightTaperStart <= signalEndTime))
            throw new ArgumentException("Invalid APx chirp signal range.");
        return GenerateApChirp(t0, t3, f1, f3, f3, sampleRate, 0.0, signalEndTime, rightTaperStart, out timeAxis);
    }

    private static double[] GenerateApChirp(double t0, double t3, double f1, double f2, double f3, int sampleRate,
        double levelDbfs, double signalEndTime, double? rightTaperStart, out double[] timeAxis)
    {
        // 时间步长为 1/Fs 秒，把持续时间换算为整数个采样点。这里按采样点生成相位，返回的时间轴和波形必须一一对应。
        double samplePeriod = 1.0 / sampleRate;
        // APx rounds duration divided by the sample period. Multiplication can cross a half-sample boundary.
        int n = (int)Math.Round(Math.Abs((signalEndTime - t0) / samplePeriod), MidpointRounding.AwayFromZero);
        timeAxis = new double[n];
        double dt = n != 1 ? samplePeriod : 0.0;
        for (int i = 0; i < n; i++) timeAxis[i] = t0 + i * dt;

        // 尝试使用可用的专用数值路径，无法得到结果时由下方托管公式生成。得到单位幅值波形以后统一施加 dBFS 设置。
        var samples = n > 1 && f2 >= Math.Min(f1, f3) && f2 <= Math.Max(f1, f3)
            ? SequenceBinEstimator.GenerateChirp(n, t0, t3, f1, f2, f3, dt, signalEndTime, rightTaperStart) : null;
        samples ??= GenerateManagedApChirp(t0, t3, f1, f2, f3, dt, signalEndTime, rightTaperStart, timeAxis);
        double amplitude = SignalGeneratorDsp.DbfsToLinearPeak(levelDbfs);
        if (amplitude != 1.0)
            for (int i = 0; i < n; i++) samples[i] *= amplitude;
        return samples;
    }

    private static double[] GenerateManagedApChirp(double t0, double t3, double f1, double f2, double f3, double dt,
        double signalEndTime, double? rightTaperStart, double[] timeAxis)
    {
        int n = timeAxis.Length;
        var samples = new double[n];
        // 指数扫频满足 f(t)=f₁·(f₃/f₁)^(t/t₃)。正弦的相位要对 2πf(t) 积分，因此式中会出现 t₃/ln(f₃/f₁)，直接用瞬时频率乘时间会得到另一条扫频曲线。
        double ratio = f3 / f1;
        double lnRatio = Math.Log(ratio);
        double phi0 = f1 * t3 / lnRatio * (1.0 - Math.Pow(ratio, t0 / t3));
        double factor = 2.0 * Math.PI * f1 * t3 / lnRatio;
        double baseVal = Math.Pow(ratio, 1.0 / t3);
        double initPhase = 2.0 * Math.PI * phi0;

        for (int i = 0; i < n; i++)
        {
            double phase = Math.Pow(baseVal, timeAxis[i]) - 1.0;
            phase *= factor;
            phase += initPhase;
            samples[i] = Math.Sin(phase);
        }

        // 指数相位决定扫到哪个频率，包络决定端点有多响；两者分开算才可独立控制起振和收尾。
        int leftCount = (int)Math.Floor(-t0 / dt);
        if (leftCount > 0) ApplyApChirpTaper(timeAxis, samples, 0, Math.Min(leftCount, n), 0.0, t0);
        // 令瞬时频率等于 f2 可解出右端渐弱的起点 t2。调用者另给起点时，使用该秒数。
        double t2 = rightTaperStart ?? t3 / lnRatio * Math.Log(f2 / f1);
        double rightStart = signalEndTime - (signalEndTime - t2);
        int rightCount = (int)Math.Floor((signalEndTime - rightStart) / dt);
        if (rightCount > 0) ApplyApChirpTaper(timeAxis, samples, Math.Max(0, n - rightCount), n, rightStart, signalEndTime);
        return samples;
    }

    private static void ApplyApChirpTaper(double[] times, double[] signal, int start, int end, double from, double to)
    {
        // 余弦从 1 平滑过渡到 0，交换时间边界即可反向渐入。把包络乘到原信号上，可减轻突然开关产生的宽带瞬态。
        double inverseDuration = 1.0 / (to - from);
        for (int i = start; i < end; i++)
        {
            double radians = (times[i] - from) * Math.PI;
            radians *= inverseDuration;
            double raisedCosine = 0.5 * Math.Cos(Math.Clamp(radians, 0.0, Math.PI)) + 0.5;
            signal[i] *= raisedCosine * raisedCosine;
        }
    }

    /// <summary>按配置生成激励波形（支持 APx500 ApChirp 与通用对数 LogChirp）。</summary>
    public static double[] GenerateExcitationChirp(ContinuousSweepConfig config, int sampleRate)
    {
        // 这里选定激励族，决定后续分析可假定哪些频率被驱动。Fast Sweep 用预排的离散音，另外两种按时间改变瞬时频率。
        if (config.ChirpType == ContinuousChirpType.FastSweep)
            return FastSweepGenerator.Generate(BenchFastSweep.Plan(config,sampleRate)).Samples;
        if (config.ChirpType == ContinuousChirpType.LogChirp)
            return GenerateLogChirp(config, sampleRate);

        // AP 型将分析截止 f2 与载波最后频率 f3 分开，留下上端渐弱段；下端同样额外留负时间前导。
        double f1 = Math.Clamp(config.StartFrequencyHz, 5.0, sampleRate * 0.40);
        double f2 = Math.Max(f1 + 10.0, CheckStopFrequency(config.StopFrequencyHz, sampleRate));
        double f3 = Math.Min(sampleRate * 0.49, f2 * 1.05);
        double t3 = Math.Clamp(config.DurationSeconds, 0.1, 15.0);
        double t0 = -Math.Min(0.05, t3 * 0.10);
        return GenerateApChirp(t0, t3, f1, f2, f3, sampleRate, config.LevelDbfs, out _);
    }

    internal static double LogChirpEndFrequency(double startHz, double stopHz, int sampleRate) =>
        Math.Min(sampleRate * 0.5, Math.Max(startHz + 10.0, stopHz) * 1.1);

    public static double[] GenerateLogChirp(ContinuousSweepConfig config, int sampleRate)
    {
        double f1 = Math.Clamp(config.StartFrequencyHz, 5.0, sampleRate * 0.40);
        double f2 = Math.Max(f1 + 10.0, CheckStopFrequency(config.StopFrequencyHz, sampleRate));
        double duration = Math.Clamp(config.DurationSeconds, 0.1, 15.0);
        int n = Math.Max(2048, (int)Math.Round(duration * sampleRate));

        double amp = SignalGeneratorDsp.DbfsToLinearPeak(config.LevelDbfs);
        // 对齐 APx500 ChirpParams：载波在 [0, T3] 内从 F1 扫至 F3 = min(1.1·F2, Fs/2)，
        // 右端余弦平方窗施加在 [Freq2Time(F2), T3]（即有效频带 F2 之上），保持总长度不变且 [F1, F2] 全程满幅。
        double f3 = LogChirpEndFrequency(f1, f2, sampleRate);
        double lnRatio = Math.Log(f3 / f1);
        double l = duration / lnRatio;
        double k = 2.0 * Math.PI * f1 * l;
        double t2 = l * Math.Log(f2 / f1);
        double rightSpan = duration - t2;

        var chirp = new double[n];
        // 对齐 APx500 ChirpParams / LogChirp：t = 0 处 sin(0) = 0 无左端淡入窗，仅在 [t2, T] (F2..F3) 施加右端余弦平方淡出，
        // 避免在 [0, 2 ms] 截削起始低频能量导致 20 Hz 幅度误差与虚假谐波失真。
        for (int i = 0; i < n; i++)
        {
            double t = (double)i / sampleRate;
            double env = 1.0;
            if (rightSpan > 0.0 && t > t2)
            {
                double u = Math.Clamp((t - t2) / rightSpan, 0.0, 1.0);
                double rc = 0.5 * (1.0 + Math.Cos(Math.PI * u));
                env = rc * rc;
            }

            chirp[i] = amp * env * Math.Sin(k * (Math.Exp(t / l) - 1.0));
        }

        return chirp;
    }

    /// <summary>
    /// 直接从参考激励波形本身测量有效段 AES17 电平（dBFS，自动剔除首尾静音保护带与淡入淡出过渡）。
    /// 用于声学 SPL 绝对标定，彻底隔离界面发生器电平滑块对开环 WAV 分析结果的污染。
    /// </summary>
    public static double MeasureActiveReferenceLevelDbfs(ReadOnlySpan<double> referenceSignal)
    {
        if (referenceSignal.Length == 0) return -180.0;

        double peak = 0.0;
        for (int i = 0; i < referenceSignal.Length; i++)
        {
            double a = Math.Abs(referenceSignal[i]);
            if (a > peak) peak = a;
        }
        if (peak <= 1e-9) return -180.0;

        // 前后静音也计入 RMS 会低估实际激励。用峰值的 5% 定位首尾，再在够长的有效区间内裁掉两端过渡。
        double gate = peak * 0.05;
        int first = 0;
        while (first < referenceSignal.Length && Math.Abs(referenceSignal[first]) < gate) first++;
        int last = referenceSignal.Length - 1;
        while (last > first && Math.Abs(referenceSignal[last]) < gate) last--;

        int span = last - first + 1;
        if (span < 16) return -180.0;

        // 跳过首尾 5% 淡入淡出包络，在平顶稳态段计算均方根
        int trim = span >= 200 ? (int)(span * 0.05) : 0;
        int s0 = first + trim;
        int s1 = last - trim;
        if (s1 <= s0) { s0 = first; s1 = last; }

        // 交流 RMS² 等于均方减均值平方，因而恒定偏置不会增加此处的激励电平。乘 √2 后按满幅正弦的 RMS 基准换成 dBFS。
        double sumSq = 0, sum = 0;
        for (int i = s0; i <= s1; i++)
        {
            sum += referenceSignal[i];
            sumSq += referenceSignal[i] * referenceSignal[i];
        }
        int count = s1 - s0 + 1;
        double mean = sum / count;
        double rms = Math.Sqrt(Math.Max(0,sumSq / count - mean * mean));
        return SignalGeneratorDsp.LinearToDbfs(rms * AudioMetricsAnalyzer.Sqrt2);
    }

    /// <summary>
    /// 对真实采集或开环导入的 Chirp 响应执行完整参考/响应配对校验、削顶检测与正则化频域反卷积。
    /// 严格拦截：空白/静音参考、非宽带参考（选错文件）、截短录音、无相关响应与断线静音，绝不生成虚假 0 ms 曲线。
    /// </summary>
    public static ContinuousSweepResult AnalyzeDeconvolution(
        ReadOnlySpan<double> recordedResponse,
        ReadOnlySpan<double> referenceChirp,
        int sampleRate,
        ContinuousSweepConfig config,
        string sourceDescription = "Live Capture", int leakageOffsetSamples = 0, bool computeGroupDelay = true)
    {
        config = BenchFastSweep.WithTableBand(config, sampleRate);
        if (referenceChirp.Length < 1024 || recordedResponse.Length < 1024)
            throw new InvalidOperationException("Chirp 激励或实测响应样本长度不足（需 >= 1024 点），无法执行反卷积。");

        // 1. 校验参考激励有效性（拦截空白/静音参考 WAV）与有效激励跨度
        // 参考波形是后面做除法的分母。先验证它确实含有足够长的有效激励，再看录到的响应够不够长。
        double refPeak = 0.0;
        for (int i = 0; i < referenceChirp.Length; i++)
        {
            double a = Math.Abs(referenceChirp[i]);
            if (a > refPeak) refPeak = a;
        }
        double refActiveLevelDbfs = MeasureActiveReferenceLevelDbfs(referenceChirp);
        if (refPeak <= 1e-6 || refActiveLevelDbfs < -90.0)
            throw new InvalidOperationException($"参考激励无效或为静音（实测 {refActiveLevelDbfs:F1} dBFS），拒绝生成反卷积曲线。");

        double refGate = refPeak * 0.05;
        int refFirst = 0;
        while (refFirst < referenceChirp.Length && Math.Abs(referenceChirp[refFirst]) < refGate) refFirst++;
        int refLast = referenceChirp.Length - 1;
        while (refLast > refFirst && Math.Abs(referenceChirp[refLast]) < refGate) refLast--;
        int activeRefSpan = refLast - refFirst + 1;
        if (activeRefSpan < 1024)
            throw new InvalidOperationException($"参考激励有效时长过短（仅 {activeRefSpan} 样本），无法构成有效 Chirp 扫频。");

        // 2. 拦截截短录音（实录长度明显短于参考有效激励长度）
        if (recordedResponse.Length < (int)(activeRefSpan * 0.90))
            throw new InvalidOperationException($"响应录音被截短（仅 {recordedResponse.Length} 样本，参考有效段需至少 {activeRefSpan} 样本）。");

        // 3. 校验实际响应信号强度与削顶状态（拦截断线/无输入）
        // 峰值接近数字满量程提示可能削顶；整段 RMS 太小则无法可靠反推出系统响应。峰值和 RMS 分别检查这两个条件。
        double rawPeak = 0.0;
        double sumSq = 0.0;
        for (int i = 0; i < recordedResponse.Length; i++)
        {
            double v = recordedResponse[i];
            double abs = Math.Abs(v);
            if (abs > rawPeak) rawPeak = abs;
            sumSq += v * v;
        }
        double respRmsDbfs = SignalGeneratorDsp.LinearToDbfs(Math.Sqrt(sumSq / recordedResponse.Length) * AudioMetricsAnalyzer.Sqrt2);
        if (respRmsDbfs < -90.0)
            throw new InvalidOperationException($"未检测到有效 Chirp 响应信号（实测 {respRmsDbfs:F1} dBFS，可能已拔掉环回线/麦克风或输入静音）。");
        bool wasClipping = rawPeak >= 0.999;

        int lenX = referenceChirp.Length;
        int lenY = recordedResponse.Length;
        bool isFastSweep = config.ChirpType == ContinuousChirpType.FastSweep;
        int before = (int)(0.05 * sampleRate);

        // 对齐 APx500 ChirpParams：连续扫频采用 GoodLengths 帧长（而非 2 的幂过度补零），
        // 保证线性 IR 门长 linFftLen = FftLength / 2 及频率分辨率与原版一致；Fast Sweep 维持独立离散分析路径。
        int fftSize = isFastSweep
            ? NextPowerOfTwo(Math.Max(4096, Math.Max(lenX, lenY) + Math.Min(lenX, lenY) / 2))
            : SequenceTransform.NextLength(Math.Max(Math.Max(lenX, lenY), 2 * before + 2), allowShort: true);

        var xBuf = new Complex[fftSize];
        var yBuf = new Complex[fftSize];
        for (int i = 0; i < lenX; i++) xBuf[i] = new Complex(referenceChirp[i], 0.0);
        for (int i = 0; i < lenY; i++) yBuf[i] = new Complex(recordedResponse[i], 0.0);

        SequenceTransform.Forward(xBuf);
        SequenceTransform.Forward(yBuf);

        double maxPower = 0.0;
        double deconvBinHz = (double)sampleRate / fftSize;
        int fStartBin = Math.Clamp((int)Math.Ceiling(Math.Max(20.0, config.StartFrequencyHz) / deconvBinHz), 1, fftSize / 2 - 2);
        double stopHz = CheckStopFrequency(config.StopFrequencyHz, sampleRate);
        int fStopBin = Math.Clamp((int)Math.Floor(stopHz / deconvBinHz), fStartBin + 1, fftSize / 2 - 1);
        for (int i = fStartBin; i <= fStopBin; i++)
        {
            double p = xBuf[i].Real * xBuf[i].Real + xBuf[i].Imaginary * xBuf[i].Imaginary;
            if (p > maxPower) maxPower = p;
        }
        if (maxPower <= 1e-18)
            throw new InvalidOperationException("参考激励频谱无有效交流能量，无法执行反卷积。");

        // 4. 校验参考信号在目标扫频带内具备宽带覆盖（拦截误选单音或非 Chirp 文件作为参考）
        // 检查频带中有多少点获得了足够激励。若参考只含一个强音，单凭该音无法判断整条系统频响。Fast Sweep 的离散激励另有分析分支。
        double bandFloor = maxPower * Math.Pow(10.0, -45.0 / 10.0);
        int checkedBins = 0, excitedBins = 0;
        int stride = Math.Max(1, (fStopBin - fStartBin) / 256);
        for (int b = fStartBin; b <= fStopBin; b += stride)
        {
            checkedBins++;
            double p = xBuf[b].Real * xBuf[b].Real + xBuf[b].Imaginary * xBuf[b].Imaginary;
            if (p >= bandFloor) excitedBins++;
        }
        if (!isFastSweep && checkedBins > 8 && (double)excitedBins / checkedBins < 0.25)
            throw new InvalidOperationException($"参考文件不是有效的宽带扫频激励（目标频带仅 {excitedBins * 100.0 / checkedBins:F0}% 受激，可能选错了参考文件）。");

        double minimumReferenceMagnitude = Math.Sqrt(maxPower) * 1e-6;
        int half = fftSize / 2;

        if (isFastSweep)
        {
            var hSpec = new Complex[fftSize];
            double eps = Math.Max(maxPower * 1e-7, 1e-18);
            for (int i = 0; i < fftSize; i++)
            {
                double denom = xBuf[i].Real * xBuf[i].Real + xBuf[i].Imaginary * xBuf[i].Imaginary + eps;
                hSpec[i] = yBuf[i] * (Complex.Conjugate(xBuf[i]) / denom);
            }
            var irComplex = (Complex[])hSpec.Clone();
            SequenceTransform.Inverse(irComplex);
            var fullIrFast = new double[fftSize];
            int rawPeakFast = 0;
            double peakAbsFast = 0.0, irSumSqFast = 0.0;
            for (int i = 0; i < fftSize; i++)
            {
                double val = irComplex[i].Real;
                fullIrFast[i] = val;
                irSumSqFast += val * val;
                if (Math.Abs(val) > peakAbsFast) { peakAbsFast = Math.Abs(val); rawPeakFast = i; }
            }
            double irRmsFast = Math.Sqrt(irSumSqFast / fftSize);
            double irCrestFast = irRmsFast > 1e-18 ? peakAbsFast / irRmsFast : 0.0;
            if (peakAbsFast <= 1e-6 || irCrestFast < 6.0)
                throw new InvalidOperationException($"参考激励与实测响应未形成有效相关脉冲峰（IR 峰均比 {irCrestFast:F2} < 6.0，可能文件不配对、严重失步或仅为无关噪声）。");

            int signedPeakFast = rawPeakFast <= half ? rawPeakFast : rawPeakFast - fftSize;
            double delayMsFast = signedPeakFast * 1000.0 / sampleRate;
            int tailFast = (int)(sampleRate * Math.Clamp(config.ResponseTailMs / 1000.0, 0.1, 3.0));
            int firstFast = Math.Max(-half, Math.Min(0, signedPeakFast) - (int)Math.Ceiling(sampleRate * 0.010));
            int lastFast = Math.Min(half - 1, Math.Max(0, signedPeakFast) + tailFast);
            int lenFast = lastFast - firstFast + 1;
            var irDispFast = new double[lenFast];
            var irTimeFast = new double[lenFast];
            for (int i = 0; i < lenFast; i++)
            {
                int sample = firstFast + i;
                irDispFast[i] = fullIrFast[(sample + fftSize) % fftSize];
                irTimeFast[i] = sample * 1000.0 / sampleRate;
            }
            double calFast = AudioMetricsAnalyzer.ComputeCalibrationGainOffsetDb(
                config.InputFullScaleVoltageVrms, config.OutputFullScaleVoltageVrms);
            var fastPoints = ExtractResponsePointsFromSpectrum(
                hSpec, sampleRate, config.StartFrequencyHz, stopHz,
                config.OutputPoints == 0 ? 1200 : config.OutputPoints, refActiveLevelDbfs, gainOffsetDb: calFast,
                phaseMode: config.PhaseMode, delaySamples: signedPeakFast,
                measuredMagnitude: bin => xBuf[bin].Magnitude >= minimumReferenceMagnitude ? yBuf[bin].Magnitude / xBuf[bin].Magnitude : double.NaN);
            return BenchFastSweep.Analyze(recordedResponse, referenceChirp, sampleRate, config, signedPeakFast,
                new(fastPoints, irDispFast, irTimeFast, delayMsFast, signedPeakFast, peakAbsFast, sampleRate,
                    wasClipping, sourceDescription, EnergyTimeCurveDb: ComputeEnergyTimeCurveDb(irDispFast),
                    Acquired: recordedResponse.ToArray(), Reference: referenceChirp.ToArray()));
        }

        // 对齐 APx500 ChirpParams ctor (L28295-28347)：
        // 通带 [F1, F2] 内取精确逆 1/X(k)，通带外按 100×|1/X(F1)| 与 1.1×|1/X(F2)| 限幅，直流格置零，
        // 并乘以 (-1)^k 将时间零点居中于 fullIr[FftLength / 2]。
        int lowRefBin = Math.Clamp((int)Math.Round(config.StartFrequencyHz / sampleRate * fftSize, MidpointRounding.AwayFromZero), 1, half - 2);
        int highRefBin = Math.Clamp((int)Math.Round(stopHz / sampleRate * fftSize, MidpointRounding.AwayFromZero), lowRefBin + 1, half - 1);
        double lowMag = Math.Max(xBuf[lowRefBin].Magnitude, minimumReferenceMagnitude);
        double highMag = Math.Max(xBuf[highRefBin].Magnitude, minimumReferenceMagnitude);
        double lowInvLimit = 100.0 / lowMag;
        double highInvLimit = 1.1 / highMag;

        var hCenteredSpec = new Complex[fftSize];
        hCenteredSpec[0] = Complex.Zero;
        for (int k = 1; k <= half; k++)
        {
            double mag = xBuf[k].Magnitude;
            Complex inv;
            if (mag >= minimumReferenceMagnitude)
            {
                inv = Complex.One / xBuf[k];
                double invMag = 1.0 / mag;
                if (k <= lowRefBin && invMag > lowInvLimit) inv *= lowInvLimit / invMag;
                else if (k > highRefBin && invMag > highInvLimit) inv *= highInvLimit / invMag;
            }
            else
            {
                inv = Complex.Zero;
            }
            if ((k & 1) != 0) inv = -inv;
            hCenteredSpec[k] = yBuf[k] * inv;
            if (k < half)
                hCenteredSpec[fftSize - k] = Complex.Conjugate(hCenteredSpec[k]);
        }

        // 反卷积回到时域（t = 0 位于 half = fftSize / 2）
        SequenceTransform.Inverse(hCenteredSpec);
        var centeredIr = new double[fftSize];
        double irSumSq = 0.0;
        int globalPeakIdx = half;
        double globalPeakAbs = 0.0;
        for (int i = 0; i < fftSize; i++)
        {
            double val = hCenteredSpec[i].Real;
            centeredIr[i] = val;
            irSumSq += val * val;
            double abs = Math.Abs(val);
            if (abs > globalPeakAbs) { globalPeakAbs = abs; globalPeakIdx = i; }
        }

        // 对齐 APx500 FullAnalysis (L28816-28837)：优先在 [half - before, fftSize - 1] 内搜索主峰；
        // 若开环录音存在超过 50 ms 的负时延导致主峰落在该区间之前，再回退到全局主峰。
        int causalSearchStart = Math.Max(0, half - before);
        int peakInFullIr = causalSearchStart;
        double peakAbs = 0.0;
        for (int i = causalSearchStart; i < fftSize; i++)
        {
            double abs = Math.Abs(centeredIr[i]);
            if (abs > peakAbs) { peakAbs = abs; peakInFullIr = i; }
        }
        if (globalPeakAbs > peakAbs * 2.0)
        {
            peakAbs = globalPeakAbs;
            peakInFullIr = globalPeakIdx;
        }

        // 5. 校验反卷积脉冲响应相关性（拦截互不相关的噪声或错配的参考/响应文件）
        double irRms = Math.Sqrt(irSumSq / fftSize);
        double irCrest = irRms > 1e-18 ? peakAbs / irRms : 0.0;
        if (peakAbs <= 1e-6 || irCrest < 6.0)
            throw new InvalidOperationException($"参考激励与实测响应未形成有效相关脉冲峰（IR 峰均比 {irCrest:F2} < 6.0，可能文件不配对、严重失步或仅为无关噪声）。");

        int signedPeakIdx = peakInFullIr - half;
        double fractionalPeakIdx = ChirpResultMath.PeakInterp(centeredIr, peakInFullIr);
        double delayMs = (fractionalPeakIdx - half) * 1000.0 / sampleRate;

        // 高速错开串扰模式：对齐 APx500 OPPE837xSG，从居中 IR 原位切除 2 ms 串扰脉冲，
        // 不再截断直通通道的线性窗长（避免 Ch1 低频响应、相位与群延迟劣化）。
        double[]? excisedXtalkSlice = null;
        if (leakageOffsetSamples != 0)
            excisedXtalkSlice = ChirpCrosstalk.ExciseStaggeredImpulse(centeredIr, peakInFullIr, leakageOffsetSamples, sampleRate, stopHz);

        // 对齐 APx500 linImpResp：保留主峰/零点前 50 ms 至因果半轴末端，时间轴以零时刻为基准（毫秒）
        int firstSample = Math.Max(-half, Math.Min(0, signedPeakIdx) - before);
        int lastSample = half - 1;
        int length = lastSample - firstSample + 1;
        var irDisplay = new double[length];
        var irTimeMs = new double[length];
        for (int i = 0; i < length; i++)
        {
            int sample = firstSample + i;
            irDisplay[i] = centeredIr[half + sample];
            irTimeMs[i] = sample * 1000.0 / sampleRate;
        }

        double calGainOffsetDb = AudioMetricsAnalyzer.ComputeCalibrationGainOffsetDb(
            config.InputFullScaleVoltageVrms, config.OutputFullScaleVoltageVrms);

        // 对齐 APx500 LinearResponseResults + FftWithAsymWin.KvSOGjdaHk：
        // LogChirp 严格采用原版加窗脉冲频谱（不做 refGatedBuf 除法，避免 20–50 Hz +0.097 dB 边缘畸变）；
        // 仅保留给旧版非界面 ApChirp 的带限参考窗归一以维持历史单测口径。
        bool normalizeLegacyApChirp = config.ChirpType == ContinuousChirpType.ApChirp;
        double[]? centeredRefIr = null;
        if (normalizeLegacyApChirp)
        {
            var refCenteredSpec = new Complex[fftSize];
            refCenteredSpec[0] = Complex.Zero;
            for (int k = 1; k <= half; k++)
            {
                double mag = xBuf[k].Magnitude;
                Complex inv = mag >= minimumReferenceMagnitude ? Complex.One / xBuf[k] : Complex.Zero;
                if (mag >= minimumReferenceMagnitude)
                {
                    double invMag = 1.0 / mag;
                    if (k <= lowRefBin && invMag > lowInvLimit) inv *= lowInvLimit / invMag;
                    else if (k > highRefBin && invMag > highInvLimit) inv *= highInvLimit / invMag;
                }
                if ((k & 1) != 0) inv = -inv;
                refCenteredSpec[k] = xBuf[k] * inv;
                if (k < half) refCenteredSpec[fftSize - k] = Complex.Conjugate(refCenteredSpec[k]);
            }
            SequenceTransform.Inverse(refCenteredSpec);
            centeredRefIr = new double[fftSize];
            for (int i = 0; i < fftSize; i++) centeredRefIr[i] = refCenteredSpec[i].Real;
        }

        int linStart = peakInFullIr >= causalSearchStart ? causalSearchStart : Math.Max(0, peakInFullIr - before);
        int available = fftSize - linStart;
        int peakInLin = peakInFullIr - linStart;
        int linFftLen = 2 * (half / 2);
        int sliceStart = Math.Clamp(peakInLin - linFftLen / 2, 0, Math.Max(0, available - linFftLen));
        int used = Math.Min(available, linFftLen);
        int peakInSlice = peakInLin - sliceStart;
        var asymWin = SequenceFrequencyResponse.CreateImpulseWindow(used, peakInSlice, 0.90, 0.95);

        bool needGd = !config.OpenLoop && computeGroupDelay;
        var gatedBuf = new Complex[linFftLen];
        Complex[]? refGatedBuf = normalizeLegacyApChirp ? new Complex[linFftLen] : null;
        Complex[]? gatedMomentBuf = needGd ? new Complex[linFftLen] : null;
        Complex[]? refMomentBuf = needGd && normalizeLegacyApChirp ? new Complex[linFftLen] : null;
        for (int i = 0; i < used; i++)
        {
            double w = asymWin[i];
            int rel = i - peakInSlice;
            double ySample = centeredIr[linStart + sliceStart + i] * w;
            gatedBuf[i] = new Complex(ySample, 0.0);
            if (needGd)
                gatedMomentBuf![i] = new Complex(ySample * rel, 0.0);
            if (normalizeLegacyApChirp)
            {
                int refSrc = half + rel;
                double rSample = (uint)refSrc < (uint)fftSize ? centeredRefIr![refSrc] * w : 0.0;
                refGatedBuf![i] = new Complex(rSample, 0.0);
                if (needGd) refMomentBuf![i] = new Complex(rSample * rel, 0.0);
            }
        }
        SequenceTransform.Forward(gatedBuf);
        if (refGatedBuf is not null) SequenceTransform.Forward(refGatedBuf);
        if (needGd)
        {
            SequenceTransform.Forward(gatedMomentBuf!);
            if (refMomentBuf is not null) SequenceTransform.Forward(refMomentBuf);
        }

        var linearSpec = new Complex[linFftLen];
        double linBinHz = (double)sampleRate / linFftLen;
        int linHalf = linFftLen / 2;
        double peakPhaseStep = Math.Tau * peakInSlice / linFftLen;
        for (int k = 0; k <= linHalf; k++)
        {
            int xBin = Math.Clamp((int)Math.Round(k * linBinHz / deconvBinHz), 0, half);
            Complex val;
            if (xBuf[xBin].Magnitude < minimumReferenceMagnitude)
                val = new Complex(double.NaN, double.NaN);
            else if (refGatedBuf is not null)
                val = refGatedBuf[k].Magnitude >= 1e-6 ? gatedBuf[k] / refGatedBuf[k] : new Complex(double.NaN, double.NaN);
            else
                val = gatedBuf[k] * Complex.FromPolarCoordinates(1.0, k * peakPhaseStep);
            linearSpec[k] = val;
            if (k > 0 && k < linHalf) linearSpec[linFftLen - k] = Complex.Conjugate(val);
        }

        double MeasuredMagnitude(int bin) => linearSpec[bin].Magnitude;
        var points = ExtractResponsePointsFromSpectrum(
            linearSpec, sampleRate, config.StartFrequencyHz, stopHz,
            config.OutputPoints == 0 ? 1200 : config.OutputPoints, refActiveLevelDbfs, gainOffsetDb: calGainOffsetDb,
            phaseMode: config.PhaseMode, delaySamples: signedPeakIdx, measuredMagnitude: MeasuredMagnitude,
            alreadyPeakAligned: true);

        // 激励包络可能随频率改变。把每个频点的激励电平加回数字增益，才能估计该点的响应电平。
        var envelope = SweepReferenceLevels.Measure(referenceChirp, sampleRate, config.StartFrequencyHz, config.StopFrequencyHz);
        double ExcitationLevel(double hz) => envelope == null ? refActiveLevelDbfs : FrequencyGainTable.GainAtHz(envelope, hz);
        double referenceHz = Math.Clamp(1000, config.StartFrequencyHz, stopHz);
        double referenceLevel = SpectrumGain(referenceHz, linBinHz, linFftLen, MeasuredMagnitude, 0) + ExcitationLevel(referenceHz);

        SweepSpectrum? measuredSpectrum = null;
        if (config.OutputPoints == 0)
        {
            Func<int, double> magnitudeAt = MeasuredMagnitude;
            var hz = SpectrumFrequencies(config.StartFrequencyHz, stopHz, linBinHz);
            var gains = new double[hz.Length];
            var levels = new double[hz.Length];
            var relative = new double[hz.Length];
            var phase = ExtractChirpPhases(linearSpec, hz, linBinHz, signedPeakIdx, config.PhaseMode, alreadyPeakAligned: true);
            for (int i = 0; i < hz.Length; i++)
            {
                double g = SpectrumGain(hz[i], linBinHz, linFftLen, magnitudeAt, calGainOffsetDb);
                gains[i] = g;
                levels[i] = g - calGainOffsetDb + ExcitationLevel(hz[i]);
                relative[i] = levels[i] - referenceLevel;
                if (!double.IsFinite(g)) phase[i] = double.NaN;
            }
            measuredSpectrum = new(hz, gains, levels, relative, phase);
        }
        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];
            double excitation = ExcitationLevel(p.FrequencyHz);
            double level = p.Ch1GainDb - calGainOffsetDb + excitation;
            points[i] = p with
            {
                ExcitationToneDbfs = excitation,
                Ch1LevelDbfs = level,
                Ch2LevelDbfs = level,
                Ch1RelativeDb = level - referenceLevel,
                Ch2RelativeDb = level - referenceLevel
            };
        }

        double[] etcDb = ComputeEnergyTimeCurveDb(irDisplay);

        IReadOnlyList<FrequencyResponsePoint>? h2Points = null;
        IReadOnlyList<FrequencyResponsePoint>? h3Points = null;
        IReadOnlyList<FrequencyResponsePoint>? thdPoints = null;
        Dictionary<int, BenchTrace>? productLevels = null;
        if (config.ExtractHarmonics)
        {
            var harm = ChirpResultMath.ExtractContinuousHarmonics(
                centeredIr, peakInFullIr, lenY, linearSpec, linFftLen, sampleRate, config, points, ExcitationLevel, useIecRatio: true);
            h2Points = harm.H2;
            h3Points = harm.H3;
            thdPoints = harm.Thd;
            productLevels = harm.ProductLevels.Count > 0 ? harm.ProductLevels : null;
        }

        BenchTrace? crosstalk = null;
        if (leakageOffsetSamples != 0)
        {
            int harmFftLen = SequenceTransform.NextLength(Math.Max(lenY >> 3, 1024), allowShort: true);
            crosstalk = excisedXtalkSlice != null
                ? ChirpCrosstalk.FromExcisedImpulse(excisedXtalkSlice, linearSpec, linFftLen, harmFftLen, sampleRate, config.StartFrequencyHz, stopHz)
                : ChirpCrosstalk.FromStaggeredImpulse(centeredIr, peakInFullIr, leakageOffsetSamples,
                    points.Where(point => point.FrequencyHz >= 1000).Select(point => point.FrequencyHz).ToArray(), sampleRate);
        }

        BenchTrace? groupDelay = null;
        if (needGd && gatedMomentBuf is not null)
        {
            var gdFreqs = config.OutputPoints == 0 ? SpectrumFrequencies(config.StartFrequencyHz, stopHz, linBinHz)
                : points.Select(p => p.FrequencyHz).ToArray();
            var gdMs = new double[gdFreqs.Length];
            double msPerSample = 1000.0 / sampleRate;
            for (int i = 0; i < gdFreqs.Length; i++)
            {
                int bin = (int)Math.Round(gdFreqs[i] * linFftLen / sampleRate);
                if (bin is > 0 && bin < linHalf && gatedBuf[bin].Magnitude > 1e-8 && (refGatedBuf is null || refGatedBuf[bin].Magnitude > 1e-6))
                {
                    double yMoment = (gatedMomentBuf[bin] / gatedBuf[bin]).Real;
                    double rMoment = refMomentBuf is not null && refGatedBuf is not null ? (refMomentBuf[bin] / refGatedBuf[bin]).Real : 0.0;
                    gdMs[i] = (signedPeakIdx + yMoment - rMoment) * msPerSample;
                }
                else gdMs[i] = double.NaN;
            }
            groupDelay = new(gdFreqs, gdMs);
        }

        return new ContinuousSweepResult(
            points, irDisplay, irTimeMs, delayMs, signedPeakIdx, peakAbs, sampleRate,
            wasClipping, sourceDescription, h2Points, h3Points, thdPoints, etcDb, Acquired: recordedResponse.ToArray(), Reference: referenceChirp.ToArray(),
            CrosstalkMode: crosstalk == null ? ChirpCrosstalkMode.None : ChirpCrosstalkMode.HighSpeed, Crosstalk: crosstalk,
            ProductLevels: productLevels,
            GroupDelay: groupDelay,
            MeasuredSpectrum: measuredSpectrum);
    }

    /// <summary>计算脉冲响应的能量-时间曲线（ETC，基于 Hilbert 解析包络的相对峰值 dB）。</summary>
    public static double[] ComputeEnergyTimeCurveDb(ReadOnlySpan<double> ir)
    {
        if (ir.Length == 0) return Array.Empty<double>();
        int n = NextPowerOfTwo(Math.Max(64, ir.Length));
        var buf = new Complex[n];
        for (int i = 0; i < ir.Length; i++) buf[i] = new Complex(ir[i], 0.0);
        FftEngine.ForwardInPlace(buf);
        // 解析信号保留并加倍正频率，清除负频率，直流和奈奎斯特点各保留一次。逆变换取模即得随时间变化的包络。
        for (int k = 1; k < n / 2; k++) buf[k] *= 2.0;
        for (int k = n / 2 + 1; k < n; k++) buf[k] = Complex.Zero;
        FftEngine.InverseInPlace(buf);

        double maxEnv = 1e-12;
        var env = new double[ir.Length];
        for (int i = 0; i < ir.Length; i++)
        {
            double e = buf[i].Magnitude;
            env[i] = e;
            if (e > maxEnv) maxEnv = e;
        }
        // 同一条脉冲响应的最大包络作为 0 dB，这条曲线用于比较反射随时间的衰减。
        var db = new double[ir.Length];
        for (int i = 0; i < ir.Length; i++)
            db[i] = 20.0 * Math.Log10(Math.Max(env[i] / maxEnv, 1e-6));
        return db;
    }

    // 简化的虚拟入口默认分析通道 1，得到的增益和延迟来自软件 DUT 的输入、输出样本。
    public static ContinuousSweepResult RunVirtualContinuousSweep(
        ContinuousSweepConfig config,
        VirtualDutConfig dutConfig,
        int sampleRate,
        int extraDelaySamples = 0) =>
        RunVirtualContinuousSweep(config, dutConfig, sampleRate, extraDelaySamples, InputChannelSelection.Channel1);

    public static ContinuousSweepResult RunVirtualContinuousSweep(
        ContinuousSweepConfig config,
        VirtualDutConfig dutConfig,
        int sampleRate,
        int extraDelaySamples,
        InputChannelSelection inputChannel)
    {
        // 用同一路的模拟响应和激励参考成对分析，交换输入通道时参考也必须随之切换。
        var capture = SimulateStereoSweep(config,dutConfig,sampleRate,extraDelaySamples,true,true);
        bool ch2 = inputChannel == InputChannelSelection.Channel2;
        return AnalyzeDeconvolution(ch2 ? capture.Right : capture.Left,capture.Drive.Reference(ch2),
            sampleRate,config,ch2 ? "Virtual Demo (In Ch2)" : "Virtual Demo (In Ch1)");
    }

    public static (double[] Left,double[] Right,StereoSweepDrive Drive) SimulateStereoSweep(
        ContinuousSweepConfig config,VirtualDutConfig dutConfig,int sampleRate,int extraDelaySamples,bool outCh1,bool outCh2,SignalPathConfig? path=null)
    {
        // 双路驱动前可插入确定的样本延时，再经虚拟 DUT 得到响应。这能检验分析结果是否随已知延时平移。
        var drive = StereoSweepDrive.Generate(config,sampleRate,outCh1,outCh2);
        int delay = Math.Max(0,extraDelaySamples);
        var srcL = new double[drive.Left.Length + delay + 2048];
        var srcR = new double[srcL.Length];
        Array.Copy(drive.Left,0,srcL,delay,drive.Left.Length);
        Array.Copy(drive.Right,0,srcR,delay,drive.Right.Length);

        var dstL = new double[srcL.Length];
        var dstR = new double[srcL.Length];
        // 模拟输出 EQ 在 DUT 之前，输入 EQ 在 DUT 之后。因此两段 EQ 都可影响最终观测到的幅相。
        new SignalPathEqualizer().Process(path?.OutputEq,sampleRate,srcL,srcR);
        var dut = new VirtualDutProcessor();
        dut.ProcessBlock(dutConfig, SignalGeneratorDsp.DbfsToLinearPeak(config.LevelDbfs), sampleRate, srcL, srcR, dstL, dstR);
        new SignalPathEqualizer().Process(path?.InputEq,sampleRate,dstL,dstR);

        return (dstL,dstR,drive);
    }

    public static AcousticResponseResult AnalyzeAcousticResponse(
        ReadOnlySpan<double> recordedResponse,
        ReadOnlySpan<double> referenceChirp,
        int sampleRate,
        double inputFullScaleVoltageVrms,
        AcousticResponseConfig config,
        string sourceDescription = "Live Capture")
    {
        // Acoustic SPL 仅取决于输入电压标定和麦克风灵敏度，不叠加输出标定偏移
        // 先从参考与响应恢复脉冲响应，再按声学窗口挑选要保留的到达声；群延迟留到窗口确定后才计算。
        var baseChirpCfg = config.Chirp.Clone();
        baseChirpCfg.InputFullScaleVoltageVrms = 1.0;
        baseChirpCfg.OutputFullScaleVoltageVrms = 1.0;
        var baseResult = AnalyzeDeconvolution(recordedResponse, referenceChirp, sampleRate, baseChirpCfg, sourceDescription, computeGroupDelay: false);

        baseResult=baseResult with { RubAndBuzz=RubAndBuzzAnalyzer.AnalyzeLogChirp(baseResult,baseChirpCfg,config.RubBuzzHighPassFactor) };
        double actualRefLevelDbfs = MeasureActiveReferenceLevelDbfs(referenceChirp);
        return WindowAcousticResponse(baseResult, actualRefLevelDbfs, inputFullScaleVoltageVrms, config);
    }

    public static AcousticResponseResult WindowAcousticResponse(ContinuousSweepResult baseResult, double actualRefLevelDbfs,
        double inputFullScaleVoltageVrms, AcousticResponseConfig config)
    {
        int sampleRate = baseResult.SampleRate;
        string sourceDescription = baseResult.SourceDescription;
        // 把麦克风灵敏度从 mV/Pa 换成 V/Pa，满量程电压除以灵敏度得到对应声压。相对 20 µPa 取 20log₁₀ 才有 dB SPL 的物理含义，准确性依赖输入标定。
        double micVoltsPerPa = Math.Max(config.MicSensitivityMvPerPa, 0.01) / 1000.0;
        double fullScalePa = Math.Max(inputFullScaleVoltageVrms, 1e-4) / micVoltsPerPa;
        double zeroDbfsSpl = 20.0 * Math.Log10(fullScalePa / 20e-6);
        // 绝对声压标定严格基于参考激励实测幅度，彻底隔离界面 SweepLevel 滑块改动
        double splOffset = actualRefLevelDbfs + zeroDbfsSpl;

        var ir = baseResult.ImpulseResponse;
        var irTime = baseResult.ImpulseTimeMs;
        int fftSize = NextPowerOfTwo(Math.Max(8192, ir.Length * 2));
        var unwindowedSpec = ComputeIrSpectrum(ir, fftSize);
        var band=BenchFastSweep.WithTableBand(config.Chirp,sampleRate);
        double stopHz = CheckStopFrequency(band.StopFrequencyHz, sampleRate);
        var toneFrequencies=band.ChirpType==ContinuousChirpType.FastSweep ? baseResult.ResponsePoints.Select(p=>p.FrequencyHz).ToArray() : null;
        var unwindowedSpl = ExtractResponsePointsFromSpectrum(
            unwindowedSpec, sampleRate, band.StartFrequencyHz, stopHz,
            band.OutputPoints, actualRefLevelDbfs, splOffset, requestedFrequencies:toneFrequencies);

        // 在显示 IR 数组中定位直达声主峰（兼容正延迟与负时间偏移）
        // 代码取幅度最大的脉冲作为自动窗中心。若反射强于直达声，最大峰也可能是反射；缩短右窗会降低低频分辨能力。
        int peakPosInIr = 0;
        double maxAbsIr = 0.0;
        for (int i = 0; i < ir.Length; i++)
        {
            if (Math.Abs(ir[i]) > maxAbsIr) { maxAbsIr = Math.Abs(ir[i]); peakPosInIr = i; }
        }
        double peakMs = irTime[peakPosInIr];
        double winStartMs = Math.Clamp(config.AutoWindowStart ? peakMs - Math.Max(0.1, config.WindowLeftMs) : config.ManualWindowStartMs, irTime[0], irTime[^3]);
        double winEndMs = Math.Clamp(config.WindowEndMs ?? (peakMs + Math.Max(0.2, config.WindowRightMs)), winStartMs + 1000.0/sampleRate, irTime[^1]);

        var windowedIr = (double[])ir.Clone();
        var weights = new double[ir.Length];
        Array.Fill(weights, 1.0);
        if (config.EnableIrWindow)
        {
            int startIdx = Math.Clamp((int)Math.Floor((winStartMs - irTime[0]) * sampleRate / 1000.0), 0, windowedIr.Length - 1);
            int endIdx = Math.Clamp((int)Math.Ceiling((winEndMs - irTime[0]) * sampleRate / 1000.0), startIdx + 2, windowedIr.Length - 1);
            int leftTaper = Math.Max(2, (int)((peakPosInIr - startIdx) * 0.35));
            int rightTaper = Math.Max(4, (int)((endIdx - peakPosInIr) * 0.25));

            // 窗外权重为零，两端渐变，中央保留。用相同的 FFT 比较加窗前后，可看到晚到声对各频率幅度的贡献。
            for (int i = 0; i < windowedIr.Length; i++)
            {
                if (i < startIdx || i > endIdx) weights[i] = 0.0;
                else if (i < startIdx + leftTaper)
                {
                    double t = (double)(i - startIdx) / leftTaper;
                    weights[i] = 0.5 * (1.0 - Math.Cos(Math.PI * t));
                }
                else if (i > endIdx - rightTaper)
                {
                    double t = (double)(endIdx - i) / rightTaper;
                    weights[i] = 0.5 * (1.0 - Math.Cos(Math.PI * t));
                }
            }
        }

        for (int i = 0; i < windowedIr.Length; i++) windowedIr[i] *= weights[i];
        var windowedSpec = ComputeIrSpectrum(windowedIr, fftSize);
        // 仅闭环、连续变频模式在这里求群延迟。已乘时间窗的脉冲和它的复频谱共用同一个时间原点。
        if (!config.Chirp.OpenLoop && config.Chirp.ChirpType != ContinuousChirpType.FastSweep)
            baseResult = baseResult with { GroupDelay = ChirpResultMath.GroupDelay(windowedIr, irTime[0],
                sampleRate, band.OutputPoints == 0 ? SpectrumFrequencies(band.StartFrequencyHz,stopHz,(double)sampleRate/fftSize)
                    : baseResult.ResponsePoints.Select(p => p.FrequencyHz).ToArray(), responseSpectrum: windowedSpec) };
        var windowedSpl = ExtractResponsePointsFromSpectrum(
            windowedSpec, sampleRate, band.StartFrequencyHz, stopHz,
            band.OutputPoints, actualRefLevelDbfs, splOffset, requestedFrequencies:toneFrequencies);

        // 扫频的激励幅度若不均匀，用实际记录的逐频电平修正两条 SPL 曲线。通带摘要随后取所选 200 Hz 到 4 kHz 点的 dB 算术平均。
        var levels = baseResult.ResponsePoints.Where(p => double.IsFinite(p.ExcitationToneDbfs)).OrderBy(p=>p.FrequencyHz).ToArray();
        if (levels.Length > 0)
        {
            var table = new BenchTrace(levels.Select(p=>p.FrequencyHz).ToArray(),levels.Select(p=>p.ExcitationToneDbfs).ToArray());
            foreach (var series in new[]{windowedSpl,unwindowedSpl})
                for (int i=0;i<series.Count;i++)
                {
                    var p=series[i];
                    double correction=FrequencyGainTable.GainAtHz(table,p.FrequencyHz)-actualRefLevelDbfs;
                    series[i]=p with {Ch1GainDb=p.Ch1GainDb+correction,Ch2GainDb=p.Ch2GainDb+correction,
                        Ch1LevelDbfs=p.Ch1LevelDbfs+correction,Ch2LevelDbfs=p.Ch2LevelDbfs+correction};
                }
        }
        var passband = windowedSpl.Where(p => p.FrequencyHz >= 200.0 && p.FrequencyHz <= 4000.0).ToList();
        double avgSpl = passband.Count > 0 ? passband.Average(p => p.Ch1GainDb) : 0.0;

        return new AcousticResponseResult(
            windowedSpl, unwindowedSpl, ir, baseResult.ImpulseTimeMs,
            peakMs, winStartMs, winEndMs, baseResult.EstimatedDelayMs, avgSpl,
            baseResult.WasClipping, sourceDescription, actualRefLevelDbfs, baseResult, weights);
    }

    private static Complex[] ComputeIrSpectrum(double[] ir, int fftSize)
    {
        // 实脉冲响应放入复数缓冲，空余样本补零。变换得到的复响应同时保留幅度和相位信息。
        var buf = new Complex[fftSize];
        int copyLen = Math.Min(ir.Length, fftSize);
        for (int i = 0; i < copyLen; i++) buf[i] = new Complex(ir[i], 0.0);
        FftEngine.ForwardInPlace(buf);
        return buf;
    }

    // Preserve every bin inside the sweep. Only the two requested band edges may fall between bins.
    private static double[] SpectrumFrequencies(double start, double stop, double binHz)
    {
        // 保留区间内部各个 FFT 格点，另外加入恰好等于用户设置的起止频率，边缘点可能需要插值。
        int first=(int)Math.Floor(start/binHz)+1;
        int last=(int)Math.Ceiling(stop/binHz)-1;
        var result=new double[Math.Max(0,last-first+1)+2];
        result[0]=start;
        for (int i=first;i<=last;i++) result[1+i-first]=i*binHz;
        result[^1]=stop;
        return result;
    }

    private static double SpectrumGain(double hz, double binHz, int fftSize, Func<int,double> magnitudeAt, double offset)
    {
        // 将目标频率除以 binHz 得到非整数格点，在相邻线性幅值上插值后转成 dB。改变插值与取对数的先后会改变点间结果。
        double position = Math.Clamp(hz / binHz, 1, fftSize / 2 - 1);
        int low = (int)position;
        double a = magnitudeAt(low);
        if (position != low)
            a += (position-low)*(magnitudeAt(Math.Min(low+1,fftSize/2-1))-a);
        return 20*Math.Log10(a)+offset;
    }

    private static List<FrequencyResponsePoint> ExtractResponsePointsFromSpectrum(
        Complex[] hSpec, int sampleRate, double startHz, double stopHz, int pointCount, double excitationLevelDbfs,
        double gainOffsetDb, ContinuousPhaseMode? phaseMode = null, int delaySamples = 0,
        Func<int, double>? measuredMagnitude = null, double[]? requestedFrequencies = null, bool alreadyPeakAligned = false)
    {
        // 先建立输出频率网格，再让幅度、相位和相对参考共用它，确保同一行各列对应同一物理频率。
        int fftSize = hSpec.Length;
        double binHz = (double)sampleRate / fftSize;
        double[] freqs = requestedFrequencies ?? (pointCount == 0 ? SpectrumFrequencies(startHz,stopHz,binHz) : FrequencyResponseSweeper.GenerateLogFrequencies(
            Math.Max(10.0, startHz), stopHz, Math.Clamp(pointCount, 20, 4096)));
        if (requestedFrequencies == null) freqs[^1] = stopHz;
        var phases = phaseMode is { } mode
            ? ExtractChirpPhases(hSpec, freqs, binHz, delaySamples, mode, alreadyPeakAligned)
            : null;

        var raw = new List<FrequencyResponsePoint>(freqs.Length);
        int halfLen = fftSize / 2;
        var magnitudeAt = measuredMagnitude ?? (k => hSpec[k].Magnitude);
        // 在给定频率上评估线性幅度并换算成 dB，叠加 I/O 量程标定；相对量的参考限制在实际扫频范围内。
        double Gain(double hz) => SpectrumGain(hz,binHz,fftSize,magnitudeAt,gainOffsetDb);
        double refGainDb = Gain(Math.Clamp(1000.0, startHz, stopHz));

        for (int i = 0; i < freqs.Length; i++)
        {
            double f = freqs[i];
            int bin = Math.Clamp((int)Math.Round(f / binHz), 1, halfLen - 1);
            Complex h = hSpec[bin];
            double gainDb = Gain(f);
            double phaseDeg = phases?[i] ??
                FrequencyResponseSweeper.NormalizeAngleDeg(h.Phase * (180.0 / Math.PI));

            raw.Add(new FrequencyResponsePoint(
                f, gainDb + excitationLevelDbfs, gainDb + excitationLevelDbfs,
                gainDb, gainDb, gainDb - refGainDb, gainDb - refGainDb,
                phaseDeg, phaseDeg, phaseDeg, XValue: f, IsValid: double.IsFinite(gainDb)));
        }

        return raw;
    }

    private static double[] ExtractChirpPhases(
        Complex[] spectrum, double[] frequencies, double binHz, int delaySamples, ContinuousPhaseMode mode, bool alreadyPeakAligned = false)
    {
        // 纯延时给出与频率成正比的相位斜率。先按脉冲峰延时对齐，再沿相邻频点连续展开，可避免跨越 ±180° 时错误地平均到 0°。
        double delayDegreesPerBin = 360.0 * delaySamples / spectrum.Length;
        int lastBin = spectrum.Length / 2 - 1;
        // 定位目标频率下方的频格，预留上方一格供相位插值。
        int BinBelow(double frequency) => Math.Clamp((int)Math.Floor(frequency / binHz), 1, lastBin - 1);
        // 若频谱尚未围绕主峰对齐，先加上与整采样时延相反的线性相位；已对齐的频谱直接取相角。
        double PhaseWithPeakAligned(int bin) => FrequencyResponseSweeper.NormalizeAngleDeg(
            spectrum[bin].Phase * (180.0 / Math.PI) + (alreadyPeakAligned ? 0.0 : bin * delayDegreesPerBin));

        var result = new double[frequencies.Length];
        bool unwrap = mode is ContinuousPhaseMode.Unwrapped or ContinuousPhaseMode.RelativeToCh1 or ContinuousPhaseMode.Excess;
        bool ascending = frequencies[0] <= frequencies[^1];
        int cursor = BinBelow(ascending ? frequencies[0] : frequencies[^1]);
        double previousPhase = PhaseWithPeakAligned(cursor);
        double unwrappedPhase = previousPhase;

        for (int orderedIndex = 0; orderedIndex < frequencies.Length; orderedIndex++)
        {
            int i = ascending ? orderedIndex : frequencies.Length - 1 - orderedIndex;
            double position = frequencies[i] / binHz;
            int bin = BinBelow(frequencies[i]);
            double below = PhaseWithPeakAligned(bin);
            double above = PhaseWithPeakAligned(bin + 1);

            // 去除已知整采样延迟后再插值。展开模式扫描密集 FFT 频点，避免稀疏绘图点漏掉整圈。
            if (unwrap)
            {
                while (cursor < bin)
                {
                    double nextPhase = PhaseWithPeakAligned(++cursor);
                    unwrappedPhase += FrequencyResponseSweeper.NormalizeAngleDeg(nextPhase - previousPhase);
                    previousPhase = nextPhase;
                }
            }
            // 相邻格点的相位增量先折回单圈，再按目标格内位置插值。例如 +179° 与 −179° 在此只相隔 2°。
            double delta = FrequencyResponseSweeper.NormalizeAngleDeg(above - below);
            double phase = (unwrap ? unwrappedPhase : below) + Math.Clamp(position - bin, 0.0, 1.0) * delta;
            if (mode != ContinuousPhaseMode.IrPeakAligned)
                phase -= position * delayDegreesPerBin;
            result[i] = unwrap ? phase : FrequencyResponseSweeper.NormalizeAngleDeg(phase);
        }

        return result;
    }

    // 上取整到 2 的幂，使补零后的缓冲既容纳所需样本数，又能交给基二 FFT。
    private static int NextPowerOfTwo(int v) { int p = 1; while (p < v) p <<= 1; return p; }

    /// <summary>
    /// 对齐 APx500 信号路径输入滤波（High-Pass 含 AC &lt;10 Hz 交流耦合、Low-Pass、Weighting）：
    /// 沿连续采集波形就地运行，使连续扫描与实时仪表共享完全一致的输入滤波链路。
    /// </summary>
    public static void ApplySignalPathFilters(double[] samples, int sampleRate, AnalyzerFilterConfig filter, bool channel2)
    {
        if (samples.Length == 0 || sampleRate <= 0) return;
        bool hpOn = channel2 ? filter.Ch2HighPassEnabled : filter.Ch1HighPassEnabled;
        bool lpOn = channel2 ? filter.Ch2LowPassEnabled : filter.Ch1LowPassEnabled;
        bool wtOn = channel2 ? filter.Ch2WeightingEnabled : filter.Ch1WeightingEnabled;

        if (hpOn && filter.HighPassHz > 0.0)
        {
            if (Math.Abs(filter.HighPassHz - 10.0) < 0.05 && filter.HighPassType == HighPassFilterType.Butterworth4)
            {
                // 对齐 APx500 Bench Mode AC (<10 Hz)：直接沿连续采集流运行 ApxAcCouplingFilter；
                // 不对前段信号盲估 DC（避免在短前导输入中把 20 Hz 正弦首半周误当直流预充）。
                new ApxAcCouplingFilter(sampleRate).Process(samples, samples);
            }
            else
            {
                var hp = filter.HighPassType == HighPassFilterType.Elliptic5
                    ? ApxEllipticLowPass.CreateEllipticHighPassStereo(filter.HighPassHz, sampleRate).Left
                    : StreamingIirCascade.CreateButterworthHighPass(4, filter.HighPassHz, sampleRate);
                hp.ProcessInPlace(samples);
            }
        }
        if (lpOn && filter.LowPassHz > 0.0)
        {
            var lp = filter.LowPassType is LowPassFilterType.Aes17 or LowPassFilterType.Elliptic8
                ? ApxEllipticLowPass.CreateStereo(ApxEllipticLowPass.RelativeCutoff(filter.LowPassHz, sampleRate), sampleRate).Left
                : StreamingIirCascade.CreateButterworthLowPass(8, filter.LowPassHz, sampleRate);
            lp.ProcessInPlace(samples);
        }
        if (wtOn && filter.Weighting != AnalyzerWeighting.Flat)
        {
            var wt = ApxAnsiWeighting.Create(filter.Weighting, sampleRate);
            wt.ProcessInPlace(samples);
        }
    }
}
