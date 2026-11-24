// 同时有参考输入 x 和测量输出 y 时，用两路的分段频谱来估计复数传递函数。只有输入确实激励了该频点，输出与输入的比值才有测量意义；图层会收到逐点有效性。
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// 双通道传递函数分析器（Welch FRF 幅频/相频 + 相干度 γ² + 系统延迟）。
/// <para>
/// 严格防误导保护：
/// 1. 拒绝单声道复制或同一物理通道映射到两路的伪双通道输入（<c>hasIndependentChannels == false</c>）；
/// 2. 逐频段校验参考功率谱 <c>S_xx(f)</c> 能量有效性：无激励频段标为 <c>IsValid = false</c>（NaN / null），
///    不计入平均相干度，且图上不跨越无效频段连线；
/// 3. 识别窄带单音参考（有效激励频段占比 &lt; 15%）：仅报告受激频段，不输出虚假的宽带延迟结论。
/// </para>
/// </summary>
public static class TransferFunctionAnalyzer
{
    // 可选 FFT 长度受 Bench 记录表限制，并且至少 512 点。实际频格间距是 Fs/N。
    private static int ValidateLength(int n) => n >= 512 && FftEngine.BenchLengths.Contains(n) ? n :
        throw new ArgumentOutOfRangeException(nameof(n), "Unsupported transfer FFT length.");

    // 邻窗重叠越多，每次新增样本越少。步进以样本表示，最少 128 点。
    private static int GetHop(int fftSize, double overlap) =>
        Math.Max(128, (int)(fftSize * (1.0 - Math.Clamp(overlap, 0.0, 0.75))));

    /// <summary>多通道传递函数分析（同一参考通道对多个测量通道独立执行 FRF 估计）。</summary>
    public static TransferFunctionResult[] AnalyzeMultiChannel(
        ReadOnlySpan<double> referenceSignal,
        IReadOnlyList<double[]> measuredChannels,
        int sampleRate,
        TransferFunctionConfig config,
        bool hasIndependentChannels = true,
        string referenceRoutingDescription = "Ref → Multi-Ch",
        CancellationToken cancellationToken = default)
    {
        var results = new TransferFunctionResult[measuredChannels.Count];
        for (int ch = 0; ch < measuredChannels.Count; ch++)
        {
            results[ch] = Analyze(referenceSignal, measuredChannels[ch], sampleRate, config,
                hasIndependentChannels, $"{referenceRoutingDescription} [Ch{ch + 1}]", cancellationToken);
        }
        return results;
    }

    // 第一帧用 N 点，每增加一次平均只需再取 hop 点，M 帧共需 N+(M−1)hop。
    public static int RequiredSamples(TransferFunctionConfig config)
    {
        int fftSize = ValidateLength(config.FftSize);
        return fftSize + GetHop(fftSize, config.OverlapRatio) * Math.Max(0, Math.Clamp(config.Averages, 0, 1000) - 1);
    }

    public static TransferFunctionResult Analyze(
        ReadOnlySpan<double> referenceSignal,
        ReadOnlySpan<double> measuredSignal,
        int sampleRate,
        TransferFunctionConfig config,
        bool hasIndependentChannels = true,
        string referenceRoutingDescription = "Ref → Meas", CancellationToken cancellationToken = default)
    {
        // 传递函数需要参考与响应两路观测。若把同一路复制两次，大多数被激励频率都会呈现接近 1 的假传递函数。
        if (!hasIndependentChannels)
        {
            return new TransferFunctionResult(
                false,
                "拒绝双通道 FRF：当前输入为单声道复制或两路映射到同一物理通道，缺少独立参考与测量通道。",
                Array.Empty<FrequencyResponsePoint>(),
                double.NaN, double.NaN, double.NaN,
                0, 0, false, referenceRoutingDescription);
        }

        int len = Math.Min(referenceSignal.Length, measuredSignal.Length);
        int fftSize = ValidateLength(config.FftSize);
        // 指定平均次数时，必须有足够的同步样本才能构成全部完整帧；未指定时至少要够一帧。
        if (config.Averages > 0 && len < RequiredSamples(config))
            return new TransferFunctionResult(false, $"等待 {RequiredSamples(config)} 点以完成所选 Welch 平均。",
                [], double.NaN, double.NaN, double.NaN, HasValidOutput: false, HasValidDelay: false);
        if (len < fftSize)
            return new TransferFunctionResult(false, $"The selected FFT length requires {fftSize} samples; received {len}.",
                [], double.NaN, double.NaN, double.NaN, HasValidOutput: false, HasValidDelay: false);

        if (len < 512 || fftSize < 512)
            return new TransferFunctionResult(false, "样本长度不足（需 >= 512 点）", Array.Empty<FrequencyResponsePoint>(), double.NaN, double.NaN, double.NaN, 0, 0, false, referenceRoutingDescription);

        // 检查参考通道与测量通道各自交流 RMS（分开判定参考有效性与输出有效性）
        double refMean = 0.0, measMean = 0.0;
        for (int i = 0; i < len; i++) { refMean += referenceSignal[i]; measMean += measuredSignal[i]; }
        refMean /= len;
        measMean /= len;
        double refSumSq = 0.0, measSumSq = 0.0;
        for (int i = 0; i < len; i++)
        {
            double acRef = referenceSignal[i] - refMean;
            double acMeas = measuredSignal[i] - measMean;
            refSumSq += acRef * acRef;
            measSumSq += acMeas * acMeas;
        }
        double refRmsDbfs = SignalGeneratorDsp.LinearToDbfs(Math.Sqrt(refSumSq / len) * AudioMetricsAnalyzer.Sqrt2);
        double measRmsDbfs = SignalGeneratorDsp.LinearToDbfs(Math.Sqrt(measSumSq / len) * AudioMetricsAnalyzer.Sqrt2);
        if (refRmsDbfs < config.MinReferencePowerDbfs)
            return new TransferFunctionResult(false, $"参考通道无有效信号（{refRmsDbfs:F1} dBFS < {config.MinReferencePowerDbfs:F1} dBFS）", Array.Empty<FrequencyResponsePoint>(), double.NaN, double.NaN, double.NaN, 0, 0, false, referenceRoutingDescription, HasValidOutput: false, HasValidDelay: false);

        bool hasValidOutput = measRmsDbfs >= config.MinReferencePowerDbfs;

        int hop = GetHop(fftSize, config.OverlapRatio);
        int firstOffset = config.Averages > 0 ? len - RequiredSamples(config) : 0;
        var window = FftEngine.CreateWindow(fftSize, config.WindowType);

        int halfLen = fftSize / 2 + 1;
        var sxx = new double[halfLen];
        var syy = new double[halfLen];
        var sxy = new Complex[halfLen];
        var fullH = new Complex[fftSize];

        int frameCount = 0;
        var bufX = new Complex[fftSize];
        var bufY = new Complex[fftSize];

        // 对同一时刻的两路信号取等长窗口、去各自均值、乘同一窗。只有这样，交叉谱保留的相位差才对应同一段数据。
        for (int offset = firstOffset; offset + fftSize <= len; offset += hop)
        {
            cancellationToken.ThrowIfCancellationRequested();
            double meanX = 0.0, meanY = 0.0;
            for (int i = 0; i < fftSize; i++)
            {
                meanX += referenceSignal[offset + i];
                meanY += measuredSignal[offset + i];
            }
            meanX /= fftSize;
            meanY /= fftSize;

            for (int i = 0; i < fftSize; i++)
            {
                double w = window.Coefficients[i];
                bufX[i] = new Complex((referenceSignal[offset + i] - meanX) * w, 0.0);
                bufY[i] = new Complex((measuredSignal[offset + i] - meanY) * w, 0.0);
            }

            FftEngine.ForwardInPlace(bufX);
            FftEngine.ForwardInPlace(bufY);

            // 自谱 |X|²、|Y|² 描述各路能量，交叉谱保留两路相位关系。先跨帧累加这些量，再求比值，可降低不相关噪声的影响。
            for (int k = 0; k < halfLen; k++)
            {
                double px = bufX[k].Real * bufX[k].Real + bufX[k].Imaginary * bufX[k].Imaginary;
                double py = bufY[k].Real * bufY[k].Real + bufY[k].Imaginary * bufY[k].Imaginary;
                sxx[k] += px;
                syy[k] += py;
                // 若被测电路在这个频点使 Y=H·X，两边乘 conj(X) 得 Y·conj(X)=H·|X|²。
                // 于是跨帧累计左边，再除以累计的输入功率，就得到 H1 的幅值和相位。
                sxy[k] += bufY[k] * Complex.Conjugate(bufX[k]);
            }
            frameCount++;
        }

        if (frameCount == 0)
            return new TransferFunctionResult(false, "未能形成完整 Welch 分析帧", Array.Empty<FrequencyResponsePoint>(), double.NaN, double.NaN, double.NaN, 0, 0, false, referenceRoutingDescription);

        // 将单帧平均自谱换算为绝对单边功率以逐频段判定有效激励
        double powerNorm = 4.0 / ((double)fftSize * fftSize * window.NoisePowerGain * frameCount);
        double maxSxx = 0.0;
        for (int k = 1; k < halfLen - 1; k++)
            if (sxx[k] > maxSxx) maxSxx = sxx[k];

        // 参考有激励的频点才能推断增益。相对门槛排除远弱于谱峰的点，绝对门槛排除接近底噪的点，取较严格者。
        double relPowerFloor = maxSxx * Math.Pow(10.0, -Math.Clamp(config.DynamicRangeBelowPeakDb, 15.0, 90.0) / 10.0);
        double absPowerFloor = Math.Pow(10.0, config.MinReferencePowerDbfs / 10.0) / Math.Max(powerNorm, 1e-24);
        double binValidThreshold = Math.Max(relPowerFloor, absPowerFloor);
        double eps = Math.Max(maxSxx * 1e-10, 1e-24);

        // 输出可以保留 20 Hz 到可用上限内的每个格点，也可将对数间隔的目标频率吸附到最近格点并去重。
        double binHz = (double)sampleRate / fftSize;
        int firstBandBin = Math.Max(1,(int)Math.Ceiling(20/binHz));
        int lastBandBin = Math.Min(halfLen-1,(int)Math.Floor(20000/binHz));
        double[] freqs = config.OutputPoints == 0
            ? Enumerable.Range(firstBandBin,lastBandBin-firstBandBin+1).Select(k=>k*binHz).ToArray()
            : FrequencyResponseSweeper.GenerateLogFrequencies(20.0, Math.Min(20000.0, sampleRate * 0.45), Math.Clamp(config.OutputPoints, 20, 300))
            .Select(f => Math.Clamp((int)Math.Round(f / binHz), 1, halfLen - 2)).Distinct()
            .Select(k => k * binHz).ToArray();
        double calibrationDb = AudioMetricsAnalyzer.ComputeCalibrationGainOffsetDb(
            config.MeasurementFullScaleVoltageVrms, config.ReferenceFullScaleVoltageVrms);

        // 若 Y=HX，则 conj(X)Y≈H|X|²。H1 用交叉谱除以输入自谱，H2 用输出自谱除以交叉谱的共轭；选择幅值模式时只保留功率比的平方根。
        Complex Estimate(int k) => config.CalculationMode switch
        {
            TransferFunctionCalculationMode.H2 => sxy[k].Magnitude > 1e-24
                ? syy[k] / Complex.Conjugate(sxy[k]) : new Complex(double.NaN, double.NaN),
            TransferFunctionCalculationMode.MagnitudeOnly => new Complex(Math.Sqrt(syy[k] / Math.Max(sxx[k], 1e-24)), 0),
            _ => sxy[k] / (sxx[k] + eps)
        };

        // 确定参考 1 kHz 或首个有效频段用于 dBr 归一化
        int ref1kBin = Math.Clamp((int)Math.Round(1000.0 / binHz), 1, halfLen - 2);
        double? refGainForRelative = sxx[ref1kBin] >= binValidThreshold
            ? 20.0 * Math.Log10(Math.Max(Estimate(ref1kBin).Magnitude, 1e-9)) + calibrationDb
            : null;

        var points = new List<FrequencyResponsePoint>(freqs.Length);
        double cohSum = 0.0;
        int validBandCount = 0;

        foreach (double f in freqs)
        {
            int k = Math.Clamp((int)Math.Round(f / binHz), 1, halfLen - 1);
            // 若输入在该频点几乎没能量，输出噪声除以很小的数也会给出很大的比值。
            // 门限检验的是参考输入谱；参考有效但响应过弱时，保留该点受激标记，把不可用的增益或相位单独报告。
            bool binExcited = sxx[k] >= binValidThreshold;

            if (!binExcited)
            {
                // 无激励频段明确标为无效（IsValid=false, NaN / null），绝不输出 -159 dB 虚假点
                points.Add(new FrequencyResponsePoint(
                    FrequencyHz: f,
                    Ch1LevelDbfs: double.NaN,
                    Ch2LevelDbfs: double.NaN,
                    Ch1GainDb: double.NaN,
                    Ch2GainDb: double.NaN,
                    Ch1RelativeDb: double.NaN,
                    Ch2RelativeDb: double.NaN,
                    Ch1PhaseDeg: null,
                    Ch2PhaseDeg: null,
                    MeasuredPhaseDeg: null,
                    XValue: f,
                    Coherence: null,
                    IsValid: false));
                continue;
            }

            if (!hasValidOutput)
            {
                // 参考有效但测量通道静音：保留受激频段计数，相位不可用、相干度为 0，增益标为 NaN/不可用
                validBandCount++;
                points.Add(new FrequencyResponsePoint(
                    FrequencyHz: f,
                    Ch1LevelDbfs: double.NaN,
                    Ch2LevelDbfs: double.NaN,
                    Ch1GainDb: double.NaN,
                    Ch2GainDb: double.NaN,
                    Ch1RelativeDb: double.NaN,
                    Ch2RelativeDb: double.NaN,
                    Ch1PhaseDeg: null,
                    Ch2PhaseDeg: null,
                    MeasuredPhaseDeg: null,
                    XValue: f,
                    Coherence: 0.0,
                    IsValid: false));
                continue;
            }

            Complex h = Estimate(k);
            double gainDb = 20.0 * Math.Log10(Math.Max(h.Magnitude, 1e-9)) + calibrationDb;
            double? phaseDeg = config.CalculationMode == TransferFunctionCalculationMode.MagnitudeOnly ? null
                : FrequencyResponseSweeper.NormalizeAngleDeg(Math.Atan2(h.Imaginary, h.Real) * (180.0 / Math.PI));

            // 相干度为 |Sxy|²/(Sxx·Syy)。不同帧的相位关系越稳定，交叉谱越能同向累加；单帧的这个比值几乎恒为 1，不能单凭它断定关系稳定。
            double sxyMagSq = sxy[k].Real * sxy[k].Real + sxy[k].Imaginary * sxy[k].Imaginary;
            double denom = sxx[k] * syy[k];
            double coherence = denom > 1e-24 ? Math.Clamp(sxyMagSq / denom, 0.0, 1.0) : 0.0;

            refGainForRelative ??= gainDb;
            cohSum += coherence;
            validBandCount++;

            points.Add(new FrequencyResponsePoint(
                FrequencyHz: f,
                Ch1LevelDbfs: gainDb,
                Ch2LevelDbfs: gainDb,
                Ch1GainDb: gainDb,
                Ch2GainDb: gainDb,
                Ch1RelativeDb: gainDb - refGainForRelative.Value,
                Ch2RelativeDb: gainDb - refGainForRelative.Value,
                Ch1PhaseDeg: phaseDeg,
                Ch2PhaseDeg: phaseDeg,
                MeasuredPhaseDeg: phaseDeg,
                XValue: f,
                Coherence: frameCount >= 2 ? coherence : null,
                IsValid: true));
        }

        if (validBandCount == 0)
        {
            return new TransferFunctionResult(
                false, "参考通道在 20 Hz–20 kHz 内无任何有效激励频段。",
                points, double.NaN, double.NaN, double.NaN,
                0, freqs.Length, true, referenceRoutingDescription, HasValidOutput: hasValidOutput, HasValidDelay: false);
        }

        // 被激励的点太少时，频域仍可给出局部读数，但很难由一条窄带谱唯一定位时域脉冲。
        double excitedFraction = (double)validBandCount / freqs.Length;
        bool isNarrowband = excitedFraction < 0.15;
        double meanCoh = frameCount < 2 ? double.NaN : hasValidOutput ? cohSum / validBandCount : 0.0;

        // 仅在参考宽带、测量通道有有效输出、相干度达标且 IR 存在显著主峰时报告延迟（绝不在静音或低相干下误报 0 延迟）
        double delaySamples = double.NaN;
        double delayMs = double.NaN;
        bool hasValidDelay = false;
        string? outputNote = null;
        double[]? impulse = null;

        if (!hasValidOutput)
        {
            outputNote = $"测量通道静音/无有效输出（{measRmsDbfs:F1} dBFS），相位与延迟不可用（N/A）。";
        }
        else if (config.CalculationMode == TransferFunctionCalculationMode.MagnitudeOnly)
        {
            outputNote = "Magnitude Only 不提供相位或脉冲延迟。";
        }
        else if (!isNarrowband)
        {
            // 实值信号的负频率是正频率的共轭。补齐另一半频谱、将未被激励的频点置零，再逆变换得到有限带宽的脉冲响应。
            for (int k = 0; k < halfLen; k++)
            {
                Complex h = sxx[k] >= binValidThreshold ? Estimate(k) : Complex.Zero;
                if (!double.IsFinite(h.Real) || !double.IsFinite(h.Imaginary)) h = Complex.Zero;
                fullH[k] = h;
                if (k > 0 && k < halfLen - 1) fullH[fftSize - k] = Complex.Conjugate(h);
            }

            var irBuf = new Complex[fftSize];
            for (int i = 0; i < fftSize; i++) irBuf[i] = Complex.Conjugate(fullH[i]);
            FftEngine.ForwardInPlace(irBuf);

            // Convert the unscaled inverse transform into a voltage-ratio impulse response.
            double scale = Math.Pow(10, calibrationDb / 20) / fftSize;
            impulse = new double[fftSize];
            int peakIdx = 0;
            double peakVal = 0.0;
            double irSumSq = 0.0;
            for (int i = 0; i < fftSize; i++)
            {
                double r = irBuf[i].Real;
                impulse[i] = r * scale;
                irSumSq += r * r;
                double a = Math.Abs(r);
                if (a > peakVal) { peakVal = a; peakIdx = i; }
            }

            double irRms = Math.Sqrt(irSumSq / fftSize);
            double irCrest = irRms > 1e-18 ? peakVal / irRms : 0.0;

            // 这里用绝对峰值除以整条脉冲的 RMS，不是用底噪作分母。连同帧数和平均相干度通过门限才报告时延。
            if (frameCount >= 2 && meanCoh >= 0.25 && peakVal > 1e-6 && irCrest >= 5.0)
            {
                hasValidDelay = true;
                // 后半段的峰按负延时解释。先给出带符号的样本差，除以 Fs 并乘 1000 得毫秒。
                delaySamples = peakIdx <= fftSize / 2 ? peakIdx : peakIdx - fftSize;
                delayMs = delaySamples * 1000.0 / sampleRate;
            }
            else
            {
                outputNote = frameCount < 2 ? "Welch 帧不足，无法验证相干度或延迟。"
                    : $"相干度过低（γ²={meanCoh:F3}）或无稳定脉冲峰（峰均比 {irCrest:F1}），延迟不可用（N/A）。";
            }
        }

        string? narrowbandReason = isNarrowband
            ? $"检测到窄带参考（仅 {validBandCount}/{freqs.Length} 频段受激）；仅显示受激频段，请使用宽带激励（如白/粉红噪声或 Chirp）获取完整 FRF 与延迟。"
            : null;

        var binFrequencies = new double[halfLen];
        var refFft = new double[halfLen]; var inputFft = new double[halfLen];
        var refPsd = new double[halfLen]; var inputPsd = new double[halfLen];
        var corr = new Complex[fftSize];
        double refEnergy = 0, inputEnergy = 0;
        double windowSum = window.Coefficients.Sum();
        // 幅度谱按窗幅值归一，功率谱密度还要按采样率和窗能量归一。单边谱中直流和奈奎斯特频点没有镜像伙伴，权重不同。
        double psdScale = frameCount * sampleRate * fftSize * window.NoisePowerGain;
        // 正弦功率除以帧数后按窗和的平方校准。内部频格要合并两侧振幅，因而此处用 factor²；下方的功率密度仅用 factor。
        double ToneDb(double power, int factor) => power > 0
            ? 10 * Math.Log10(power / frameCount * factor * factor / (windowSum * windowSum)) : double.NegativeInfinity;
        for (int k = 0; k < halfLen; k++)
        {
            int factor = k == 0 || k == fftSize / 2 ? 1 : 2;
            binFrequencies[k] = k * binHz;
            refFft[k] = ToneDb(sxx[k], factor);
            inputFft[k] = ToneDb(syy[k], factor);
            refPsd[k] = factor * sxx[k] / psdScale;
            inputPsd[k] = factor * syy[k] / psdScale;
            refEnergy += factor * sxx[k];
            inputEnergy += factor * syy[k];
            corr[k] = sxy[k];
            if (factor == 2) corr[fftSize-k] = Complex.Conjugate(sxy[k]);
        }
        FftEngine.InverseInPlace(corr);
        // 互谱逆变换给出相关随滞后的变化。把零延时移到横轴中央，并用两路能量做归一化，便于比较不同输入电平的记录。
        var lag = new double[fftSize];
        var correlation = new double[fftSize];
        // 互谱这里对应窗内的循环相关，横轴仅覆盖一个 FFT 周期。除以两路能量的几何平均后得到无量纲相似度。
        double normalization = Math.Sqrt(refEnergy * inputEnergy) / fftSize;
        for (int i = 0; i < fftSize; i++)
        {
            int n = i - fftSize / 2;
            lag[i] = 1000.0 * n / sampleRate;
            correlation[i] = normalization > 0 ? corr[(n+fftSize) % fftSize].Real / normalization : double.NaN;
        }
        var spectra = new TransferSpectra(binFrequencies, inputFft, refFft, inputPsd, refPsd, lag, correlation);

        return new TransferFunctionResult(
            HasValidReference: true,
            InvalidReason: narrowbandReason,
            FrfPoints: points,
            MeanCoherence: meanCoh,
            EstimatedDelayMs: delayMs,
            EstimatedDelaySamples: delaySamples,
            ValidBandCount: validBandCount,
            TotalBandCount: freqs.Length,
            IsNarrowbandReference: isNarrowband,
            ReferenceRoutingDescription: referenceRoutingDescription,
            HasValidOutput: hasValidOutput,
            HasValidDelay: hasValidDelay,
            OutputStatusNote: outputNote,
            ImpulseResponse: impulse, Spectra: spectra);
    }
}
