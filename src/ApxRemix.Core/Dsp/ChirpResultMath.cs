using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public static class ChirpResultMath
{
    private const int InterpHalfLen = 63;
    private const int InterpTapCount = 2 * InterpHalfLen + 1;
    private const int InterpPhases = 256;
    private static readonly double[,] SincInterpTable = BuildSincInterpTable();

    private static double[,] BuildSincInterpTable()
    {
        // 对齐 APx500 AudioPrecision.Processing.Interpolation.Interp（127 抽头加窗 sinc，256 档子采样相位表）。
        // 采用 Kaiser(β=10.4) 窗截断 sinc，并按每档和归一化，可在 [0, 1] 相位上达到 < 1/256 采样点的峰值定位精度。
        const double beta = 10.4;
        const double halfSpan = InterpHalfLen + 0.5;
        double invI0Beta = 1.0 / BesselI0(beta);
        var table = new double[InterpPhases + 1, InterpTapCount];
        for (int p = 0; p <= InterpPhases; p++)
        {
            if (p == 0) { table[p, InterpHalfLen] = 1.0; continue; }
            if (p == InterpPhases) { table[p, InterpHalfLen + 1] = 1.0; continue; }
            double frac = (double)p / InterpPhases;
            double sum = 0.0;
            for (int j = 0; j < InterpTapCount; j++)
            {
                double x = (j - InterpHalfLen) - frac;
                double sinc = Math.Abs(x) < 1e-12 ? 1.0 : Math.Sin(Math.PI * x) / (Math.PI * x);
                double u = x / halfSpan;
                double w = Math.Abs(u) <= 1.0 ? BesselI0(beta * Math.Sqrt(1.0 - u * u)) * invI0Beta : 0.0;
                double tap = sinc * w;
                table[p, j] = tap;
                sum += tap;
            }
            if (sum != 0.0)
                for (int j = 0; j < InterpTapCount; j++) table[p, j] /= sum;
        }
        return table;
    }

    private static double BesselI0(double x)
    {
        double sum = 1.0, term = 1.0, y = 0.25 * x * x;
        for (int k = 1; k < 40; k++)
        {
            term *= y / (k * k);
            sum += term;
            if (term <= 1e-16 * sum) break;
        }
        return sum;
    }

    /// <summary>
    /// 对齐 APx500 <c>Interp.PeakInterp</c>：围绕整采样峰值从 0.5 步长二分细化至 1/256 采样点，
    /// 用 127 抽头加窗 sinc 评估绝对幅值极大点，返回子采样峰值位置。
    /// </summary>
    internal static double PeakInterp(ReadOnlySpan<double> dataIn, int peakIndex)
    {
        if (dataIn.Length == 0) return 0.0;
        int n = dataIn.Length;
        int idx = Math.Clamp(peakIndex, 0, n - 1);

        static double Eval(ReadOnlySpan<double> data, double pos)
        {
            int len = data.Length;
            int baseIdx = (int)Math.Floor(pos);
            int phase = (int)Math.Round((pos - baseIdx) * InterpPhases, MidpointRounding.AwayFromZero);
            if (phase >= InterpPhases) { baseIdx++; phase -= InterpPhases; }
            else if (phase < 0) { baseIdx--; phase += InterpPhases; }
            if (phase == 0) return (uint)baseIdx < (uint)len ? data[baseIdx] : 0.0;

            double acc = 0.0;
            int start = baseIdx - InterpHalfLen;
            for (int j = 0; j < InterpTapCount; j++)
            {
                int src = start + j;
                if ((uint)src < (uint)len) acc += SincInterpTable[phase, j] * data[src];
            }
            return acc;
        }

        double bestPos = idx;
        for (double step = 0.5; step >= 1.0 / InterpPhases; step *= 0.5)
        {
            double c0 = bestPos - step;
            double c1 = bestPos;
            double c2 = bestPos + step;
            double v0 = Math.Abs(Eval(dataIn, c0));
            double v1 = Math.Abs(Eval(dataIn, c1));
            double v2 = Math.Abs(Eval(dataIn, c2));
            if (v0 > v1 && v0 >= v2) bestPos = c0;
            else if (v2 > v1 && v2 > v0) bestPos = c2;
        }
        return bestPos;
    }

    /// <summary>
    /// 对齐 APx500 <c>BoxcarDecimator.SimpleBoxcarDecimation</c>：
    /// 将第 k 次谐波在发射频率轴 k·f 上的幅值谱按每 k 格做等权盒式平均并压缩到激励基频轴 f。
    /// </summary>
    internal static double[] SimpleBoxcarDecimation(ReadOnlySpan<double> spectrum, int decimationFactor, bool padToFullLength = true)
    {
        if (decimationFactor <= 1) return spectrum.ToArray();
        int n = spectrum.Length;
        int outCount = n / decimationFactor;
        var output = new double[padToFullLength ? n : outCount];
        double inv = 1.0 / decimationFactor;
        int src = 0;
        for (int j = 0; j < outCount; j++)
        {
            double sum = 0.0;
            for (int k = 0; k < decimationFactor; k++) sum += spectrum[src++];
            output[j] = sum * inv;
        }
        return output;
    }

    /// <summary>
    /// 拟合 5 阶多项式（对称横轴归一化到 [-1, 1] 保证数值条件数稳定），用于 APx500 <c>OPPE837xSG</c> 串扰切片基底扣除。
    /// </summary>
    internal static double[] FitPolynomial5(ReadOnlySpan<double> values)
    {
        int n = values.Length;
        if (n <= 6) return values.ToArray();
        const int m = 6;
        var ata = new double[m, m];
        var aty = new double[m];
        var xp = new double[m];
        double invHalf = 2.0 / (n - 1);
        for (int i = 0; i < n; i++)
        {
            double x = i * invHalf - 1.0;
            xp[0] = 1.0;
            for (int p = 1; p < m; p++) xp[p] = xp[p - 1] * x;
            double y = values[i];
            for (int r = 0; r < m; r++)
            {
                aty[r] += xp[r] * y;
                for (int c = r; c < m; c++) ata[r, c] += xp[r] * xp[c];
            }
        }
        for (int r = 1; r < m; r++)
            for (int c = 0; c < r; c++) ata[r, c] = ata[c, r];

        // 部分主元高斯消元求解 6×6 正规方程
        for (int k = 0; k < m; k++)
        {
            int pivot = k;
            double maxAbs = Math.Abs(ata[k, k]);
            for (int r = k + 1; r < m; r++)
            {
                double a = Math.Abs(ata[r, k]);
                if (a > maxAbs) { maxAbs = a; pivot = r; }
            }
            if (maxAbs < 1e-18) continue;
            if (pivot != k)
            {
                for (int c = k; c < m; c++) (ata[k, c], ata[pivot, c]) = (ata[pivot, c], ata[k, c]);
                (aty[k], aty[pivot]) = (aty[pivot], aty[k]);
            }
            double invDiag = 1.0 / ata[k, k];
            for (int r = k + 1; r < m; r++)
            {
                double factor = ata[r, k] * invDiag;
                for (int c = k + 1; c < m; c++) ata[r, c] -= factor * ata[k, c];
                aty[r] -= factor * aty[k];
            }
        }
        var coef = new double[m];
        for (int r = m - 1; r >= 0; r--)
        {
            double sum = aty[r];
            for (int c = r + 1; c < m; c++) sum -= ata[r, c] * coef[c];
            coef[r] = Math.Abs(ata[r, r]) > 1e-18 ? sum / ata[r, r] : 0.0;
        }

        var fitted = new double[n];
        for (int i = 0; i < n; i++)
        {
            double x = i * invHalf - 1.0;
            double val = coef[m - 1];
            for (int p = m - 2; p >= 0; p--) val = val * x + coef[p];
            fitted[i] = val;
        }
        return fitted;
    }

    public static (BenchTrace? Channel1, BenchTrace? Channel2) PhaseTraces(ContinuousPhaseCapture measured,
        ContinuousPhaseMode mode, double? fitMinHz = null, double? fitMaxHz = null)
    {
        if (mode == ContinuousPhaseMode.RelativeToCh1)
        {
            var first = measured.Channel1;
            var second = measured.Channel2;
            // 对齐 APx500 Difference 相位：Ch1 始终发布 0° 参考曲线（单通道也不空白），Ch2 发布 Ch2 - Ch1。
            if (first == null) return (null, null);
            var ch1Zero = new BenchTrace(first.X, new double[first.X.Length]);
            if (second == null) return (ch1Zero, null);
            if (first.X.AsSpan().SequenceEqual(second.X))
            {
                var values = new double[first.X.Length];
                for (int i = 0; i < values.Length; i++) values[i] = second.Y[i] - first.Y[i];
                return (ch1Zero, new(second.X, values));
            }
            // Staggered channels can cross different FFT padding boundaries. Use only the
            // shared measured bins when subtracting their unwrapped phases.
            var commonHz = new List<double>(); var difference = new List<double>();
            for (int a = 0, b = 0; a < first.X.Length && b < second.X.Length;)
            {
                if (first.X[a] < second.X[b]) a++;
                else if (first.X[a] > second.X[b]) b++;
                else { commonHz.Add(first.X[a]); difference.Add(second.Y[b++] - first.Y[a++]); }
            }
            return commonHz.Count == 0 ? (ch1Zero, null) : (ch1Zero, new(commonHz.ToArray(), difference.ToArray()));
        }

        BenchTrace? Display(BenchTrace? trace)
        {
            // 原始展开相位可以保留整圈。需要折叠就归约到单圈，需要扣除线性相位趋势则以有效点拟合斜率和截距。
            if (trace == null || mode == ContinuousPhaseMode.Unwrapped) return trace;
            if (mode == ContinuousPhaseMode.Wrapped)
                return new(trace.X, Array.ConvertAll(trace.Y, FrequencyResponseSweeper.NormalizeAngleDeg));
            // Fit absolute phase against frequency (Hz), independent of the displayed logarithmic axis.
            var fit = Enumerable.Range(0, trace.X.Length).Where(i => double.IsFinite(trace.Y[i]) &&
                trace.X[i] >= (fitMinHz ?? 0) && trace.X[i] <= (fitMaxHz ?? double.PositiveInfinity)).ToArray();
            if (fit.Length < 2) return null;
            double meanX = fit.Average(i => trace.X[i]), meanY = fit.Average(i => trace.Y[i]);
            double variance = fit.Sum(i => Math.Pow(trace.X[i] - meanX, 2));
            if (variance == 0) return null;
            // 在真实 Hz 轴上对相位角作最小二乘拟合，斜率用度/Hz 表示。纯传播延时产生直线，相减后可观察其余频率依赖。
            double slope = fit.Sum(i => (trace.X[i] - meanX) * (trace.Y[i] - meanY)) / variance;
            return new(trace.X, trace.Y.Select((y, i) => y - (meanY + slope * (trace.X[i] - meanX))).ToArray());
        }
        return (Display(measured.Channel1), Display(measured.Channel2));
    }

    public static BenchTrace CrossCorrelation(double[] reference, double[] recording, int rate)
    {
        int length = checked(reference.Length + recording.Length - 1);
        if (reference.Length == 0 || recording.Length == 0 || rate < 1) throw new ArgumentException("Missing sweep data.");
        int n = checked((int)BitOperations.RoundUpToPowerOf2((uint)length));
        var x = new Complex[n];
        var y = new Complex[n];
        // 延迟估计观察两路随平移量的相似程度。频域的 Y·conj(X) 逆变换给出所有平移位置的相关，再用两路能量的几何平均归一化。
        double xx = 0, yy = 0;
        for (int i = 0; i < reference.Length; i++) { x[i] = reference[i]; xx += reference[i] * reference[i]; }
        for (int i = 0; i < recording.Length; i++) { y[i] = recording[i]; yy += recording[i] * recording[i]; }
        FftEngine.ForwardInPlace(x);
        FftEngine.ForwardInPlace(y);
        for (int i = 0; i < n; i++) y[i] *= Complex.Conjugate(x[i]);
        FftEngine.InverseInPlace(y);
        double norm = Math.Sqrt(xx * yy);
        double[] time = new double[length], coefficient = new double[length];
        for (int i = 0; i < length; i++)
        {
            int lag = i - reference.Length + 1;
            time[i] = 1000.0 * lag / rate;
            coefficient[i] = norm > 0 ? Math.Clamp(y[(lag + n) % n].Real / norm, -1, 1) : double.NaN;
        }
        return new(time, coefficient);
    }

    // -d(arg H)/dω = Re{FFT(n*h)/FFT(h)} in samples. Include the physical time
    // origin of the stored IR, even when the graph phase was aligned to its peak.
    public static BenchTrace GroupDelay(double[] impulse, double originMs, int rate, double[] frequencies, double[]? gate = null, Complex[]? responseSpectrum = null)
    {
        int n = responseSpectrum?.Length ?? (int)BitOperations.RoundUpToPowerOf2((uint)Math.Max(4096, impulse.Length * 2));
        var h = responseSpectrum ?? new Complex[n];
        var moment = new Complex[n];
        // 群延迟是 −dφ/dω。傅里叶变换中对频率求导等价于时域样本乘时间，因此同时变换 h[n] 和 n·h[n] 即可计算，最后加回时间轴原点。
        for (int i = 0; i < impulse.Length; i++)
        {
            double sample = impulse[i] * (gate is null ? 1 : gate[i]);
            if (responseSpectrum == null) h[i] = sample;
            moment[i] = sample * i;
        }
        if (responseSpectrum == null) SequenceTransform.Forward(h);
        SequenceTransform.Forward(moment);
        var delay = new double[frequencies.Length];
        for (int i = 0; i < delay.Length; i++)
        {
            // 将 Hz 换到最近频格。分母 H 几乎为零时相位导数没有可靠估计；其余点取复比值实部，由样本换成毫秒。
            int bin = (int)Math.Round(frequencies[i] * n / rate);
            delay[i] = bin is > 0 && bin < n / 2 && h[bin].Magnitude > 1e-8
                ? originMs + 1000.0 / rate * (moment[bin] / h[bin]).Real : double.NaN;
        }
        return new(frequencies, delay);
    }

    internal readonly record struct ContinuousHarmonicResults(
        IReadOnlyList<FrequencyResponsePoint>? H2,
        IReadOnlyList<FrequencyResponsePoint>? H3,
        IReadOnlyList<FrequencyResponsePoint>? Thd,
        Dictionary<int, BenchTrace> ProductLevels);

    /// <summary>
    /// 对齐 APx500 <c>ChirpParams.GetHarmonic</c> → <c>GetHarmRespCore</c>（<c>FftWithAsymWin</c> + <c>BoxcarDecimator</c>）
    /// → <c>ReprocessImpulseResults.ReprocessThd</c> → <c>Distortion.ComputeDistortion</c>（默认 IEC 比值，合并 H2..MaxHarmNum）。
    /// 一次性输出 H2、H3、THD（Ratio/Level）与 H2..H20 各阶产物电平，消除多处重复提取的分歧。
    /// </summary>
    internal static ContinuousHarmonicResults ExtractContinuousHarmonics(
        double[] centeredIr,
        int peakInFullIr,
        int acqLen,
        Complex[] linearSpec,
        int linFftLen,
        int sampleRate,
        ContinuousSweepConfig config,
        IReadOnlyList<FrequencyResponsePoint> linearPoints,
        Func<double, double> excitationLevelAtHz,
        bool useIecRatio = true)
    {
        double f1 = Math.Clamp(config.StartFrequencyHz, 5.0, sampleRate * 0.40);
        double f2 = Math.Max(f1 + 10.0, ContinuousSweepAnalyzer.CheckStopFrequency(config.StopFrequencyHz, sampleRate));
        double fEnd = config.ChirpType == ContinuousChirpType.ApChirp
            ? Math.Min(sampleRate * 0.49, f2 * 1.05)
            : ContinuousSweepAnalyzer.LogChirpEndFrequency(f1, f2, sampleRate);
        double duration = Math.Clamp(config.DurationSeconds, 0.1, 15.0);
        int fftSize = centeredIr.Length;

        if (fEnd / f1 < 2.2 || linearPoints.Count == 0)
            return new(null, null, null, new Dictionary<int, BenchTrace>());

        // 对齐 APx500 ChirpParams ctor L28283-28284 与 ffLE9lvl5G L28659
        int maxHarmNum = Math.Clamp(
            (int)Math.Floor(Math.Pow(fEnd / f1, (double)acqLen / (2.0 * duration * sampleRate)) - 0.25),
            2, 20);
        double samplesPerNeper = Math.Floor(duration * sampleRate) / Math.Log(fEnd / f1);
        int harmFftLen = SequenceTransform.NextLength(Math.Max(acqLen >> 3, 1024), allowShort: true);
        int harmHalfBins = harmFftLen / 2 + 1;

        var decimatedByOrder = new Dictionary<int, double[]>(20);

        for (int order = 2; order <= 20; order++)
        {
            int leftOffset = (int)Math.Round(samplesPerNeper * Math.Log(order + 0.25), MidpointRounding.AwayFromZero);
            int centerOffset = (int)Math.Round(samplesPerNeper * Math.Log(order), MidpointRounding.AwayFromZero);
            int rightOffset = (int)Math.Round(samplesPerNeper * Math.Log(order - 0.25), MidpointRounding.AwayFromZero);
            int startIdx = peakInFullIr - leftOffset;
            int endIdx = peakInFullIr - rightOffset;
            int sliceLen = endIdx - startIdx + 1;
            int peakInSlice = leftOffset - centerOffset;
            bool inBounds = startIdx >= 0 && endIdx < fftSize;
            // 对于短尾响采集（如测试中仅补少量尾样本），高于 maxHarmNum 的高阶谐波在循环卷积中绕回尾部远端，仍可安全提取单阶产物；
            // THD 汇总严格按 APx500 MaxHarmNum 上限执行。
            if (!inBounds && (order <= maxHarmNum || leftOffset >= fftSize - 64))
                continue;
            if (sliceLen < 4 || peakInSlice < 0 || peakInSlice >= sliceLen)
                continue;

            var slice = new double[sliceLen];
            if (inBounds)
                Array.Copy(centeredIr, startIdx, slice, 0, sliceLen);
            else
                for (int i = 0; i < sliceLen; i++)
                    slice[i] = centeredIr[((startIdx + i) % fftSize + fftSize) % fftSize];
            SubtractLinearTrend(slice);

            // 对齐 APx500 FftWithAsymWin.KvSOGjdaHk：超出 harmFftLen 时围绕峰值裁切，不足时补零至 harmFftLen
            int sliceStart = sliceLen > harmFftLen
                ? Math.Clamp(peakInSlice - harmFftLen / 2, 0, sliceLen - harmFftLen)
                : 0;
            int used = Math.Min(sliceLen, harmFftLen);
            var asymWin = SequenceFrequencyResponse.CreateImpulseWindow(used, peakInSlice - sliceStart, 0.90, 0.95);
            var buf = new Complex[harmFftLen];
            for (int i = 0; i < used; i++)
                buf[i] = new Complex(slice[sliceStart + i] * asymWin[i], 0.0);
            SequenceTransform.Forward(buf);

            var rawMag = new double[harmHalfBins];
            for (int k = 0; k < harmHalfBins; k++) rawMag[k] = buf[k].Magnitude;

            decimatedByOrder[order] = SimpleBoxcarDecimation(rawMag, order, padToFullLength: true);
        }

        if (decimatedByOrder.Count == 0)
            return new(null, null, null, new Dictionary<int, BenchTrace>());

        double harmDf = (double)sampleRate / harmFftLen;
        double linDf = (double)sampleRate / linFftLen;
        int linMaxBin = linFftLen / 2 - 1;
        double nyquist = sampleRate * 0.5;
        bool useHarmonicAxis = config.OutputPoints == 0;

        double SampleDecimated(double[] dec, int order, double freqHz, int exactBin)
        {
            int maxValidBin = (harmFftLen / 2 - 1) / order;
            if (maxValidBin < 1) return double.NaN;
            if (exactBin >= 1)
                return exactBin <= maxValidBin ? dec[exactBin] : double.NaN;
            double pos = freqHz / harmDf;
            if (pos < 0.5 || pos > maxValidBin + 0.5) return double.NaN;
            int lo = Math.Clamp((int)Math.Floor(pos), 1, maxValidBin);
            int hi = Math.Min(lo + 1, maxValidBin);
            double frac = Math.Clamp(pos - lo, 0.0, 1.0);
            return dec[lo] + frac * (dec[hi] - dec[lo]);
        }

        double FundamentalMagAt(double freqHz)
        {
            double pos = Math.Clamp(freqHz / linDf, 1.0, linMaxBin);
            int lo = (int)pos;
            double a = linearSpec[lo].Magnitude;
            if (pos != lo && lo + 1 <= linMaxBin)
                a += (pos - lo) * (linearSpec[lo + 1].Magnitude - a);
            return double.IsFinite(a) && a > 1e-12 ? a : 1e-12;
        }

        // 对齐 APx500 HarmonicAxis (LXAOjjqXmJk)：全分辨率模式 (OutputPoints == 0) 使用 harmFftLen 精确整数频格
        // [floor(F1/harmDf) .. floor((Fs/4 - eps)/harmDf)]，各阶截断于 order * f < Fs/2（而非 f2/2）；
        // 稀疏输出模式 (OutputPoints > 0) 保持与请求频率网格兼容。
        double[] frequencies;
        int[] exactBins;
        bool[] pointValid;
        if (useHarmonicAxis)
        {
            int startBin = Math.Max(1, (int)Math.Floor(f1 / harmDf));
            int endBin = Math.Min((harmFftLen / 2 - 1) / 2, (int)Math.Floor(f2 / harmDf));
            if (endBin < startBin)
                return new(null, null, null, new Dictionary<int, BenchTrace>());
            int count = endBin - startBin + 1;
            frequencies = new double[count];
            exactBins = new int[count];
            pointValid = new bool[count];
            for (int i = 0; i < count; i++)
            {
                int bin = startBin + i;
                exactBins[i] = bin;
                frequencies[i] = bin * harmDf;
                pointValid[i] = true;
            }
        }
        else
        {
            frequencies = new double[linearPoints.Count];
            exactBins = new int[linearPoints.Count];
            pointValid = new bool[linearPoints.Count];
            for (int i = 0; i < linearPoints.Count; i++)
            {
                frequencies[i] = linearPoints[i].FrequencyHz;
                exactBins[i] = -1;
                pointValid[i] = linearPoints[i].IsValid;
            }
        }

        var productLevels = new Dictionary<int, BenchTrace>(decimatedByOrder.Count);
        var productArrays = new Dictionary<int, double[]>(decimatedByOrder.Count);
        foreach (int order in decimatedByOrder.Keys)
        {
            var arr = new double[frequencies.Length];
            Array.Fill(arr, double.NaN);
            productArrays[order] = arr;
        }

        var h2List = new List<FrequencyResponsePoint>(frequencies.Length);
        var h3List = new List<FrequencyResponsePoint>(frequencies.Length);
        var thdList = new List<FrequencyResponsePoint>(frequencies.Length);

        for (int i = 0; i < frequencies.Length; i++)
        {
            double f = frequencies[i];
            int exactBin = exactBins[i];
            bool validBin = pointValid[i];
            double fundMag = FundamentalMagAt(f);
            double fundExcitationDbfs = excitationLevelAtHz(Math.Clamp(f, f1, f2));
            double fundLinearFs = fundMag * SignalGeneratorDsp.DbfsToLinearPeak(fundExcitationDbfs);

            double sumSqFs = 0.0;
            bool anyValid = false;
            double h2RatioDb = double.NaN, h2LevelDbfs = double.NaN;
            bool h2Valid = false;
            double h3RatioDb = double.NaN, h3LevelDbfs = double.NaN;
            bool h3Valid = false;

            foreach (var (order, dec) in decimatedByOrder)
            {
                double targetHz = order * f;
                if (!validBin || targetHz >= nyquist || (!useHarmonicAxis && targetHz > f2))
                    continue;

                double relMag = SampleDecimated(dec, order, f, exactBin);
                if (!double.IsFinite(relMag)) continue;

                // 分离出的第 k 次谐波脉冲给出 Y_k(k·f)/X(k·f)；乘回激励电平即得绝对电平（线性 FS）
                double harmExcitationDbfs = excitationLevelAtHz(Math.Clamp(targetHz, f1, f2));
                double harmLinearFs = relMag * SignalGeneratorDsp.DbfsToLinearPeak(harmExcitationDbfs);
                double harmLevelDbfs = 20.0 * Math.Log10(Math.Max(harmLinearFs, 1e-15));
                productArrays[order][i] = harmLevelDbfs;

                if (order <= maxHarmNum)
                {
                    sumSqFs += harmLinearFs * harmLinearFs;
                    anyValid = true;
                }

                if (order == 2)
                {
                    h2Valid = true;
                    h2LevelDbfs = harmLevelDbfs;
                    h2RatioDb = 20.0 * Math.Log10(Math.Max(harmLinearFs / Math.Max(fundLinearFs, 1e-15), 1e-15));
                }
                else if (order == 3)
                {
                    h3Valid = true;
                    h3LevelDbfs = harmLevelDbfs;
                    h3RatioDb = 20.0 * Math.Log10(Math.Max(harmLinearFs / Math.Max(fundLinearFs, 1e-15), 1e-15));
                }
            }

            double thdLevelDbfs = anyValid ? 20.0 * Math.Log10(Math.Max(Math.Sqrt(sumSqFs), 1e-15)) : double.NaN;
            double thdRatio = double.NaN;
            if (anyValid && fundLinearFs > 1e-15)
            {
                double denomSq = useIecRatio ? sumSqFs + fundLinearFs * fundLinearFs : fundLinearFs * fundLinearFs;
                thdRatio = Math.Sqrt(sumSqFs / denomSq);
            }
            double thdDb = anyValid && double.IsFinite(thdRatio) ? 20.0 * Math.Log10(Math.Max(thdRatio, 1e-15)) : double.NaN;
            double thdPct = anyValid && double.IsFinite(thdRatio) ? thdRatio * 100.0 : double.NaN;

            h2List.Add(new FrequencyResponsePoint(f, h2LevelDbfs, h2LevelDbfs, h2RatioDb, h2RatioDb, h2RatioDb, h2RatioDb, null, null, null, XValue: f, IsValid: h2Valid));
            h3List.Add(new FrequencyResponsePoint(f, h3LevelDbfs, h3LevelDbfs, h3RatioDb, h3RatioDb, h3RatioDb, h3RatioDb, null, null, null, XValue: f, IsValid: h3Valid));
            thdList.Add(new FrequencyResponsePoint(f, thdLevelDbfs, thdLevelDbfs, thdDb, thdDb, thdDb, thdDb, null, null, null, XValue: f, Ch1ThdNPercent: thdPct, Ch1ThdNDb: thdDb, IsValid: anyValid));
        }

        foreach (var (order, arr) in productArrays)
        {
            if (arr.Any(double.IsFinite))
                productLevels[order] = new BenchTrace(frequencies, arr);
        }

        bool hasH2 = decimatedByOrder.ContainsKey(2);
        bool hasH3 = decimatedByOrder.ContainsKey(3);
        return new(hasH2 ? h2List : null, hasH3 ? h3List : null, anyHasValid(thdList) ? thdList : null, productLevels);

        static bool anyHasValid(List<FrequencyResponsePoint> list)
        {
            for (int i = 0; i < list.Count; i++) if (list[i].IsValid) return true;
            return false;
        }
    }

    // 对齐 APx500 ChirpParams.GetHarmonic 的 1 阶多项式去趋势（消除谐波切片内的直流底座与线性斜率）。
    internal static void SubtractLinearTrend(double[] slice)
    {
        int n = slice.Length;
        if (n < 2) return;
        double mid = 0.5 * (n - 1);
        double sumY = 0, sumXY = 0, sumXX = 0;
        for (int i = 0; i < n; i++)
        {
            double x = i - mid;
            sumY += slice[i];
            sumXY += x * slice[i];
            sumXX += x * x;
        }
        double mean = sumY / n;
        double slope = sumXX > 0 ? sumXY / sumXX : 0;
        for (int i = 0; i < n; i++)
            slice[i] -= mean + slope * (i - mid);
    }
}
