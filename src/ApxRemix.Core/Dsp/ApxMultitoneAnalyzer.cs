using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed record MultitoneAnalysisResult(
    double FreqErrorRatio,
    double[][] ToneFreqs,
    double[][] PhaseFreqs,
    double[][] Levels,
    double[][] Spectrum,
    double[][] NoiseDensity,
    double[][] TdPlusNoiseSpectrum,
    double[] NoiseRms,
    double[] TdPlusNLevel,
    double[] TdPlusNRatio,
    double[] DcLevel,
    double[] TotalRms,
    double[][] TonePhasesDeg);

/// <summary>
/// APx500 10.0.0.17 多音信号发生与分析引擎独立复刻（对齐 <c>MultitoneGenerationLibrary</c> 与 <c>MultitoneLibrary</c>）。
/// 支持 Newman / Schroeder / Zygmund / Rudin-Shapiro 低波峰因数相位优化、互调避让、频率偏差检测与多相重采样、
/// 逐音幅相响应、TD+N 谱与总比值、噪声 RMS 及噪声功率谱密度。
/// </summary>
public static class ApxMultitoneAnalyzer
{
    public static double CanonicalizePhaseDeg(double phaseDeg)
    {
        double turns = Math.Round(phaseDeg / 360.0);
        return phaseDeg - turns * 360.0;
    }

    /// <summary>对齐 <c>MultitoneGenerationLibrary.ComputeNewmanPhases</c>。</summary>
    public static double[] ComputeNewmanPhases(int count)
    {
        var phases = new double[count];
        for (int i = 0; i < count; i++)
            phases[i] = CanonicalizePhaseDeg(180.0 * i * i / count + 90.0);
        return phases;
    }

    /// <summary>对齐 <c>MultitoneGenerationLibrary.ComputeSchroederPhases</c>。</summary>
    public static double[] ComputeSchroederPhases(double phaseZeroDeg, ReadOnlySpan<double> amplitudes)
    {
        int n = amplitudes.Length;
        var phases = new double[n];
        if (n == 0) return phases;
        double sumSq = 0.0;
        for (int i = 0; i < n; i++) sumSq += amplitudes[i] * amplitudes[i];
        phases[0] = phaseZeroDeg;
        for (int j = 1; j < n; j++)
        {
            double acc = 0.0;
            for (int k = 0; k < j; k++)
                acc += (j - k) * amplitudes[j] * amplitudes[j] / sumSq;
            phases[j] = CanonicalizePhaseDeg((phaseZeroDeg - Math.Tau * acc) * 180.0 / Math.PI + 90.0);
        }
        return phases;
    }

    /// <summary>对齐 <c>MultitoneGenerationLibrary.ComputeZygmundPhases</c>。</summary>
    public static double[] ComputeZygmundPhases(int count)
    {
        var phases = new double[count];
        for (int i = 1; i <= count; i++)
            phases[i - 1] = CanonicalizePhaseDeg(10.0 * i * Math.Log(i) * 180.0 / Math.PI + 90.0);
        return phases;
    }

    /// <summary>对齐 <c>MultitoneGenerationLibrary.ComputeRudin_ShapiroPhases</c>。</summary>
    public static double[] ComputeRudinShapiroPhases(int count)
    {
        var phases = new double[count];
        for (int i = 0; i < count; i++)
            phases[i] = CanonicalizePhaseDeg(180.0 * RudinShapiroBit(i) + 90.0);
        return phases;
    }

    private static int RudinShapiroBit(int n)
    {
        int sign = 1;
        while (n > 0)
        {
            int low = n & 1;
            n >>= 1;
            if (n == 0) break;
            if (low == 1 && (n & 1) == 1) sign = -sign;
        }
        return sign != 1 ? 1 : 0;
    }

    /// <summary>对齐 <c>MultitoneGenerationLibrary.QuantizeFrequencies</c>。</summary>
    public static void QuantizeFrequencies(double[] toneFrequencies, double sampleRate, double maxAllowedHz, int generatorLength)
    {
        double df = sampleRate / generatorLength;
        int maxBin = (int)Math.Floor(maxAllowedHz / df);
        for (int i = 0; i < toneFrequencies.Length; i++)
        {
            if (double.IsNaN(toneFrequencies[i])) continue;
            int bin = Math.Clamp((int)Math.Round(toneFrequencies[i] / df), 1, Math.Max(1, maxBin));
            toneFrequencies[i] = bin * df;
        }
    }

    /// <summary>对齐 <c>MultitoneGenerationLibrary.GenerateMultitone</c>。</summary>
    public static double[] GenerateMultitone(ReadOnlySpan<double> toneFrequencies, ReadOnlySpan<double> phasesDeg,
        ReadOnlySpan<double> amplitudes, double sampleRate, int generatorLength)
    {
        var wave = new double[generatorLength];
        for (int i = 0; i < toneFrequencies.Length; i++)
        {
            double f = toneFrequencies[i];
            if (f >= sampleRate * 0.5) continue;
            double phase = Math.PI * phasesDeg[i] / 180.0;
            double step = Math.Tau * f / sampleRate;
            double amp = amplitudes[i];
            for (int n = 0; n < generatorLength; n++)
            {
                wave[n] += amp * Math.Sin(phase);
                phase += step;
            }
        }
        return wave;
    }

    public static double ComputeCrestFactor(ReadOnlySpan<double> waveform)
    {
        if (waveform.IsEmpty) return double.NaN;
        double peak = 0.0, sumSq = 0.0;
        for (int i = 0; i < waveform.Length; i++)
        {
            double a = Math.Abs(waveform[i]);
            if (a > peak) peak = a;
            sumSq += waveform[i] * waveform[i];
        }
        double rms = Math.Sqrt(sumSq / waveform.Length);
        return rms > 0 ? peak / rms : double.NaN;
    }

    /// <summary>对齐 <c>MultitoneGenerationLibrary.ComputeHarmonics</c> 与 <c>ComputeIMDHarmonics</c>。</summary>
    public static int[] ComputeHarmonicBins(int bin, int generatorLength)
    {
        var list = new List<int>();
        for (int k = 2 * bin; k < generatorLength; k += bin) list.Add(k);
        return list.ToArray();
    }

    public static int[] ComputeImdHarmonicBins(int binOne, int binTwo, int generatorLength)
    {
        var list = new List<int>();
        int diff = binTwo - binOne;
        if (diff <= 0) return [];
        for (int m = 1; binOne - m * diff >= 0; m++) list.Add(binOne - m * diff);
        for (int m = 1; binTwo + m * diff < generatorLength; m++) list.Add(binTwo + m * diff);
        return list.ToArray();
    }

    /// <summary>
    /// 对齐 <c>MultitoneLibrary.ResampleAndAveragePowerSpectrum</c> + <c>AnalyzeOneChannel</c>：
    /// 给定多声道采集波形（每声道长度为 <c>2 * generatorLength * numAverages</c> 或更长）、音表与采样率，
    /// 输出各单音电平/相位、总 RMS、DC、TD+N 谱与总比值、噪声 RMS 与噪声密度。
    /// </summary>
    public static MultitoneAnalysisResult Analyze(
        double[][] acquisitions,
        double[] toneFrequencies,
        double[]? tonePhasesDeg,
        double sampleRate,
        int generatorLength,
        int numAverages = 1,
        double toneBinPercentHalfWidth = 0.0,
        bool isDigital = true,
        bool enableFrequencyCorrection = false,
        AnalyzerWeighting weighting = AnalyzerWeighting.Flat)
    {
        int chCount = acquisitions.Length;
        double df = sampleRate / generatorLength;
        var toneBins = new int[toneFrequencies.Length];
        for (int i = 0; i < toneFrequencies.Length; i++)
            toneBins[i] = (int)Math.Round(toneFrequencies[i] / df);
        var phases = tonePhasesDeg ?? new double[toneFrequencies.Length];

        int fftLen = 2 * generatorLength;
        int totalNeeded = fftLen * Math.Max(1, numAverages);

        double freqRatio = 1.0;
        var work = new double[chCount][];
        if (enableFrequencyCorrection && acquisitions[0].Length >= totalNeeded + ApxMoveToBinCenter.SrcFirLen)
        {
            freqRatio = EstimateMultitoneFreqRatio(acquisitions[0], toneFrequencies, sampleRate);
            if (Math.Abs(freqRatio - 1.0) <= 1e-8) freqRatio = 1.0;
            for (int c = 0; c < chCount; c++)
                work[c] = freqRatio == 1.0
                    ? acquisitions[c][..totalNeeded]
                    : ApxMoveToBinCenter.Resample(acquisitions[c], freqRatio, totalNeeded);
        }
        else
        {
            for (int c = 0; c < chCount; c++)
            {
                int len = Math.Min(acquisitions[c].Length, totalNeeded);
                work[c] = new double[totalNeeded];
                Array.Copy(acquisitions[c], work[c], len);
            }
        }

        // APx MultitoneLibrary.F3fnl1tkEF: 无窗 FFT（长度 = 2 * generatorLength，使测试音落在偶数频格 2*toneBin，奇数频格为纯噪声）。
        int halfBins = fftLen / 2 + 1;
        var avgPower = new double[chCount][];
        var avgComplex = new Complex[chCount][];
        var rectWin = SequenceFftWindow.Get(fftLen, FftWindowType.Rectangular);
        for (int c = 0; c < chCount; c++)
        {
            avgPower[c] = new double[halfBins];
            avgComplex[c] = new Complex[halfBins];
        }

        for (int avg = 0; avg < numAverages; avg++)
        {
            for (int c = 0; c < chCount; c++)
            {
                var segment = work[c].AsSpan(avg * fftLen, fftLen);
                var fft = rectWin.Transform(segment);
                var pow = SequenceFftWindow.Power(fft, fftLen, halfBins);
                for (int k = 0; k < halfBins; k++)
                {
                    avgPower[c][k] += pow[k];
                    avgComplex[c][k] += fft[k];
                }
            }
        }

        double invAvg = 1.0 / numAverages;
        double powScale = invAvg * (isDigital ? 2.0 : 1.0);
        for (int c = 0; c < chCount; c++)
        {
            for (int k = 0; k < halfBins; k++)
            {
                avgPower[c][k] *= powScale;
                avgComplex[c][k] *= invAvg;
            }
            if (isDigital) avgPower[c][0] *= 0.5;
        }

        var levels = new double[chCount][];
        var spectrum = new double[chCount][];
        var noiseDensity = new double[chCount][];
        var tdPlusNoiseSpec = new double[chCount][];
        var noiseRms = new double[chCount];
        var tdPlusNLevel = new double[chCount];
        var tdPlusNRatio = new double[chCount];
        var dcLevel = new double[chCount];
        var totalRms = new double[chCount];
        var measuredPhases = new double[chCount][];
        var toneFreqsPerCh = new double[chCount][];
        var phaseFreqsPerCh = new double[chCount][];

        double[]? weightGain = null;
        if (weighting != AnalyzerWeighting.Flat)
        {
            weightGain = new double[halfBins];
            double binStep = sampleRate / fftLen;
            for (int k = 0; k < halfBins; k++)
                weightGain[k] = AudioMetricsAnalyzer.ComputeWeightingPowerGain(k * binStep, weighting);
        }

        for (int c = 0; c < chCount; c++)
        {
            var p = avgPower[c];
            var z = avgComplex[c];
            toneFreqsPerCh[c] = (double[])toneFrequencies.Clone();
            phaseFreqsPerCh[c] = (double[])toneFrequencies.Clone();

            // 1. Tone levels
            levels[c] = new double[toneBins.Length];
            for (int i = 0; i < toneBins.Length; i++)
            {
                int prev = i > 0 ? 2 * toneBins[i - 1] : -1;
                int next = i < toneBins.Length - 1 ? 2 * toneBins[i + 1] : -1;
                ComputeToneBinBounds(2 * toneBins[i], prev, next, halfBins - 1, toneBinPercentHalfWidth, out int lo, out int hi);
                levels[c][i] = Math.Sqrt(BandPower(p, lo, hi));
            }

            // 2. Tone phases
            measuredPhases[c] = new double[toneBins.Length];
            for (int i = 0; i < toneBins.Length; i++)
            {
                int bin = toneBins[i] << 1;
                double rawDeg = bin < z.Length ? Math.Atan2(z[bin].Imaginary, z[bin].Real) * 180.0 / Math.PI : double.NaN;
                measuredPhases[c][i] = CanonicalizePhaseDeg(rawDeg - phases[i]);
            }

            // 3. Full amplitude spectrum
            spectrum[c] = new double[halfBins];
            for (int k = 0; k < halfBins; k++) spectrum[c][k] = Math.Sqrt(Math.Max(0.0, p[k]));

            // 4. DC & Total AC RMS
            dcLevel[c] = Math.Sqrt(Math.Max(0.0, p[0])) * Math.Sign(z[0].Real);
            double acPow = 0.0;
            for (int k = 1; k < halfBins; k++) acPow += p[k];
            totalRms[c] = Math.Sqrt(Math.Max(0.0, acPow));

            // 5. Noise RMS & Noise Density (odd bins only, multiplied by 2 to restore even+odd noise power)
            var isToneBin = BuildToneExclusionMask(toneBins, halfBins, toneBinPercentHalfWidth);
            double oddNoisePow = 0.0;
            noiseDensity[c] = new double[halfBins / 2];
            double densityScale = (double)((halfBins - 1) * 4) / sampleRate;
            for (int k = 1; k < halfBins; k += 2)
            {
                double wp = weightGain is null ? p[k] : p[k] * weightGain[k];
                if (!isToneBin[k])
                {
                    oddNoisePow += wp;
                    noiseDensity[c][k / 2] = Math.Sqrt(Math.Max(0.0, p[k] * densityScale));
                }
                else
                {
                    noiseDensity[c][k / 2] = double.NaN;
                }
            }
            noiseRms[c] = Math.Sqrt(Math.Max(0.0, oddNoisePow * 2.0));

            // 6. TD+N Spectrum, Level & Ratio (all non-tone bins k >= 1)
            tdPlusNoiseSpec[c] = new double[halfBins];
            Array.Fill(tdPlusNoiseSpec[c], double.NaN);
            double tdPow = 0.0;
            for (int k = 1; k < halfBins; k++)
            {
                if (!isToneBin[k])
                {
                    tdPlusNoiseSpec[c][k] = Math.Sqrt(Math.Max(0.0, p[k]));
                    tdPow += weightGain is null ? p[k] : p[k] * weightGain[k];
                }
            }
            double toneRss = 0.0;
            for (int i = 0; i < levels[c].Length; i++) toneRss += levels[c][i] * levels[c][i];
            toneRss = Math.Sqrt(toneRss);
            tdPlusNLevel[c] = Math.Sqrt(Math.Max(0.0, tdPow));
            tdPlusNRatio[c] = toneRss > 0.0 ? tdPlusNLevel[c] / toneRss : double.NaN;
        }

        return new MultitoneAnalysisResult(
            freqRatio, toneFreqsPerCh, phaseFreqsPerCh, levels, spectrum, noiseDensity,
            tdPlusNoiseSpec, noiseRms, tdPlusNLevel, tdPlusNRatio, dcLevel, totalRms, measuredPhases);
    }

    private static double EstimateMultitoneFreqRatio(double[] acq, double[] toneFreqs, double sampleRate)
    {
        int n = acq.Length & ~1;
        if (n < 64) return 1.0;
        var win = SequenceFftWindow.Get(n, FftWindowType.BlackmanHarris);
        var pow = SequenceFftWindow.Power(win.Transform(acq.AsSpan(0, n)), n, n / 2 + 1);
        double df = sampleRate / n;
        var ratios = new List<double>();
        var powers = new List<double>();
        foreach (double f in toneFreqs)
        {
            double expectedBin = f / df;
            int halfSearch = Math.Max(2, (int)Math.Round(0.03 * expectedBin));
            int center = (int)Math.Round(expectedBin);
            int lo = Math.Max(1, center - halfSearch);
            int hi = Math.Min(pow.Length - 2, center + halfSearch);
            if (lo >= hi) continue;
            int peak = lo;
            double maxP = 0.0;
            for (int k = lo; k <= hi; k++)
                if (pow[k] > maxP) { maxP = pow[k]; peak = k; }
            double pwr = BandPower(pow, Math.Max(0, peak - win.HalfWidth), Math.Min(pow.Length - 1, peak + win.HalfWidth));
            if (pwr < 1e-10) continue;
            double exact = SequenceTransform.Centroid(pow, peak, win.HalfWidth);
            if (exact > 0)
            {
                ratios.Add(expectedBin / exact);
                powers.Add(pwr);
            }
        }
        if (ratios.Count == 0) return 1.0;
        var sorted = ratios.OrderBy(v => v).ToArray();
        double median = sorted[sorted.Length / 2];
        double num = 0.0, den = 0.0;
        for (int i = 0; i < ratios.Count; i++)
        {
            if (Math.Abs(ratios[i] - median) <= 1e-4)
            {
                num += ratios[i] * powers[i];
                den += powers[i];
            }
        }
        return den > 0 ? num / den : median;
    }

    private static bool[] BuildToneExclusionMask(int[] toneBins, int halfBins, double pctHalfWidth)
    {
        var mask = new bool[halfBins];
        int last = halfBins - 1;
        for (int i = 0; i < toneBins.Length; i++)
        {
            int prev = i > 0 ? 2 * toneBins[i - 1] : -1;
            int next = i < toneBins.Length - 1 ? 2 * toneBins[i + 1] : -1;
            ComputeToneBinBounds(2 * toneBins[i], prev, next, last, pctHalfWidth, out int lo, out int hi);
            for (int k = Math.Max(0, lo); k <= Math.Min(last, hi); k++) mask[k] = true;
        }
        return mask;
    }

    private static void ComputeToneBinBounds(int bin, int prevBin, int nextBin, int maxBin, double pctHalfWidth,
        out int minBin, out int maxOut)
    {
        int half = (int)Math.Ceiling(pctHalfWidth * bin);
        int leftLimit = prevBin != -1 ? Math.Max(((bin - prevBin) >> 1) - 1, 0) : bin;
        int rightLimit = nextBin != -1 ? Math.Max(((nextBin - bin) >> 1) - 1, 0) : maxBin - bin;
        minBin = bin - Math.Min(leftLimit, half);
        maxOut = bin + Math.Min(rightLimit, half);
    }

    private static double BandPower(double[] pow, int lo, int hi)
    {
        double sum = 0.0;
        int s = Math.Max(0, lo);
        int e = Math.Min(pow.Length - 1, hi);
        for (int k = s; k <= e; k++) sum += pow[k];
        return sum;
    }
}
