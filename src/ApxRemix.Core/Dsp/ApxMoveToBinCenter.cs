using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APx500 10.0.0.17 <c>MoveToBinCenterLibrary</c> 独立复刻。
/// 当信号主峰落在 FFT 频格中心的 ±3% 容差内时，通过 140 抽头 Kaiser 窗 Sinc 抗混叠重采样
/// 将主峰精确搬移至整数频格中心，从而允许无窗（矩形窗）FFT 获得零频谱泄漏的高动态范围频谱。
/// </summary>
public static class ApxMoveToBinCenter
{
    public const int SrcFirLen = 140;
    public const double SrcFirHalfLen = 69.0;
    public const double FftLenMultiplier = 1.03;

    private const int PolyphaseCount = 64;
    private const double KaiserBeta = 14.0; // >120 dB stopband sidelobe rejection
    private static readonly Lazy<double[,]> PolyphaseTable = new(BuildPolyphaseSincTable);

    /// <summary>对齐 <c>MoveToBinCenterLibrary.CalculateMinimumAcqLength</c>。</summary>
    public static int CalculateMinimumAcqLength(int desiredFftLength)
    {
        if (desiredFftLength <= 0) return 0;
        return Math.Max(desiredFftLength + SrcFirLen, (int)(FftLenMultiplier * desiredFftLength + 0.5));
    }

    /// <summary>对齐 <c>MoveToBinCenterLibrary.CheckIfFreqWithinTolerance</c>（±3% 整数频格容差）。</summary>
    public static bool IsFrequencyWithinTolerance(double measuredHz, double sampleRate, int fftLength)
    {
        if (!(measuredHz > 0) || !(sampleRate > 0) || fftLength < 16) return false;
        double df = sampleRate / fftLength;
        for (int k = 1; df * k <= sampleRate * 0.5; k++)
        {
            double target = df * k;
            if (measuredHz >= target * 0.97 && measuredHz <= target * 1.03)
                return true;
        }
        return false;
    }

    /// <summary>对齐 <c>MoveToBinCenterLibrary.ComputeFrequencyRatio</c>（BlackmanHarris4 加窗质心求 peakBin / exactBin）。</summary>
    public static double? ComputeFrequencyRatio(ReadOnlySpan<double> samples, FftWindowType windowType = FftWindowType.BlackmanHarris4)
    {
        int n = samples.Length;
        if (n < 16 || n % 2 != 0) return null;
        var window = SequenceFftWindow.Get(n, windowType);
        var fft = window.Transform(samples);
        var pow = SequenceFftWindow.Power(fft, n, n / 2 + 1);
        int peakBin = -1;
        double peakVal = 0.0;
        for (int k = 5; k <= n / 2; k++)
        {
            if (pow[k] > peakVal) { peakVal = pow[k]; peakBin = k; }
        }
        if (peakBin <= 0) return null;
        double exactBin = SequenceTransform.Centroid(pow, peakBin, window.HalfWidth);
        if (!(exactBin > 0)) return null;
        return peakBin / exactBin;
    }

    /// <summary>
    /// 对齐 <c>MoveToBinCenterLibrary.MoveToBinCenter</c>：检测主峰并在 ±3% 范围内重采样至整数频格中心。
    /// </summary>
    public static double[] MoveToBinCenter(ReadOnlySpan<double> data, int desiredLength, double sampleRate, out double? srcRatio)
    {
        int minLen = CalculateMinimumAcqLength(desiredLength);
        if (data.Length < minLen)
            throw new ArgumentException($"MoveToBinCenter requires at least {minLen} samples for FFT length {desiredLength}.", nameof(data));

        var probe = data[..desiredLength];
        // 全 NaN 常量保护（与原版一致）
        if (double.IsNaN(data[0]))
        {
            bool allSame = true;
            for (int i = 1; i < data.Length; i++)
                if (!double.IsNaN(data[i])) { allSame = false; break; }
            if (allSame)
            {
                srcRatio = null;
                return probe.ToArray();
            }
        }

        // 检查主频 > 4 bin 且在整数频格 ±3% 内
        var probeWin = SequenceFftWindow.Get(desiredLength, FftWindowType.BlackmanHarris4);
        var probePow = SequenceFftWindow.Power(probeWin.Transform(probe), desiredLength, desiredLength / 2 + 1);
        int peakBin = -1;
        double peakVal = 0.0;
        for (int k = 1; k <= desiredLength / 2; k++)
        {
            if (probePow[k] > peakVal) { peakVal = probePow[k]; peakBin = k; }
        }
        double exactBin = peakBin > 0 ? SequenceTransform.Centroid(probePow, peakBin, probeWin.HalfWidth) : 0.0;
        double measuredHz = exactBin * sampleRate / desiredLength;
        if (exactBin <= 4.0 || !IsFrequencyWithinTolerance(measuredHz, sampleRate, desiredLength))
            throw new InvalidOperationException("Dominant synchronous frequency not detected within ±3% of an FFT bin center.");

        int outLen = Math.Min(data.Length - SrcFirLen, (int)(data.Length / FftLenMultiplier + 0.5));
        srcRatio = ComputeFrequencyRatio(probe);
        if (!srcRatio.HasValue || Math.Abs(srcRatio.Value - 1.0) <= 1e-12)
            return data[..outLen].ToArray();
        if (srcRatio.Value > 1.03 || srcRatio.Value < 0.97)
            throw new InvalidOperationException($"Computed SRC ratio {srcRatio.Value:F6} exceeds ±3% synchronous tolerance.");

        return Resample(data, srcRatio.Value, outLen);
    }

    /// <summary>
    /// 64 相 × 140 抽头多相 Sinc + 4 点拉格朗日相位插值重采样器（对齐 APx <c>MoveToBinCenterLibrary</c> / <c>MultitoneLibrary</c> 内部 SRC）。
    /// </summary>
    public static double[] Resample(ReadOnlySpan<double> input, double ratio, int outputLength)
    {
        if (outputLength <= 0) return [];
        if (Math.Abs(ratio - 1.0) <= 1e-12)
        {
            var copy = new double[outputLength];
            input[..Math.Min(input.Length, outputLength)].CopyTo(copy);
            return copy;
        }
        var table = PolyphaseTable.Value;
        var output = new double[outputLength];
        Span<double> kernel = stackalloc double[SrcFirLen];

        // 4-point Lagrange weights on {-1, 0, 1, 2}
        const double w0 = -1.0 / 6.0, w1 = 0.5, w2 = -0.5, w3 = 1.0 / 6.0;

        for (int i = 0; i < outputLength; i++)
        {
            double pos = i * ratio;
            int idx = (int)pos;
            double phase = (pos - idx) * PolyphaseCount; // [0, 64)
            int pInt = (int)phase;
            double pFrac = phase - pInt;
            if (pInt >= PolyphaseCount) { pInt = PolyphaseCount; pFrac = 0.0; }

            if (pFrac == 0.0)
            {
                int row = pInt + 1;
                for (int t = 0; t < SrcFirLen; t++) kernel[t] = table[row, t];
            }
            else
            {
                double l0 = w0 / (pFrac + 1.0);
                double l1 = w1 / pFrac;
                double l2 = w2 / (pFrac - 1.0);
                double l3 = w3 / (pFrac - 2.0);
                double invSum = 1.0 / (l0 + l1 + l2 + l3);
                for (int t = 0; t < SrcFirLen; t++)
                {
                    kernel[t] = (l0 * table[pInt, t] + l1 * table[pInt + 1, t]
                        + l2 * table[pInt + 2, t] + l3 * table[pInt + 3, t]) * invSum;
                }
            }

            double acc = 0.0;
            int maxT = Math.Min(SrcFirLen, input.Length - idx);
            for (int t = 0; t < maxT; t++)
                acc += input[idx + t] * kernel[t];
            output[i] = acc;
        }
        return output;
    }

    private static double[,] BuildPolyphaseSincTable()
    {
        // Rows 0..66 correspond to fractional shifts (-1..65) / 64 so row (p+1) has shift p/64.
        // Cutoff slightly below Nyquist (0.94) avoids aliasing even for ratio up to 1.03.
        const double cutoff = 0.94;
        var table = new double[PolyphaseCount + 3, SrcFirLen];
        double i0Beta = BesselI0(KaiserBeta);
        for (int row = 0; row < PolyphaseCount + 3; row++)
        {
            double shift = (row - 1) / (double)PolyphaseCount; // -1/64 .. 65/64
            double sum = 0.0;
            for (int t = 0; t < SrcFirLen; t++)
            {
                double x = t - SrcFirHalfLen - shift;
                double sinc = Math.Abs(x) < 1e-12 ? cutoff : Math.Sin(Math.PI * cutoff * x) / (Math.PI * x);
                double u = x / (SrcFirHalfLen + 1.0);
                double win = Math.Abs(u) >= 1.0 ? 0.0 : BesselI0(KaiserBeta * Math.Sqrt(1.0 - u * u)) / i0Beta;
                double v = sinc * win;
                table[row, t] = v;
                sum += v;
            }
            double inv = sum != 0 ? 1.0 / sum : 1.0;
            for (int t = 0; t < SrcFirLen; t++) table[row, t] *= inv;
        }
        return table;
    }

    private static double BesselI0(double x)
    {
        double sum = 1.0, term = 1.0;
        double half = 0.5 * x;
        for (int k = 1; k <= 30; k++)
        {
            term *= (half / k) * (half / k);
            sum += term;
            if (term < 1e-16 * sum) break;
        }
        return sum;
    }
}
