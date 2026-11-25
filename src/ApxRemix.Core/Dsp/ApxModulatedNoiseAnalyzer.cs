using System.Numerics;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APx500 10.0.0.17 调制噪声测量独立复刻（对齐 <c>AudioPrecision.Processing.Measurements.ModulatedNoise.ModulatedNoiseMeasurement</c>
/// 及 <c>Constantinides.ConstantinidesTransform</c>）。
/// 流程：整周期梳状陷波（基波 + 可选工频抑制）→ 4 阶 Constantinides 高通滤波（弃去前 3 周期瞬态）→ 全波整流 |x| → 无窗 FFT → 前 5 次调制谐波均值与非谐波噪声均值之比的平方根。
/// </summary>
public static class ApxModulatedNoiseAnalyzer
{
    private const double ProtoA2Stage1 = 0.03956612989658;
    private const double ProtoA2Stage2 = 0.44646269217169;
    private const double ProtoCutoff = 0.5;

    /// <summary>APx <c>Constantinides.ConstantinidesTransform</c> 二阶低通→高通极点映射。</summary>
    public static void ConstantinidesTransform(double fNormalized, double fProto, double a2Proto, out double a1, out double a2)
    {
        double alpha = Math.Sin(Math.PI * 0.5 * (fProto - fNormalized))
            / Math.Sin(Math.PI * 0.5 * (fProto + fNormalized));
        double alphaSq = alpha * alpha;
        double inv = 1.0 / (1.0 + alphaSq * a2Proto);
        a1 = inv * (-2.0 * alpha * (1.0 + a2Proto));
        a2 = inv * (alphaSq + a2Proto);
    }

    public static double[] Process(double[][] acquisitions, double sampleRate, double fundamentalHz, double highPassHz, double lineRejectHz = 0.0)
    {
        var results = new double[acquisitions.Length];
        for (int i = 0; i < acquisitions.Length; i++)
            results[i] = ProcessChannel(acquisitions[i], sampleRate, fundamentalHz, highPassHz, lineRejectHz, out _);
        return results;
    }

    public static double ProcessChannel(ReadOnlySpan<double> acquisition, double sampleRate, double fundamentalHz,
        double highPassHz, double lineRejectHz, out double[] filteredSignal)
    {
        if (acquisition.IsEmpty || !(sampleRate > 0) || !(fundamentalHz > 0))
            throw new ArgumentOutOfRangeException(nameof(acquisition));

        var work = acquisition.ToArray();
        int period = Math.Max(1, (int)Math.Round(sampleRate / fundamentalHz));
        ApplyCombNotch(work, period);
        if (lineRejectHz > 0.0)
        {
            int linePeriod = Math.Max(1, (int)Math.Round(sampleRate / lineRejectHz));
            ApplyCombNotch(work, linePeriod);
        }

        int totalPeriods = work.Length / period;
        int activeLength = (totalPeriods - 6) * period;
        if (activeLength < period * 2)
        {
            filteredSignal = [];
            return double.NaN;
        }

        double fN = Math.Min(highPassHz / sampleRate * 2.0, 1.0);
        ConstantinidesTransform(fN, ProtoCutoff, ProtoA2Stage1, out double a11, out double a12);
        ConstantinidesTransform(fN, ProtoCutoff, ProtoA2Stage2, out double a21, out double a22);

        var rectified = FilterAndRectify(work, activeLength, period, a11, a12, a21, a22, out filteredSignal);
        var spectrum = ForwardUnwindowedRealFft(rectified);
        int halfBins = activeLength >> 1;
        int fundamentalBin = activeLength / period;
        return ComputeRatio(spectrum, fundamentalBin, halfBins);
    }

    private static void ApplyCombNotch(double[] buffer, int delay)
    {
        for (int i = 0; i < buffer.Length - delay; i++)
            buffer[i] -= buffer[i + delay];
    }

    private static double[] FilterAndRectify(double[] source, int activeLength, int period,
        double a11, double a12, double a21, double a22, out double[] filteredSignal)
    {
        var rectified = new double[activeLength];
        filteredSignal = new double[activeLength];
        double z11 = 0.0, z12 = 0.0, z21 = 0.0, z22 = 0.0;

        int warmup = period * 3;
        for (int i = 0; i < warmup; i++)
        {
            double x = source[i];
            double y1 = x + z11;
            z11 = -2.0 * x - a11 * y1 + z12;
            z12 = x - a12 * y1;
            double y2 = y1 + z21;
            z21 = -2.0 * y1 - a21 * y2 + z22;
            z22 = y1 - a22 * y2;
        }

        for (int j = 0; j < activeLength; j++)
        {
            double x = source[j + warmup];
            double y1 = x + z11;
            z11 = -2.0 * x - a11 * y1 + z12;
            z12 = x - a12 * y1;
            double y2 = y1 + z21;
            z21 = -2.0 * y1 - a21 * y2 + z22;
            z22 = y1 - a22 * y2;
            filteredSignal[j] = y2;
            rectified[j] = Math.Abs(y2);
        }
        return rectified;
    }

    private static Complex[] ForwardUnwindowedRealFft(double[] samples)
    {
        int n = samples.Length;
        // Supports arbitrary even/composite lengths (not just powers of two) via Bluestein/radix-2 in SequenceFftWindow/FftEngine.
        if ((n & (n - 1)) == 0)
        {
            var buf = new Complex[n];
            for (int i = 0; i < n; i++) buf[i] = new Complex(samples[i], 0.0);
            FftEngine.ForwardInPlace(buf);
            return buf;
        }
        return BluesteinForward(samples);
    }

    private static Complex[] BluesteinForward(double[] samples)
    {
        int n = samples.Length;
        int m = 1;
        while (m < 2 * n - 1) m <<= 1;

        var chirp = new Complex[n];
        for (int k = 0; k < n; k++)
        {
            double angle = -Math.PI * ((double)((long)k * k % (2L * n))) / n;
            chirp[k] = Complex.FromPolarCoordinates(1.0, angle);
        }

        var a = new Complex[m];
        var b = new Complex[m];
        for (int k = 0; k < n; k++) a[k] = samples[k] * chirp[k];
        b[0] = Complex.Conjugate(chirp[0]);
        for (int k = 1; k < n; k++)
        {
            var c = Complex.Conjugate(chirp[k]);
            b[k] = c;
            b[m - k] = c;
        }

        FftEngine.ForwardInPlace(a);
        FftEngine.ForwardInPlace(b);
        for (int i = 0; i < m; i++) a[i] *= b[i];
        FftEngine.InverseInPlace(a);

        var result = new Complex[n];
        for (int k = 0; k < n; k++) result[k] = a[k] * chirp[k];
        return result;
    }

    private static double ComputeRatio(Complex[] spectrum, int fundamentalBin, int halfBins)
    {
        double harmonicPower = 0.0;
        double noisePower = 0.0;
        for (int i = fundamentalBin; i < halfBins; i++)
        {
            double re = spectrum[i].Real;
            double im = spectrum[i].Imaginary;
            double p = re * re + im * im;
            if (i % fundamentalBin != 0)
                noisePower += p;
            else if (i <= fundamentalBin * 5)
                harmonicPower += p;
        }
        int noiseBinCount = halfBins - 5 - fundamentalBin;
        if (noiseBinCount <= 0 || noisePower <= 0.0) return double.NaN;
        return Math.Sqrt((harmonicPower / 5.0) / (noisePower / noiseBinCount));
    }
}
