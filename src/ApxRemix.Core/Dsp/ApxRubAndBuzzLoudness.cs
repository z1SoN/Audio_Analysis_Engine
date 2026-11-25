using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APx500 10.0.0.17 Rub &amp; Buzz 感知响度模型独立复刻（对齐 <c>AudioPrecision.Processing.Measurements.RubAndBuzzLoudness.PerceputalModels</c>）。
/// 包含外耳/中耳频响计权、Bark 临界频带划分（0.25 Bark 步进）、内耳本底噪声、非对称频率掩蔽扩散、
/// ISO/Zwicker 特定响度与总响度（Sone）、部分响度及误差谐波结构（EHS）。
/// </summary>
public static class ApxRubAndBuzzLoudness
{
    public static double[] ApplyEarWeighting(ReadOnlySpan<double> spectrum, double sampleRate)
    {
        if (spectrum.IsEmpty) return [];
        if (!double.IsFinite(sampleRate) || sampleRate <= 0.0) return spectrum.ToArray();
        double df = sampleRate / (2.0 * spectrum.Length);
        var result = new double[spectrum.Length];
        for (int i = 0; i < spectrum.Length; i++)
        {
            double fKhz = i * df / 1000.0;
            double db = -2.184 * Math.Pow(fKhz, -0.8)
                + 6.5 * Math.Exp(-0.6 * Math.Pow(fKhz - 3.3, 2.0))
                - 0.001 * Math.Pow(fKhz, 3.6);
            result[i] = spectrum[i] * Math.Pow(10.0, db / 20.0);
        }
        return result;
    }

    public static double HArcsin(double x) => Math.Log(x + Math.Sqrt(x * x + 1.0));

    public static double[] ConvertFrequencyAxisToBarkAxis(ReadOnlySpan<double> freqAxis)
    {
        var bark = new double[freqAxis.Length];
        for (int i = 0; i < freqAxis.Length; i++)
            bark[i] = Math.Max(0.0, 7.0 * HArcsin(freqAxis[i] / 650.0));
        return bark;
    }

    public static double[] FindCenterFrequencyOfBarkSubdivisions(int numSubdivisions)
    {
        if (numSubdivisions <= 0) return [];
        var centers = new double[numSubdivisions];
        const double step = 0.25;
        var bounds = ConvertFrequencyAxisToBarkAxis([80.0, 18000.0]);
        for (int i = 0; i < numSubdivisions; i++)
        {
            double lo = bounds[0] + i * step;
            double hi = Math.Min(bounds[0] + (i + 1) * step, bounds[1]);
            double mid = 0.5 * (hi - lo) + lo;
            centers[i] = 650.0 * Math.Sinh(mid / 7.0);
        }
        return centers;
    }

    public static double[] ApplyAuditoryFilterBands(ReadOnlySpan<double> freqAxis, ReadOnlySpan<double> inputSpectrum)
    {
        if (freqAxis.IsEmpty) return [];
        var bark = ConvertFrequencyAxisToBarkAxis(freqAxis);
        int count = (int)Math.Ceiling(4.0 * (bark[^1] - 1.0));
        if (count <= 0) return [];
        var bands = new double[count];
        for (int i = 0; i < inputSpectrum.Length; i++)
        {
            if (freqAxis[i] < 91.7 || freqAxis[i] > 17700.0) continue;
            int idx = (int)Math.Min((bark[i] - 1.0) * 4.0, count - 1);
            if (idx >= 0) bands[idx] += inputSpectrum[i];
        }
        return bands;
    }

    public static double[] GenerateInnerEarNoise(int numBarkSubBands)
    {
        var noise = new double[numBarkSubBands];
        var centers = FindCenterFrequencyOfBarkSubdivisions(numBarkSubBands);
        for (int i = 0; i < numBarkSubBands; i++)
            noise[i] = 1.4560000000000002 * Math.Pow(centers[i] / 1000.0, -0.8);
        return noise;
    }

    public static double[] ApplyFrequencySpreading(ReadOnlySpan<double> barkBands)
    {
        if (barkBands.IsEmpty) return [];
        const double lowerSlope = 27.0;
        const double barkStep = 0.25;
        const double p = 0.4;
        int n = barkBands.Length;
        var excitation = new double[n];
        var centers = FindCenterFrequencyOfBarkSubdivisions(n);

        for (int i = 0; i < n; i++)
        {
            double sum = 0.0;
            for (int j = 0; j < n; j++)
            {
                double levelDb = 20.0 * Math.Log10(barkBands[j]);
                double upperSlope = Math.Min(0.0, -24.0 - 230.0 / centers[j] + 0.2 * levelDb);
                double spread = Math.Pow(10.0, 0.1 * (levelDb + barkStep * (i - j) * (i < j ? lowerSlope : upperSlope)));
                sum += Math.Pow(spread, p);
            }
            excitation[i] = Math.Pow(sum, 1.0 / p);
        }

        for (int k = 0; k < n; k++)
        {
            double normSum = 0.0;
            for (int l = 0; l < n; l++)
            {
                double upperSlope = Math.Min(0.0, -24.0 - 230.0 / centers[l]);
                double spread = Math.Pow(10.0, 0.1 * (barkStep * (k - l) * (k < l ? lowerSlope : upperSlope)));
                normSum += Math.Pow(spread, p);
            }
            excitation[k] /= Math.Pow(normSum, 1.0 / p);
        }
        return excitation;
    }

    public static double[] CalculateLoudness(ReadOnlySpan<double> excitations, out double[] thresholds, out double totalLoudness)
    {
        totalLoudness = 0.0;
        if (excitations.IsEmpty)
        {
            thresholds = [];
            return [];
        }
        int n = excitations.Length;
        var centers = FindCenterFrequencyOfBarkSubdivisions(n);
        const double alpha = 0.23;
        const double c = 1.07664;
        var specific = new double[n];
        thresholds = new double[n];
        for (int i = 0; i < n; i++)
        {
            double eTq = Math.Pow(10.0, 0.364 * Math.Pow(centers[i] / 1000.0, -0.8));
            thresholds[i] = Math.Pow(10.0, 0.1 * (-2.0 - 2.05 * Math.Atan(centers[i] / 4000.0)
                - 0.75 * Math.Atan(Math.Pow(centers[i] / 1600.0, 2.0))));
            specific[i] = c * Math.Pow(eTq / (thresholds[i] * 10000.0), alpha)
                * (Math.Pow(1.0 - thresholds[i] + thresholds[i] * (excitations[i] / 4.0000000000000007e-10) / eTq, alpha) - 1.0);
            if (double.IsFinite(specific[i]))
                totalLoudness += Math.Max(0.0, specific[i]);
        }
        totalLoudness *= 0.22018348623853212;
        return specific;
    }

    public static double[] CalculatePartialLoudness(ReadOnlySpan<double> eResp, ReadOnlySpan<double> eRef,
        ReadOnlySpan<double> earNoiseCurve, ReadOnlySpan<double> threshResp, ReadOnlySpan<double> threshRef)
    {
        int n = eResp.Length;
        if (n == 0 || eRef.Length != n || earNoiseCurve.Length != n || threshResp.Length != n || threshRef.Length != n)
            return [];
        const double c = 0.107664;
        var result = new double[n];
        for (int i = 0; i < n; i++)
        {
            double beta = Math.Exp(-1.5 * (eResp[i] - eRef[i]) / eRef[i]);
            result[i] = c * Math.Pow(earNoiseCurve[i] / threshResp[i], 0.23)
                * (Math.Pow(1.0 + Math.Max(threshResp[i] * eResp[i] - threshRef[i] * eRef[i], 0.0)
                    / (earNoiseCurve[i] + threshRef[i] * eRef[i] * beta), 0.23) - 1.0);
        }
        return result;
    }

    public static double CalculateErrorHarmonicStructure(ReadOnlySpan<double> inputSpectrum, double sampleRate, double stimulusHz)
    {
        if (inputSpectrum.IsEmpty) return 0.0;
        int n = inputSpectrum.Length;
        var logSpec = new double[n];
        double mean = 0.0;
        for (int i = 0; i < n; i++)
        {
            logSpec[i] = Math.Log(inputSpectrum[i] + double.Epsilon);
            mean += logSpec[i];
        }
        mean /= n;
        for (int i = 0; i < n; i++) logSpec[i] -= mean;

        var hp4 = StreamingIirCascade.CreateButterworthHighPass(4, 100.0, 48000);
        ZeroPhaseButterworthHp4(hp4, logSpec);

        var win = FftEngine.CreateWindow(n, FftWindowType.Hann).Coefficients;
        var buf = new Complex[n];
        for (int i = 0; i < n; i++) buf[i] = new Complex(logSpec[i] * win[i], 0.0);
        SequenceTransform.Inverse(buf);

        var cepstrum = new double[n];
        for (int i = 0; i < n; i++) cepstrum[i] = buf[i].Magnitude;
        hp4.Reset();
        ZeroPhaseButterworthHp4(hp4, cepstrum);

        int targetQuef = (int)(sampleRate / (stimulusHz * 2.0));
        if (targetQuef < 5 || targetQuef >= n / 2) return 0.0;

        double peak = cepstrum[targetQuef];
        int halfWidth = SequenceTransform.HalfWidth(FftWindowType.Hann);
        int end = Math.Min(n / 2, targetQuef + halfWidth);
        for (int m = targetQuef; m < end; m++)
            if (cepstrum[m] >= peak) peak = cepstrum[m];

        if (stimulusHz > 1000.0)
            peak *= Math.Pow(-5.2631578947368424e-5 * (stimulusHz - 1000.0) + 1.0, 3.0);

        double sumSq = 0.0;
        for (int i = 0; i < n; i++) sumSq += cepstrum[i] * cepstrum[i];
        if (!(peak > Math.Sqrt(sumSq / n))) return 0.0;
        return peak * sampleRate / 192000.0;
    }

    private static void ZeroPhaseButterworthHp4(StreamingIirCascade filter, double[] data)
    {
        // APx PerceputalModels.CalculateErrorHarmonicStructure (Om.decompiled.cs:24717-24736):
        // forward pass and reversed pass share the same filter instance without intermediate Reset().
        filter.ProcessInPlace(data);
        Array.Reverse(data);
        filter.ProcessInPlace(data);
        Array.Reverse(data);
    }
}
