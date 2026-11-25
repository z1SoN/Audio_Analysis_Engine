using System.Collections.Concurrent;
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APxRT 10.0.0.17 实时计权与去加重滤波器完整复刻（覆盖全部 11 种 WtFilterType）。
/// 对应 APxRT.c：AnsiWeightingFilter (100127e0/10013b70)、CcirFilter (10013590)、
/// CCITTFilter (100128a0)、CMessageFilter (10013020)、DeemphasisFilter (10013a80)、
/// DeemphasisAwtFilter (10013980) 及 IirFromAnalogProto 单形优化器 (1000ad90/100097b0/10011710)。
/// </summary>
internal static class ApxAnsiWeighting
{
    private readonly record struct AnalogStage(double Pole1, double Pole2, int ZerosAtOrigin, double Imaginary = 0);
    private readonly record struct PrototypeSpec(AnalogStage[] Stages, int FittedSections, double ReferenceHz, double OverallGain);
    private readonly record struct Design(StreamingIirCascade.BiquadCoeffs[] Sections, double Gain);

    private static readonly ConcurrentDictionary<(int Rate, AnalyzerWeighting Weight), Design> Designs = new();

    private static readonly AnalogStage AnsiHigh = new(-76618.52508695953, -76618.52508695953, 2);
    private static readonly AnalogStage AnsiLow = new(-129.42731529303637, -129.42731529303637, 0);
    private static readonly AnalogStage AnsiMidA = new(-4636.125122258764, -676.4015487589464, 2);
    private static readonly AnalogStage AnsiMidB = new(-995.8848711879644, 0, 1);

    private static PrototypeSpec? GetSpec(AnalyzerWeighting weighting) => weighting switch
    {
        AnalyzerWeighting.AWeighting => new([AnsiHigh, AnsiMidA, AnsiLow], 1, 1000.0, 1.0),
        AnalyzerWeighting.BWeighting => new([AnsiHigh, AnsiMidB, AnsiLow], 1, 1000.0, 1.0),
        AnalyzerWeighting.CWeighting => new([AnsiHigh, AnsiLow], 1, 1000.0, 1.0),
        AnalyzerWeighting.Ccir1k => new(
        [
            new(-25903.701161964338, -62675.17014007283, 1),
            new(-18743.746620223184, -18743.746620223184, 0, 62460.15671371604),
            new(-23615.535228758155, -23615.535228758155, 0, 36379.90898596716)
        ], 1, 1000.0, 1.0),
        AnalyzerWeighting.Ccir2k => new(
        [
            new(-25903.701161964338, -62675.17014007283, 1),
            new(-18743.746620223184, -18743.746620223184, 0, 62460.15671371604),
            new(-23615.535228758155, -23615.535228758155, 0, 36379.90898596716)
        ], 1, 2000.0, 1.0),
        // APxRT 100128a0 CCITTFilter: 6 sections, 2 dynamically fitted numerators, ref = 800 Hz.
        AnalyzerWeighting.Ccitt => new(
        [
            new(-4257.61235457667, -585.846148593471, 2),
            new(-1125.81740971204, -1125.81740971204, 2, 1707.37194927851),
            new(-2921.37811768207, -2921.37811768207, 0, 5845.69608723553),
            new(-5282.91893850458, -5282.91893850458, 0, 14421.9191513711),
            new(-3853.72378407296, -3853.72378407296, 0, 21107.1711110092),
            new(-3788.88180907736, -3788.88180907736, 0, 26107.7485960876)
        ], 2, 800.0, 1.0),
        // APxRT 10013020 CMessageFilter: 4 sections, 1 dynamically fitted numerator, ref = 1000 Hz.
        AnalyzerWeighting.CMessage => new(
        [
            new(-4017.0, -4017.0, 2, 21575.0),
            new(-1502.0, -1502.0, 1, 1267.0),
            new(-2439.0, -2439.0, 0, 5336.0),
            new(-4690.0, -4690.0, 0, 15267.0)
        ], 1, 1000.0, 1.0),
        // APxRT 10013a80 DeemphasisFilter: 1 section, pole -1/tau, ref = 0 Hz (DC).
        AnalyzerWeighting.Deemphasis50us => new([new(-1.0 / 50e-6, 0, 0)], 1, 0.0, 1.0),
        AnalyzerWeighting.Deemphasis75us => new([new(-1.0 / 75e-6, 0, 0)], 1, 0.0, 1.0),
        // APxRT 10013980 DeemphasisAwtFilter: 3 A-wt sections + 1 De-emphasis pole, ref = 1000 Hz, overall gain 0.9541 / 0.9047.
        AnalyzerWeighting.Deemphasis50usA => new([AnsiHigh, AnsiMidA, AnsiLow, new(-1.0 / 50e-6, 0, 0)], 1, 1000.0, 0.9541),
        AnalyzerWeighting.Deemphasis75usA => new([AnsiHigh, AnsiMidA, AnsiLow, new(-1.0 / 75e-6, 0, 0)], 1, 1000.0, 0.9047),
        _ => null
    };

    // 给每条信号通路创建独立的滤波状态。相同采样率和计权可以共用系数，不能共用某一声道的历史样本。
    internal static StreamingIirCascade Create(AnalyzerWeighting weighting, int sampleRate)
    {
        if (weighting == AnalyzerWeighting.Flat || sampleRate <= 0)
            return StreamingIirCascade.Bypass();

        var spec = GetSpec(weighting);
        if (spec is null)
            return StreamingIirCascade.CreateWeightingFilter(weighting, sampleRate);

        var design = Designs.GetOrAdd((sampleRate, weighting), static key => Calculate(key.Rate, GetSpec(key.Weight)!.Value));
        return new StreamingIirCascade(design.Sections, design.Gain, useApxStateOrder: true);
    }

    /// <summary>按 APxRT 模拟原型精确计算任意计权在指定频率处的归一化功率响应 |H(jω)|²。</summary>
    internal static double EvaluateAnalogPowerGain(double frequencyHz, AnalyzerWeighting weighting)
    {
        if (weighting == AnalyzerWeighting.Flat) return 1.0;
        var spec = GetSpec(weighting);
        if (spec is null) return 1.0;
        double refPow = EvaluateAnalogRawPower(spec.Value.Stages, spec.Value.ReferenceHz * Math.Tau);
        if (refPow <= 0) return 0.0;
        double pow = EvaluateAnalogRawPower(spec.Value.Stages, frequencyHz * Math.Tau);
        double gainSq = spec.Value.OverallGain * spec.Value.OverallGain;
        return (pow / refPow) * gainSq;
    }

    private static double EvaluateAnalogRawPower(AnalogStage[] stages, double omega)
    {
        var s = new Complex(0, omega);
        Complex h = Complex.One;
        foreach (var p in stages)
        {
            Complex den = s - new Complex(p.Pole1, p.Imaginary);
            if (p.Pole2 != 0) den *= s - new Complex(p.Pole2, -p.Imaginary);
            Complex num = p.ZerosAtOrigin switch { 2 => s * s, 1 => s, _ => Complex.One };
            h *= num / den;
        }
        return Power(h);
    }

    // 目标是让数字滤波器在一系列频率上的相对功率跟随模拟原型。
    private static Design Calculate(int rate, PrototypeSpec spec)
    {
        var prototype = spec.Stages;
        int fittedCount = spec.FittedSections;
        int fixedCount = prototype.Length - fittedCount;
        var sections = new StreamingIirCascade.BiquadCoeffs[prototype.Length];
        double dt = 1d / rate;

        // 模拟极点按冲激不变映射 z = exp(p * dt)；前 fixedCount 节保留原点零点，后 fittedCount 节分子待优化。
        for (int i = 0; i < prototype.Length; i++)
        {
            var stage = prototype[i];
            double a1, a2;
            if (stage.Imaginary != 0)
            {
                var pole = Complex.Exp(new Complex(stage.Pole1, stage.Imaginary) * dt);
                a1 = -2 * pole.Real;
                a2 = Power(pole);
            }
            else
            {
                double pole1 = Math.Exp(stage.Pole1 * dt);
                double pole2 = stage.Pole2 == 0 ? 0 : Math.Exp(stage.Pole2 * dt);
                a1 = -(pole1 + pole2);
                a2 = pole1 * pole2;
            }
            if (i < fixedCount)
            {
                sections[i] = new(1, -stage.ZerosAtOrigin, stage.ZerosAtOrigin == 2 ? 1 : 0, a1, a2);
            }
            else
            {
                sections[i] = new(1, 0, 0, a1, a2);
            }
        }

        // APxRT 1000ad90 compares 101 logarithmic frequencies from 20 Hz through Nyquist.
        const int lastPoint = 100;
        const double anchor = 20 * Math.Tau;
        double ratio = Math.PI * rate / anchor;
        var target = new (Complex Z, Complex Z2, double AnalogPower, double DigitalPower)[lastPoint + 1];
        var reference = Response(spec.ReferenceHz * Math.Tau);
        for (int i = 0; i <= lastPoint; i++)
        {
            var point = Response(anchor * Math.Pow(ratio, i / (double)lastPoint));
            target[i] = (point.Z, point.Z2, point.AnalogPower / reference.AnalogPower, point.DigitalPower / reference.DigitalPower);
        }

        // 1000ad90 L7205-7217: 用 Nyquist 频点的幅值比初始化所有待拟合节的 (b1, b2) = (seed, -seed)
        double edgeGain = Math.Sqrt(target[^1].AnalogPower / target[^1].DigitalPower);
        double seed = (1 - edgeGain) / 2;
        int dim = fittedCount * 2;
        var start = new double[dim];
        for (int s = 0; s < fittedCount; s++)
        {
            start[2 * s] = seed;
            start[2 * s + 1] = -seed;
        }

        var fitted = FitNelderMead(start, Score);
        for (int s = 0; s < fittedCount; s++)
        {
            sections[fixedCount + s] = sections[fixedCount + s] with
            {
                B1 = fitted[2 * s],
                B2 = fitted[2 * s + 1]
            };
        }

        double gain = 1;
        for (int i = 0; i < sections.Length; i++)
        {
            var c = sections[i];
            var numerator = c.B1 * reference.Z + Complex.One + c.B2 * reference.Z2;
            var denominator = c.A1 * reference.Z + Complex.One + c.A2 * reference.Z2;
            gain *= Math.Sqrt(Power(numerator) / Power(denominator));
        }
        return new(sections, spec.OverallGain / gain);

        double Score(double[] x)
        {
            double refNumPow = 1.0;
            for (int s = 0; s < fittedCount; s++)
                refNumPow *= Power(Complex.One + x[2 * s] * reference.Z + x[2 * s + 1] * reference.Z2);
            double inverseReference = 1.0 / refNumPow;
            double worst = 0;
            foreach (var point in target)
            {
                double numPow = 1.0;
                for (int s = 0; s < fittedCount; s++)
                    numPow *= Power(Complex.One + x[2 * s] * point.Z + x[2 * s + 1] * point.Z2);
                double error = Math.Abs(10 * Math.Log10(point.DigitalPower * numPow * inverseReference / point.AnalogPower));
                if (error > worst) worst = error;
            }
            return worst;
        }

        (Complex Z, Complex Z2, double AnalogPower, double DigitalPower) Response(double omega)
        {
            var s = new Complex(0, omega);
            var z = Complex.FromPolarCoordinates(1, -omega * dt);
            var z2 = z * z;
            Complex analog = Complex.One, digital = Complex.One;
            for (int i = 0; i < sections.Length; i++)
            {
                var p = prototype[i];
                var c = sections[i];
                Complex analogDenominator = s - new Complex(p.Pole1, p.Imaginary);
                if (p.Pole2 != 0) analogDenominator *= s - new Complex(p.Pole2, -p.Imaginary);
                Complex analogNumerator = p.ZerosAtOrigin switch { 2 => s * s, 1 => s, _ => Complex.One };
                analog *= analogNumerator / analogDenominator;
                digital *= (Complex.One + c.B1 * z + c.B2 * z2) / (Complex.One + c.A1 * z + c.A2 * z2);
            }
            return (z, z2, Power(analog), Power(digital));
        }
    }

    private static double Power(Complex value) => value.Real * value.Real + value.Imaginary * value.Imaginary;

    // 通用 N 维 Nelder-Mead 单形优化器（严格对齐 APxRT 10011710 / 100115b0：索引跟踪 best/secondWorst/worst、无排序、shrink 不更新 best）
    private static double[] FitNelderMead(double[] start, Func<double[], double> score)
    {
        int n = start.Length;
        int m = n + 1;
        var vertices = new double[m][];
        var values = new double[m];

        vertices[0] = (double[])start.Clone();
        values[0] = score(vertices[0]);
        int bestIdx = 0, worstIdx = 0, secondWorstIdx = 0;
        double bestVal = values[0], worstVal = values[0], secondWorstVal = values[0];

        for (int i = 1; i < m; i++)
        {
            var pt = (double[])start.Clone();
            pt[i - 1] = pt[i - 1] == 0.0 ? 0.00025 : pt[i - 1] * 1.05;
            vertices[i] = pt;
            double f = score(pt);
            values[i] = f;
            if (bestVal <= f)
            {
                if (f <= worstVal)
                {
                    if (secondWorstVal < f && f < worstVal)
                    {
                        secondWorstVal = f;
                        secondWorstIdx = i;
                    }
                }
                else
                {
                    secondWorstVal = worstVal;
                    secondWorstIdx = worstIdx;
                    worstVal = f;
                    worstIdx = i;
                }
            }
            else
            {
                bestVal = f;
                bestIdx = i;
            }
        }

        static double SampleVariance(double[] f)
        {
            double f0 = f[0];
            double sum = 0.0, sumSq = 0.0;
            for (int i = 1; i < f.Length; i++)
            {
                double d = f[i] - f0;
                sum += d;
                sumSq += d * d;
            }
            return Math.Abs((sumSq - sum * sum / f.Length) / (f.Length - 1));
        }

        double variance = SampleVariance(values);
        int iter = 1;
        var centroid = new double[n];
        var reflected = new double[n];
        var expanded = new double[n];
        var contracted = new double[n];

        while (variance >= 1e-18 && iter <= 5000)
        {
            Array.Clear(centroid);
            for (int i = 0; i < m; i++)
            {
                if (i == worstIdx) continue;
                var vi = vertices[i];
                for (int j = 0; j < n; j++) centroid[j] += vi[j];
            }
            for (int j = 0; j < n; j++)
            {
                centroid[j] /= n;
                reflected[j] = 2.0 * centroid[j] - vertices[worstIdx][j];
            }

            double fRefl = score(reflected);
            if (bestVal <= fRefl && fRefl < secondWorstVal)
            {
                Array.Copy(reflected, vertices[worstIdx], n);
                values[worstIdx] = fRefl;
                int prevSec = secondWorstIdx;
                worstVal = secondWorstVal;
                worstIdx = secondWorstIdx;
                secondWorstVal = bestVal;
                secondWorstIdx = bestIdx;
                for (int i = 0; i < m; i++)
                {
                    if (i != bestIdx && i != prevSec && secondWorstVal < values[i])
                    {
                        secondWorstVal = values[i];
                        secondWorstIdx = i;
                    }
                }
            }
            else if (fRefl < bestVal)
            {
                for (int j = 0; j < n; j++)
                    expanded[j] = (centroid[j] - vertices[worstIdx][j]) * 2.0 + centroid[j];
                double fExp = score(expanded);
                bestIdx = worstIdx;
                int prevSec = secondWorstIdx;
                if (fRefl <= fExp)
                {
                    Array.Copy(reflected, vertices[worstIdx], n);
                    values[worstIdx] = fRefl;
                    bestVal = fRefl;
                }
                else
                {
                    Array.Copy(expanded, vertices[worstIdx], n);
                    values[worstIdx] = fExp;
                    bestVal = fExp;
                }
                worstIdx = secondWorstIdx;
                worstVal = secondWorstVal;
                secondWorstVal = bestVal;
                secondWorstIdx = bestIdx;
                for (int i = 0; i < m; i++)
                {
                    if (i != bestIdx && i != prevSec && secondWorstVal < values[i])
                    {
                        secondWorstVal = values[i];
                        secondWorstIdx = i;
                    }
                }
            }
            else
            {
                bool doShrink = false;
                if (fRefl >= secondWorstVal)
                {
                    if (worstVal <= fRefl)
                    {
                        for (int j = 0; j < n; j++)
                            contracted[j] = (vertices[worstIdx][j] + centroid[j]) / 2.0;
                        double fIc = score(contracted);
                        if (worstVal <= fIc)
                        {
                            doShrink = true;
                        }
                        else
                        {
                            Array.Copy(contracted, vertices[worstIdx], n);
                            values[worstIdx] = fIc;
                            worstVal = fIc;
                            if (fIc <= secondWorstVal)
                            {
                                if (fIc < bestVal)
                                {
                                    bestIdx = worstIdx;
                                    bestVal = fIc;
                                }
                                int prevSec = secondWorstIdx;
                                worstVal = secondWorstVal;
                                worstIdx = secondWorstIdx;
                                secondWorstVal = bestVal;
                                secondWorstIdx = bestIdx;
                                for (int i = 0; i < m; i++)
                                {
                                    if (i != bestIdx && i != prevSec && secondWorstVal < values[i])
                                    {
                                        secondWorstVal = values[i];
                                        secondWorstIdx = i;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        for (int j = 0; j < n; j++)
                            contracted[j] = (reflected[j] + centroid[j]) / 2.0;
                        double fOc = score(contracted);
                        if (fRefl <= fOc)
                        {
                            doShrink = true;
                        }
                        else
                        {
                            Array.Copy(contracted, vertices[worstIdx], n);
                            values[worstIdx] = fOc;
                            worstVal = fOc;
                            if (fOc <= secondWorstVal)
                            {
                                if (fOc < bestVal)
                                {
                                    bestIdx = worstIdx;
                                    bestVal = fOc;
                                }
                                int prevSec = secondWorstIdx;
                                worstVal = secondWorstVal;
                                worstIdx = secondWorstIdx;
                                secondWorstVal = bestVal;
                                secondWorstIdx = bestIdx;
                                for (int i = 0; i < m; i++)
                                {
                                    if (i != bestIdx && i != prevSec && secondWorstVal < values[i])
                                    {
                                        secondWorstVal = values[i];
                                        secondWorstIdx = i;
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    doShrink = true;
                }

                if (doShrink)
                {
                    secondWorstVal = bestVal;
                    worstVal = bestVal;
                    secondWorstIdx = bestIdx;
                    worstIdx = bestIdx;
                    for (int i = 0; i < m; i++)
                    {
                        if (i == bestIdx) continue;
                        var vi = vertices[i];
                        var vb = vertices[bestIdx];
                        for (int j = 0; j < n; j++)
                            vi[j] = (vb[j] + vi[j]) / 2.0;
                        double f = score(vi);
                        values[i] = f;
                        if (worstVal < f)
                        {
                            secondWorstVal = worstVal;
                            secondWorstIdx = worstIdx;
                            worstVal = f;
                            worstIdx = i;
                        }
                        if (secondWorstVal < f && f < worstVal)
                        {
                            secondWorstVal = f;
                            secondWorstIdx = i;
                        }
                    }
                }
            }

            variance = SampleVariance(values);
            iter++;
        }

        return vertices[bestIdx];
    }
}
