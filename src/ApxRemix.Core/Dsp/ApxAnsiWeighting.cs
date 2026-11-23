using System.Collections.Concurrent;
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

internal static class ApxAnsiWeighting
{
    private readonly record struct AnalogStage(double Pole1, double Pole2, int ZerosAtOrigin, double Imaginary = 0);
    private readonly record struct Design(StreamingIirCascade.BiquadCoeffs[] Sections, double Gain);
    private readonly record struct Candidate(double B1, double B2, double Error);
    private static readonly ConcurrentDictionary<(int Rate, AnalyzerWeighting Weight), Design> Designs = new();

    // 给每条信号通路创建独立的滤波状态。相同采样率和计权可以共用系数，不能共用某一声道的历史样本。
    internal static StreamingIirCascade Create(AnalyzerWeighting weighting, int sampleRate)
    {
        // 此处专门设计 A/B/C 和两种 CCIR，其余选择交给通用计权级，包括去加重和直通。
        if (weighting is not (AnalyzerWeighting.AWeighting or AnalyzerWeighting.BWeighting or AnalyzerWeighting.CWeighting
            or AnalyzerWeighting.Ccir1k or AnalyzerWeighting.Ccir2k))
            return StreamingIirCascade.CreateWeightingFilter(weighting, sampleRate);

        // Keep the result of the optimization, and give each channel its own filter state.
        var design = Designs.GetOrAdd((sampleRate, weighting), static key => Calculate(key.Rate, key.Weight));
        return new StreamingIirCascade(design.Sections, design.Gain, useApxStateOrder: true);
    }

    // 目标是让数字滤波器在一系列频率上的相对功率跟随模拟原型。本方法只设计系数，并未送入声卡实测。
    private static Design Calculate(int rate, AnalyzerWeighting weighting)
    {
        // APxRT 10.0.0.17 constructs these analog poles (rad/s) in 10013b70.
        var high = new AnalogStage(-76618.52508695953, -76618.52508695953, 2);
        var low = new AnalogStage(-129.42731529303637, -129.42731529303637, 0);
        // 每节保存一个或两个模拟极点与原点零点数。切换 A/B/C 会改变低频滚降，CCIR 还含两对复极点。
        AnalogStage[] prototype = weighting switch
        {
            AnalyzerWeighting.AWeighting => [high, new(-4636.125122258764, -676.4015487589464, 2), low],
            AnalyzerWeighting.BWeighting => [high, new(-995.8848711879644, 0, 1), low],
            AnalyzerWeighting.CWeighting => [high, low],
            // CCIR and Dolby 2K use the same prototype (APxRT 10013590).
            _ => [new(-25903.701161964338, -62675.17014007283, 1),
                  new(-18743.746620223184, -18743.746620223184, 0, 62460.15671371604),
                  new(-23615.535228758155, -23615.535228758155, 0, 36379.90898596716)]
        };
        var sections = new StreamingIirCascade.BiquadCoeffs[prototype.Length];
        double dt = 1d / rate;
        // 一个模拟衰减模式在一个采样间隔 dt 后乘以 exp(p·dt)。虚部使该模式在衰减的同时旋转。
        for (int i = 0; i < prototype.Length; i++)
        {
            var stage = prototype[i];
            if (stage.Imaginary != 0)
            {
                var pole = Complex.Exp(new Complex(stage.Pole1, stage.Imaginary) * dt);
                sections[i] = new(1, 0, 0, -2 * pole.Real, Power(pole));
                continue;
            }
            // 实极点逐个映射，第二极点用 0 标记缺省。分子在 z=1 的零点个数决定抑制直流的阶数。
            double pole1 = Math.Exp(stage.Pole1 * dt);
            double pole2 = stage.Pole2 == 0 ? 0 : Math.Exp(stage.Pole2 * dt);
            sections[i] = new(1, -stage.ZerosAtOrigin, stage.ZerosAtOrigin == 2 ? 1 : 0, -(pole1 + pole2), pole1 * pole2);
        }

        // 1000ad90 compares 101 logarithmic frequencies from 20 Hz through Nyquist.
        // The fixed sections supply the poles and DC zeros. The last numerator
        // is fitted after normalizing both responses at the curve's reference frequency.
        const int lastPoint = 100;
        const double anchor = 20 * Math.Tau;
        double ratio = Math.PI * rate / anchor;
        var target = new (Complex Z, Complex Z2, double AnalogPower, double DigitalPower)[lastPoint + 1];
        // 先让两条目标曲线各自在参考频率为 1，再比较从 20 Hz 到奈奎斯特的对数间隔频点。
        double referenceHz = weighting == AnalyzerWeighting.Ccir2k ? 2000 : 1000;
        var reference = Response(referenceHz * Math.Tau);
        for (int i = 0; i <= lastPoint; i++)
        {
            var point = Response(anchor * Math.Pow(ratio, i / (double)lastPoint));
            target[i] = (point.Z, point.Z2, point.AnalogPower / reference.AnalogPower, point.DigitalPower / reference.DigitalPower);
        }

        // 以频带上缘的偏差启动拟合，仅调末节分子的两项，保留已映射的各级极点。
        double edgeGain = Math.Sqrt(target[^1].AnalogPower / target[^1].DigitalPower);
        double seed = (1 - edgeGain) / 2;
        var fitted = Fit(seed, Score);
        sections[^1] = sections[^1] with { B1 = fitted.B1, B2 = fitted.B2 };
        double gain = 1;
        // 串联各节在参考频率的幅度相乘，其倒数作为总增益，使参考音通过后幅度不变。
        for (int i = 0; i < sections.Length; i++)
        {
            var c = sections[i];
            var numerator = c.B1 * reference.Z + Complex.One + c.B2 * reference.Z2;
            var denominator = c.A1 * reference.Z + Complex.One + c.A2 * reference.Z2;
            gain *= Math.Sqrt(Power(numerator) / Power(denominator));
        }
        return new(sections, 1 / gain);

        // 对一个候选分子，在所有目标频点计算相对误差的 dB 值，以最差频点的绝对偏差给它评分。
        double Score(double first, double second)
        {
            double inverseReference = 1 / Power(Complex.One + first * reference.Z + second * reference.Z2);
            double worst = 0;
            foreach (var point in target)
            {
                double numerator = Power(Complex.One + first * point.Z + second * point.Z2);
                double error = Math.Abs(10 * Math.Log10(point.DigitalPower * numerator * inverseReference / point.AnalogPower));
                worst = Math.Max(worst, error);
            }
            return worst;
        }

        // 给定角频率 rad/s，同时求模拟 H(jω) 和数字 H(exp(jωdt))。每一节的乘积给出整条链的响应。
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

    // 复响应的模平方是功率增益，实部与虚部的正负都不会造成负功率。
    private static double Power(Complex value) => value.Real * value.Real + value.Imaginary * value.Imaginary;

    // 在两个待定分子系数构成的平面上，用三个候选点寻找最小的最坏偏差。
    private static Candidate Fit(double seed, Func<double, double, double> score)
    {
        // APxRT 10011710 seeds a two-dimensional simplex with 5% steps.
        // 100115b0 stops it when the sample variance is below (1e-9 dB)^2.
        double step = seed == 0 ? .00025 : seed * 1.05;
        Candidate[] simplex = [At(seed, -seed), At(step, -seed), At(seed, seed == 0 ? .00025 : -step)];
        for (int i = 0; i < 5000; i++)
        {
            Array.Sort(simplex, static (left, right) => left.Error.CompareTo(right.Error));
            var best = simplex[0];
            var middle = simplex[1];
            var worst = simplex[2];
            // 比较三个误差的离散程度。达到停止阈值表示三个评分已很接近，无法据此证明全局最优。
            double d1 = middle.Error - best.Error, d2 = worst.Error - best.Error;
            double sum = d1 + d2;
            double variance = (d1 * d1 + d2 * d2 - sum * sum / 3) / 2;
            if (Math.Abs(variance) < 1e-9 * 1e-9) return best;

            // 用另外两点的中点作镜面，将最差点反射到另一侧，探测误差是否沿该方向减小。
            double mid1 = (best.B1 + middle.B1) / 2, mid2 = (best.B2 + middle.B2) / 2;
            Candidate reflected = At(2 * mid1 - worst.B1, 2 * mid2 - worst.B2);
            if (reflected.Error < best.Error)
            {
                // 反射已刷新最好成绩时再向外试一步，保留两次试探中误差较小的一点。
                Candidate expanded = At(mid1 + 2 * (mid1 - worst.B1), mid2 + 2 * (mid2 - worst.B2));
                simplex[2] = expanded.Error < reflected.Error ? expanded : reflected;
                continue;
            }
            // 反射只优于中间点时，直接替换最差点，再对新的三个点排序。
            if (reflected.Error < middle.Error)
            {
                simplex[2] = reflected;
                continue;
            }

            // 未找到足够好的反射点时向中点收缩。若仍无改进，让另两点都向最好点靠近。
            var edge = reflected.Error < worst.Error ? reflected : worst;
            Candidate contracted = At((edge.B1 + mid1) / 2, (edge.B2 + mid2) / 2);
            if (contracted.Error < edge.Error)
            {
                simplex[2] = contracted;
                continue;
            }
            simplex[1] = At((best.B1 + middle.B1) / 2, (best.B2 + middle.B2) / 2);
            simplex[2] = At((best.B1 + worst.B1) / 2, (best.B2 + worst.B2) / 2);
        }
        throw new InvalidOperationException("The realtime weighting design did not converge.");

        // 把系数与同一次计算得到的评分装在一起，保证排序与下次搜索针对相同候选。
        Candidate At(double b1, double b2) => new(b1, b2, score(b1, b2));
    }
}
