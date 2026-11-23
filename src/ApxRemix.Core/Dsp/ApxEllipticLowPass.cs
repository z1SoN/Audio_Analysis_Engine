using System.Numerics;
using static ApxRemix.Core.Dsp.StreamingIirCascade;

namespace ApxRemix.Core.Dsp;

internal static class ApxEllipticLowPass
{
    // The APxRT Analog AES17 setting uses the eighth-order elliptic prototype.
    // Each row is the upper pole and zero of one conjugate pair, in cascade order.
    private static readonly (Complex Pole, Complex Zero)[] Prototype =
    [
        (new(-0.050061961859382879, 1.0544010430691861), new(0, 1.2937938824069803)),
        (new(-0.17959150307597616, 0.98310522840240977), new(0, 1.4437942036807718)),
        (new(-0.38398300339739067, 0.77530916216000145), new(0, 1.9850289833081445)),
        (new(-0.60280069931460323, 0.31633913385572676), new(0, 5.2959231779150429))
    ];

    private const double DcGain = 0.99942451938;
    internal const double RelativeRetuneTolerance = 1e-5;

    // 以 Fs/2 为单位表示截止频率。限制上下端使后面的正切预畸变远离零和奇点。
    internal static double RelativeCutoff(double cutoffHz, int sampleRate) =>
        Math.Clamp(2.0 * cutoffHz / sampleRate, 1e-6, 0.99);

    // 按目标截止频率缩放原型的极点与零点，生成左右两套独立状态。下游可同时用相同曲线处理不同波形。
    internal static (StreamingIirCascade Left, StreamingIirCascade Right) CreateStereo(double relativeCutoff, int sampleRate)
    {
        // 双线性映射的频率刻度是非线性的，先用 tan(πfc/Fs) 预畸变，才能把设计点落回指定的数字频率。
        double prewarped = Math.Tan(Math.PI * (0.5 * sampleRate * relativeCutoff) / sampleRate);
        var sections = new BiquadCoeffs[Prototype.Length];
        double dc = 1.0;
        for (int i = 0; i < sections.Length; i++)
        {
            var (analogPole, analogZero) = Prototype[i];
            Complex pole = Bilinear(analogPole * prewarped);
            Complex zero = Bilinear(analogZero * prewarped);
            // 用根与其共轭构造实系数二阶节。零点形成阻带衰减，极点同时决定选择性与暂态。
            double b1 = -2.0 * zero.Real;
            double b2 = SquaredMagnitude(zero);
            double a1 = -2.0 * pole.Real;
            double a2 = SquaredMagnitude(pole);
            sections[i] = new BiquadCoeffs(1.0, b1, b2, a1, a2);
            // 直流处每个单位延迟都等于 1，故把三项直接相加就得到该节在直流处的增益比。
            dc *= (b1 + 1.0 + b2) / (a1 + 1.0 + a2);
        }

        double gain = DcGain / dc;
        return (new(sections, gain, useApxStateOrder: true), new(sections, gain, useApxStateOrder: true));
    }

    // 双线性映射把衰减的模拟极点映入单位圆，得到离散递推时幅度逐步衰减的条件。
    private static Complex Bilinear(Complex root) => (Complex.One + root) / (Complex.One - root);

    // 共轭根乘积等于实部平方加虚部平方，正是二次多项式的常数项。
    private static double SquaredMagnitude(Complex value) =>
        value.Real * value.Real + value.Imaginary * value.Imaginary;
}
