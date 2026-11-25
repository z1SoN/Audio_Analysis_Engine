using System.Numerics;
using static ApxRemix.Core.Dsp.StreamingIirCascade;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APxRT 10.0.0.17 椭圆高通/低通滤波器（8 阶椭圆低通 AES17/Elliptic + 5 阶椭圆高通）。
/// </summary>
internal static class ApxEllipticLowPass
{
    // The APxRT Analog AES17 / Digital Elliptic LP uses the eighth-order 0.01 dB ripple elliptic prototype.
    private static readonly (Complex Pole, Complex Zero)[] LowpassPrototype =
    [
        (new(-0.050061961859382879, 1.0544010430691861), new(0, 1.2937938824069803)),
        (new(-0.17959150307597616, 0.98310522840240977), new(0, 1.4437942036807718)),
        (new(-0.38398300339739067, 0.77530916216000145), new(0, 1.9850289833081445)),
        (new(-0.60280069931460323, 0.31633913385572676), new(0, 5.2959231779150429))
    ];

    // The APxRT Elliptic HP (10006120, order 5, 0.01 dB ripple, 60 dB stopband) prototype.
    private static readonly (Complex Pole, Complex Zero)[] HighpassPrototype =
    [
        (new(-0.21719653110351217, 1.2113624052523047), new(0, 2.633492586928729)),
        (new(-0.6502780720097704, 0.8155247260643821), new(0, 4.161410779614699))
    ];

    private const double HighpassRealPole = -0.878212056512607;
    private const double LowpassDcGain = 0.99942451938;
    private const double HighpassNyquistGain = 1.00057581198936;
    internal const double RelativeRetuneTolerance = 1e-5;

    // 以 Fs/2 为单位表示截止频率。限制上下端使后面的正切预畸变远离零和奇点。
    internal static double RelativeCutoff(double cutoffHz, int sampleRate) =>
        Math.Clamp(2.0 * cutoffHz / sampleRate, 1e-6, 0.99);

    // 按目标截止频率缩放 8 阶椭圆低通原型的极点与零点，生成左右两套独立状态。
    internal static (StreamingIirCascade Left, StreamingIirCascade Right) CreateStereo(double relativeCutoff, int sampleRate)
    {
        double prewarped = Math.Tan(Math.PI * (0.5 * sampleRate * relativeCutoff) / sampleRate);
        var sections = new BiquadCoeffs[LowpassPrototype.Length];
        double dc = 1.0;
        for (int i = 0; i < sections.Length; i++)
        {
            var (analogPole, analogZero) = LowpassPrototype[i];
            Complex pole = Bilinear(analogPole * prewarped);
            Complex zero = Bilinear(analogZero * prewarped);
            double b1 = -2.0 * zero.Real;
            double b2 = SquaredMagnitude(zero);
            double a1 = -2.0 * pole.Real;
            double a2 = SquaredMagnitude(pole);
            sections[i] = new BiquadCoeffs(1.0, b1, b2, a1, a2);
            dc *= (b1 + 1.0 + b2) / (a1 + 1.0 + a2);
        }

        double gain = LowpassDcGain / dc;
        return (new(sections, gain, useApxStateOrder: true), new(sections, gain, useApxStateOrder: true));
    }

    // APxRT 10006120 5 阶椭圆高通（2 个共轭零极点二阶节 + 1 个实极点直流零点一阶节，Nyquist 处归一化为 1.00057581198936）。
    internal static (StreamingIirCascade Left, StreamingIirCascade Right) CreateEllipticHighPassStereo(double cutoffHz, int sampleRate)
    {
        double rel = RelativeCutoff(cutoffHz, sampleRate);
        double prewarped = Math.Tan(Math.PI * (0.5 * sampleRate * rel) / sampleRate);
        var sections = new BiquadCoeffs[3];
        double nyquist = 1.0;
        for (int i = 0; i < HighpassPrototype.Length; i++)
        {
            var (analogPole, analogZero) = HighpassPrototype[i];
            Complex pole = Bilinear(prewarped / analogPole);
            Complex zero = Bilinear(prewarped / analogZero);
            double b1 = -2.0 * zero.Real;
            double b2 = SquaredMagnitude(zero);
            double a1 = -2.0 * pole.Real;
            double a2 = SquaredMagnitude(pole);
            sections[i] = new BiquadCoeffs(1.0, b1, b2, a1, a2);
            nyquist *= (1.0 - b1 + b2) / (1.0 - a1 + a2);
        }

        double realPole = Bilinear(new Complex(prewarped / HighpassRealPole, 0)).Real;
        sections[2] = new BiquadCoeffs(1.0, -1.0, 0.0, -realPole, 0.0);
        nyquist *= 2.0 / (1.0 + realPole);

        double gain = HighpassNyquistGain / nyquist;
        return (new(sections, gain, useApxStateOrder: true), new(sections, gain, useApxStateOrder: true));
    }

    private static Complex Bilinear(Complex root) => (Complex.One + root) / (Complex.One - root);

    private static double SquaredMagnitude(Complex value) =>
        value.Real * value.Real + value.Imaginary * value.Imaginary;
}
