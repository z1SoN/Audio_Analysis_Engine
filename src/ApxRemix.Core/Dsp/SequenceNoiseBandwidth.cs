using System.Numerics;

namespace ApxRemix.Core.Dsp;

/// <summary>The local elliptic filters documented for the Sequence Noise measurement.</summary>
public static class SequenceNoiseBandwidth
{
    // 两个默认带边的单位是 Hz，开关决定是否把相应滤波级放进流式通道。
    public const double DefaultHighpassHz = 20;
    public const double DefaultLowpassHz = 20_000;

    // Upper-half roots of the standard 0.01 dB ripple, 60 dB stopband analog prototypes.
    // See tests/artifacts/measurement-validation/sequence/noise-elliptic-waveform-probe.csv
    // for the independent PCM comparison against the installed APx500 10.0.0.17 processing library.
    // 高通原型每行是一组共轭根的参数，另一个实极点在创建时单独加入。
    private static readonly (double PoleReal, double PoleImaginary, double ZeroImaginary)[] HighpassPrototype =
    [
        (-0.6502780720097704, 0.8155247260643821, 4.161410779614699),
        (-0.21719653110351217, 1.2113624052523047, 2.633492586928729),
    ];

    // 低通四组共轭根，各组展开为一个实系数二阶滤波节。
    private static readonly (double PoleReal, double PoleImaginary, double ZeroImaginary)[] LowpassPrototype =
    [
        (-0.6028006988734368, 0.3163391334692356, 5.2959231865674745),
        (-0.38398300352107634, 0.7753091616653464, 1.9850289861191739),
        (-0.17959150334126917, 0.9831052282665822, 1.4437942053524004),
        (-0.050061961968267905, 1.05440104316347, 1.293793883705017),
    ];

    // 把模拟原型按采样率映到数字滤波器。两个开关都关闭时只有单位增益。
    public static StreamingIirCascade Create(int sampleRate, bool highpass, bool lowpass)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(sampleRate);
        var sections = new List<StreamingIirCascade.BiquadCoeffs>(7);
        double gain = 1;
        if (highpass)
        {
            // 高通先放一个实极点的一阶节，再按 k/原型根映射共轭对，在高频端 z=−1 归一化。
            double k = Prewarp(DefaultHighpassHz, sampleRate);
            var pole = ToDigital(k / -0.878212056512607).Real;
            double scale = (1 + pole) / 2;
            sections.Add(new(scale, -scale, 0, -pole, 0));
            foreach (var (real, imaginary, zero) in HighpassPrototype)
                sections.Add(Section(ToDigital(k / new Complex(real, imaginary)), ToDigital(k / new Complex(0, zero)), -1));
            // 由幅值分贝换算通带整体倍率；连续接入低通时两级倍率会相乘。
            gain = Math.Pow(10, 0.005 / 20);
        }
        if (lowpass)
        {
            // 低通用 k·原型根缩放截止频率，在 z=1 的直流端归一化。
            double k = Prewarp(DefaultLowpassHz, sampleRate);
            foreach (var (real, imaginary, zero) in LowpassPrototype)
                sections.Add(Section(ToDigital(k * new Complex(real, imaginary)), ToDigital(new Complex(0, k * zero)), 1));
            gain *= Math.Pow(10, -0.005 / 20);
        }
        return new(sections.ToArray(), gain);
    }

    // 双线性变换前用正切预畸变截止频率，将归一化频率夹到 0.495 以下或该值，避免在 π/2 发散。
    private static double Prewarp(double cutoffHz, int sampleRate) =>
        Math.Tan(Math.PI * Math.Min(cutoffHz / sampleRate, 0.495));

    // 将归一化的模拟平面根映射到数字 z 平面。
    private static Complex ToDigital(Complex root) => (1 + root) / (1 - root);

    // 共轭根展开得到实系数二次式；比较指定端点的分子分母，设定该端的单位增益。
    private static StreamingIirCascade.BiquadCoeffs Section(Complex pole, Complex zero, int normalizeAt)
    {
        double a1 = -2 * pole.Real, a2 = pole.Magnitude * pole.Magnitude;
        double b1 = -2 * zero.Real;
        double scale = (1 + normalizeAt * a1 + a2) / (2 + normalizeAt * b1);
        return new(scale, b1 * scale, scale, a1, a2);
    }
}
