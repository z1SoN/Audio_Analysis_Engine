using System.Globalization;
using System.Numerics;
using static ApxRemix.Core.Dsp.SequenceX87FixedPoint;

namespace ApxRemix.Core.Dsp;

internal static class SequenceX86ToneMath
{
    // Pi 在 160 位小数的定点尺度上保存；用于改换正弦参考的 π/2 则来自 binary64 的 Math.PI。
    private static readonly BigInteger Pi = BigInteger.Parse("3243F6A8885A308D313198A2E03707344A4093822", NumberStyles.HexNumber, CultureInfo.InvariantCulture);
    private static readonly BigInteger Binary64HalfPi = FromDouble(Math.PI / 2);

    // 由复频格方向推算正弦起始相位；无穷或 NaN 走标准 atan2 保留运行时的边界行为。
    internal static double BinPhase(double imaginary, double real, double fractionalBin)
    {
        if (!double.IsFinite(imaginary) || !double.IsFinite(real))
            return Math.Atan2(imaginary, real) + Math.PI / 2 - Math.PI * fractionalBin;

        // The x86 worksheet adds the constant to the 64-significand-bit FPATAN
        // result. The x87 then rounds the addition and subtraction to 53 bits.
        // 普通值在定点域算象限角，加 π/2 后舍入到 53 位，再扣分数频格对应的 π 倍偏移。
        var angle = Atan2(imaginary, real);
        double position = Math.ScaleB((double)RoundToSignificand(angle + Binary64HalfPi, 53), -FractionBits);
        return position - Math.PI * fractionalBin;
    }

    // 由有效值和帧内周期数恢复时域基波，用每个采样点的同相位正弦去扣原信号。
    internal static void SubtractSine(Span<double> samples, double rms, double phase, double cycles)
    {
        double amplitude = rms * Math.Sqrt(2);
        double step = cycles * 2 * Math.PI / samples.Length;
        for (int i = 0; i < samples.Length; i++, phase += step)
        {
            // 相位极小或操作数非有限时走 Math.Sin；普通角度使用定点正弦与幅值相乘，再从样本扣除。
            if (!double.IsFinite(amplitude) || !double.IsFinite(phase) || Math.Abs(phase) < 1.1102230246251565e-16)
            {
                samples[i] -= Math.Sin(phase) * amplitude;
                continue;
            }

            // Keep the estimated 64-bit FSIN significand through the multiply.
            // The rounded binary64 product feeds the separate subtraction.
            samples[i] -= Multiply(Sine(phase), amplitude);
        }
    }

    // 用两投影中较小者除以较大者，使反正切的自变量在 [0,1]。
    private static BigInteger Atan2(double imaginary, double real)
    {
        double y = Math.Abs(imaginary), x = Math.Abs(real);
        var ratio = Ratio(Math.Min(x, y), Math.Max(x, y));
        // 比值大于一半时移到 π/4 附近求余角，使级数的自变量绝对值变小。
        bool aroundQuarterPi = ratio > (One >> 1);
        if (aroundQuarterPi) ratio = ((ratio - One) << FractionBits) / (ratio + One);

        // |ratio| <= 1/2, so the remaining series after term 80 is below 2^-160.
        // 递推 r−r³/3+r⁵/5…；最多八十轮，当下一项低于定点分辨率变成零时也可结束。
        var square = (ratio * ratio) >> FractionBits;
        var term = ratio;
        var radians = term;
        for (int i = 1; i <= 80 && !term.IsZero; i++)
        {
            term = -((term * square) / One);
            radians += term / (2 * i + 1);
        }

        // 先撤销 π/4 附近的换元，再用两分量的大小与符号还原象限，最后保留 64 位有效数。
        if (aroundQuarterPi) radians += Pi >> 2;
        if (y > x) radians = (Pi >> 1) - radians;
        if (double.IsNegative(real)) radians = Pi - radians;
        if (double.IsNegative(imaginary)) radians = -radians;
        return RoundToSignificand(radians, 64);
    }

    // 把两个正数分别拆成整数有效数和指数，通过移位形成同一 160 位小数比例后做整数除法。
    private static BigInteger Ratio(double numerator, double denominator)
    {
        if (numerator == 0) return BigInteger.Zero;
        var (n, numeratorExponent) = Decompose(numerator);
        var (d, denominatorExponent) = Decompose(denominator);
        int shift = FractionBits + numeratorExponent - denominatorExponent;
        return shift < 0 ? BigInteger.Zero : ((BigInteger)n << shift) / d;
    }

}
