using System.Numerics;

namespace ApxRemix.Core.Dsp;

internal static class SequenceX87FixedPoint
{
    // 所有定点整数隐含除数 2¹⁶⁰，例如 One 代表数学上的 1。它提供多步运算时所需的额外小数位。
    internal const int FractionBits = 160;
    internal static readonly BigInteger One = BigInteger.One << FractionBits;
    // Intel SDM Vol. 1 §8.3.8. The x87 reduction constant has 66 significant bits.
    // 半圆周率用于象限折叠，正弦和余弦各自预先生成系数表，供重复求值。
    private static readonly BigInteger HalfPi = (((BigInteger)0xc90fdaa22168c234UL << 4) + 12) << (FractionBits - 67);
    private static readonly BigInteger[] CosineTerms = TaylorTerms(sine: false), SineTerms = TaylorTerms(sine: true);

    // 角度单位为弧度，返回有 160 位小数刻度的余弦值。
    internal static BigInteger Cosine(double angle) => Trig(angle, sine: false);
    // 正弦与余弦共用角度规约，只切换哪套多项式和符号规则。
    internal static BigInteger Sine(double angle) => Trig(angle, sine: true);

    // 找到最接近的 π/2 整数倍，把余角移到约 ±π/4 内，使有限项近似容易收敛。
    private static BigInteger Trig(double angle, bool sine)
    {
        var argument = FromDouble(Math.Abs(angle));
        var quadrant = (argument + (HalfPi >> 1)) / HalfPi;
        var reduced = argument - quadrant * HalfPi;
        int sector = (int)(quadrant & 3);
        // 象限奇偶决定在小区间内算 sin 还是 cos；二者都能用余角平方作多项式的自变量。
        bool sinePolynomial = (sector & 1) == (sine ? 0 : 1);
        var terms = sinePolynomial ? SineTerms : CosineTerms;
        var square = (reduced * reduced) >> FractionBits;
        var value = terms[^1];
        // 逐级乘加，每次右移 FractionBits 回到相同刻度。正弦为奇函数，最后多乘一次余角。
        for (int i = terms.Length - 2; i >= 0; i--)
            value = terms[i] + ((square * value) >> FractionBits);
        if (sinePolynomial) value = (value * reduced) >> FractionBits;
        // 结合原角度及象限还原符号，把定点结果舍到 64 位有效精度。
        bool negate = sine ? ((sector & 2) != 0) != (angle < 0) : sector is 1 or 2;
        return RoundToSignificand(negate ? -value : value, 64);
    }

    // 将定点值与 binary64 权重相乘，先保留宽整数乘积，再按 53 位舍入并还原比例和符号。
    internal static double Multiply(BigInteger value, double factor)
    {
        var (significand, exponent) = Decompose(Math.Abs(factor));
        var product = value * significand;
        int shift = Math.Max(0, (int)BigInteger.Abs(product).GetBitLength() - 53);
        double rounded = Math.ScaleB((double)RoundRight(product, shift), exponent - FractionBits + shift);
        return double.IsNegative(factor) ? -rounded : rounded;
    }

    // 将 binary64 拆成整数有效数与二次幂，再移动到统一小数位；过小到超出定点分辨率的位会丢掉。
    internal static BigInteger FromDouble(double value)
    {
        var (significand, exponent) = Decompose(Math.Abs(value));
        BigInteger fixedPoint = (BigInteger)significand << (FractionBits + exponent);
        return value < 0 ? -fixedPoint : fixedPoint;
    }

    // 普通数还原隐含首位，次正规数没有隐含首位；两种情况均返回“整数×2 的指数次方”。
    internal static (ulong Significand, int Exponent) Decompose(double value)
    {
        ulong bits = BitConverter.DoubleToUInt64Bits(value);
        int exponent = (int)(bits >> 52);
        ulong fraction = bits & 0x000f_ffff_ffff_ffffUL;
        return exponent == 0 ? (fraction, -1074) : ((1UL << 52) | fraction, exponent - 1075);
    }

    // 仅丢掉超出指定位宽的低位；舍入后再左移，让定点刻度仍是 2⁻¹⁶⁰。
    internal static BigInteger RoundToSignificand(BigInteger value, int bits)
    {
        int shift = Math.Max(0, (int)BigInteger.Abs(value).GetBitLength() - bits);
        return RoundRight(value, shift) << shift;
    }

    // 比较低位余数和半格，恰好等距选保留部分为偶数的一侧，最后放回原符号。
    private static BigInteger RoundRight(BigInteger value, int shift)
    {
        if (shift == 0) return value;
        BigInteger absolute = BigInteger.Abs(value);
        BigInteger upper = absolute >> shift;
        BigInteger lower = absolute - (upper << shift);
        BigInteger half = BigInteger.One << (shift - 1);
        if (lower > half || lower == half && !upper.IsEven) upper++;
        return value.Sign < 0 ? -upper : upper;
    }

    // 按正弦的奇次阶或余弦的偶次阶生成 21 项，系数按 ±1/阶乘 换到共同的定点尺度。
    private static BigInteger[] TaylorTerms(bool sine)
    {
        // |reduced| <= pi/4. The first omitted term is below 2^-170.
        var terms = new BigInteger[21];
        BigInteger factorial = 1;
        for (int i = 0; i < terms.Length; i++)
        {
            int degree = 2 * i + (sine ? 1 : 0);
            if (i > 0) factorial *= (degree - 1) * degree;
            terms[i] = ((i & 1) == 0 ? One : -One) / factorial;
        }
        return terms;
    }
}
