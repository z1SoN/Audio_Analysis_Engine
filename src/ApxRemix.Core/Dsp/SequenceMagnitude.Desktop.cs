using System.Numerics;

namespace ApxRemix.Core.Dsp;

internal static partial class SequenceMagnitude
{
    // 桌面配置在本地用 Extended 的 64 位有效数计算模长，特殊数沿用共同规则。
    internal static double ComputeDesktop(double real, double imaginary)
    {
        if (TrySpecialMagnitude(real, imaginary, out double special)) return special;

        // 桌面 x64 路径每一步按 x87 的 64 位有效数舍入，最后才写回 binary64。
        // 两分量平方求和后抽出偶数指数，余下的 [1,4) 范围用有效数高位和奇偶性查倒平方根种子。
        var a = Extended.FromDouble(real);
        var b = Extended.FromDouble(imaginary);
        var sum = a * a + b * b;
        int parity = sum.Exponent & 1;
        int halfExponent = (sum.Exponent - parity) / 2;
        int index = (int)((sum.Significand >> 55) & 255) + 256 * parity;
        var normalized = sum with { Exponent = parity };
        var seed = Extended.FromDouble(ReciprocalSqrtSeeds[index]);
        var half = Extended.FromDouble(0.5);
        var root = normalized * seed;
        var halfInverse = half * seed;
        // root·halfInverse 理想值是 1/2，三轮迭代同时修正根及半倒数。
        for (int i = 0; i < 3; i++)
        {
            var error = half - root * halfInverse;
            root += root * error;
            halfInverse += halfInverse * error;
        }
        // 保留桌面路径的残差计算顺序。先减低部平方，再加高部乘积。
        // 拆分近似根以计算平方误差，加上残差校正后才恢复指数、转为 double。
        var expanded = Extended.FromDouble(SplitConstant) * root;
        var high = expanded - (expanded - root);
        var low = root - high;
        var residual = (normalized - low * low) + high * (default(Extended) - high - low.Scale(1));
        return (root + residual * halfInverse).Scale(halfExponent).ToDouble();
    }

    // 用无符号整数保留 binary80 的 64 位有效数。平方后的 binary64 输入仍在其正规指数范围内。
    // 有效数最高位置于第 63 位，Exponent 标记这一位的二次幂，符号单独保存。
    private readonly record struct Extended(ulong Significand, int Exponent, bool Negative)
    {
        private const ulong LeadingBit = 1UL << 63;
        private const ulong FractionMask = (1UL << 52) - 1;

        // 从 binary64 分离指数、有效位和符号；普通数补上隐含首位，零直接用默认表示。
        internal static Extended FromDouble(double value)
        {
            ulong bits = BitConverter.DoubleToUInt64Bits(value);
            int exponent = (int)((bits >> 52) & 2047);
            ulong fraction = bits & FractionMask;
            bool negative = (bits & LeadingBit) != 0;
            if (exponent != 0)
                return new((fraction | (1UL << 52)) << 11, exponent - 1023, negative);
            if (fraction == 0)
                return default;

            // 次正规数没有隐含首位，找到其最高位并左移，同时降低指数以保持数值。
            int leftShift = BitOperations.LeadingZeroCount(fraction);
            return new(fraction << leftShift, -1011 - leftShift, negative);
        }

        // 乘以二的整数次幂只改变指数。
        internal Extended Scale(int exponent) => this with { Exponent = Exponent + exponent };

        // 两个 64 位有效数先完整乘入 128 位，再按目标精度舍入。
        public static Extended operator *(Extended a, Extended b)
            => Quantize((UInt128)a.Significand * b.Significand, a.Exponent + b.Exponent - 63, a.Negative != b.Negative);

        // 用第二个操作数反号后的加法实现减法，共享有效位对齐步骤。
        public static Extended operator -(Extended a, Extended b) => a + (b with { Negative = !b.Negative });

        // 先处理零，再把绝对值较大者放在 a，以它决定结果符号。
        public static Extended operator +(Extended a, Extended b)
        {
            if (a.Significand == 0) return b;
            if (b.Significand == 0) return a;
            if (a.Exponent < b.Exponent || a.Exponent == b.Exponent && a.Significand < b.Significand)
                (a, b) = (b, a);

            // 指数差决定对齐所需位数。超过有效数宽度时小数通常不改变舍入结果。
            int shift = a.Exponent - b.Exponent;
            bool sameSign = a.Negative == b.Negative;
            if (shift > 64)
            {
                // 二次幂下侧的相邻值间距只有当前间距的一半。
                // 二的幂朝较小值一侧的相邻间距较短，下面单独处理异号相减跨到较低指数组的边界。
                if (!sameSign && shift == 65 && a.Significand == LeadingBit && b.Significand > LeadingBit)
                    return new(ulong.MaxValue, a.Exponent - 1, a.Negative);
                return a;
            }
            // 指数差尚可容纳时，用宽整数精确对齐，再相加或相减并量化。
            UInt128 large = (UInt128)a.Significand << shift;
            UInt128 combined = sameSign ? large + b.Significand : large - b.Significand;
            return Quantize(combined, b.Exponent, a.Negative);
        }

        // 写回 binary64，先处理带符号的零及次正规数，再把普通数舍入到 53 位有效精度。
        internal double ToDouble()
        {
            ulong sign = Negative ? LeadingBit : 0;
            if (Significand == 0 || Exponent < -1075)
                return BitConverter.UInt64BitsToDouble(sign);
            if (Exponent < -1022)
                return BitConverter.UInt64BitsToDouble(sign | (ulong)RoundRight(Significand, -1011 - Exponent));

            int exponent = Exponent;
            ulong significand = (ulong)RoundRight(Significand, 11);
            // 舍入可能从最高有效位进位到指数；超过 double 上限时编码为有符号无穷。
            if (significand == 1UL << 53)
            {
                significand >>= 1;
                exponent++;
            }
            return exponent > 1023
                ? BitConverter.UInt64BitsToDouble(sign | 0x7ff0000000000000)
                : BitConverter.UInt64BitsToDouble(sign | ((ulong)(exponent + 1023) << 52) | (significand & FractionMask));
        }

        // 把宽整数的最高位对准 64 位有效数的首位，左右移多少位就在指数上补回多少位。
        private static Extended Quantize(UInt128 value, int exponent, bool negative)
        {
            if (value == 0) return default;
            ulong top = (ulong)(value >> 64);
            int shift = top == 0 ? -BitOperations.LeadingZeroCount((ulong)value) : 64 - BitOperations.LeadingZeroCount(top);
            UInt128 significand = shift < 0 ? value << -shift : RoundRight(value, shift);
            exponent += shift;
            // 右移舍入可能令有效数多出一位，此时再移一位并让指数进一。
            if (significand >> 64 != 0)
                return new((ulong)(significand >> 1), exponent + 1, negative);
            return new((ulong)significand, exponent, negative);
        }

        // 比较舍弃部分与半个单位；当两边等距时，让最后保留的二进制位为偶数。
        private static UInt128 RoundRight(UInt128 value, int shift)
        {
            if (shift == 0) return value;
            UInt128 whole = value >> shift;
            UInt128 fraction = value - (whole << shift);
            UInt128 half = (UInt128)1 << (shift - 1);
            return fraction > half || fraction == half && (whole & 1) != 0 ? whole + 1 : whole;
        }
    }
}
