using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace ApxRemix.Core.Dsp;

// 对齐 Sequence 的高精度复模。复模先舍入为 double，再平方为功率，顺序可能影响陷波频格。
internal static partial class SequenceMagnitude
{
    // 掩码保留高位以拆开平方误差；SplitConstant 用于算术拆位，查表种子用来启动倒平方根迭代。
    private const ulong HighMask = 0xffffc00000000000;
    private const double SplitConstant = 134217729;
    private static readonly double[] ReciprocalSqrtSeeds = CreateSeeds();

    // 实部与虚部是两条正交投影，其模长为 √(a²+b²)，供下游平方求功率。
    internal static double Compute(double real, double imaginary)
    {
        // 先识别特殊数，再对绝对值排序，后续可统一按 a 的数量级选择计算路径。
        if (TrySpecialMagnitude(real, imaginary, out double special)) return special;
        double a = Math.Abs(real), b = Math.Abs(imaginary);
        if (b > a) (a, b) = (b, a);

        // 将平方和分为主要部分和小补项；相距多个指数量级时把小分量交给补项处理。
        double aHigh = Value(Bits(a) & HighMask);
        double bHigh = (Bits(a) >> 32) - (Bits(b) >> 32) > 0x00600000 ? 0 : Value(Bits(b) & HighMask);
        (double high, double low) = SplitNorm(a, b, aHigh, bHigh);
        double sum = high + low;
        ulong head = Bits(sum) >> 32;
        // 根号内量太大、太小或缺少 SSE 时改用缩放路径，避免向单精度近似传入不适合的范围。
        if (head < 0x3bc00000 || head > 0x44100000 || !Sse.IsSupported)
            return Scaled(a, b);

        // 在本次核验的 x86/x64 CPU 上，标量和向量 SSE 取得同一近似倒根。
        // 无 SSE 的平台用缩放算法，避免依赖特定 CPU 的种子位模式。
        // 单精度硬件给出 1/√sum 的种子，用 high 与 low 都参与的 sum·r²−1 评价误差。
        double reciprocal = Sse.ReciprocalSqrtScalar(Vector128.CreateScalar((float)sum)).ToScalar();
        double reciprocalSquared = reciprocal * reciprocal;
        double error = (high * reciprocalSquared - 1) + low * reciprocalSquared;
        // 按 (1 + error)^(-1/2) 的前五个非恒定二项式系数补偿种子。
        // 逐次乘加评估误差多项式，最后乘回平方和，恢复到模长单位。
        double correction = -63.0 / 256 * error + 35.0 / 128;
        correction = correction * error - 5.0 / 16;
        correction = correction * error + 3.0 / 8;
        correction = correction * error - 0.5;
        correction = correction * error * reciprocal;
        return ((sum * correction) + low * reciprocal) + high * reciprocal;
    }

    // 无穷优先，其次为 NaN，并保留其载荷而设置 quiet 位；两个零直接返回零。
    private static bool TrySpecialMagnitude(double real, double imaginary, out double result)
    {
        if (double.IsInfinity(real) || double.IsInfinity(imaginary))
        {
            result = double.PositiveInfinity;
            return true;
        }
        if (double.IsNaN(real) || double.IsNaN(imaginary))
        {
            double source = double.IsNaN(real) ? real : imaginary;
            result = Value(Bits(source) | 0x0008000000000000);
            return true;
        }
        result = 0;
        return real == 0 && imaginary == 0;
    }

    // 放大或缩小相同二次幂，使后续平方和落在合适的数量级；末尾撤销输入的缩放。
    private static double Scaled(double a, double b)
    {
        int scaleExponent = 1025 - Exponent(a);
        if (scaleExponent > 1000) scaleExponent -= 23;
        double scale = PowerOfTwo(scaleExponent);
        a *= scale;
        b *= scale;
        // 缩放后把平方拆成高位和误差，数值大小落在乘法不会溢出的区间。
        // 以高低部分保存缩放后的平方和，保留后面可用来校正近似根的余量。
        double aHigh = Split(a);
        double bHigh = Exponent(a) - Exponent(b) > 6 ? 0 : Split(b);
        (double high, double low) = SplitNorm(a, b, aHigh, bHigh);
        double sum = high + low;

        // 平方和指数拆成偶数部分与奇偶余数；标准化后有效数在 [1,4)，按高位及奇偶性选种子。
        int exponent = Exponent(sum) - 1023;
        int parity = exponent & 1;
        int halfExponent = (exponent - parity) / 2;
        int tableIndex = (int)((Bits(sum) >> 44) & 255) + 256 * parity;
        double normalized = Value((Bits(sum) & 0x000fffffffffffff) | 0x3ff0000000000000) * (parity == 0 ? 1 : 2);
        double seed = ReciprocalSqrtSeeds[tableIndex];
        double root = normalized * seed;
        double halfInverse = 0.5 * seed;
        // root 和 halfInverse 的乘积应为 1/2，三轮用同一个误差细化根及半倒数。
        for (int i = 0; i < 3; i++)
        {
            double error = 0.5 - root * halfInverse;
            root += root * error;
            halfInverse += halfInverse * error;
        }

        // 用拆分平方的残差修正最后一次平方根舍入。
        // 拆分近似根后求“输入减根平方”的残差，乘半倒数修正根，再还原两次指数缩放。
        double rootHigh = Split(root);
        double rootLow = root - rootHigh;
        double correctionScale = PowerOfTwo(-2 * halfExponent);
        double residual = high * correctionScale - rootHigh * rootHigh;
        residual -= (2 * rootHigh) * rootLow;
        residual -= rootLow * rootLow;
        residual += low * correctionScale;
        root += residual * halfInverse;
        return (root * PowerOfTwo(halfExponent)) * PowerOfTwo(-scaleExponent);
    }

    // 利用 a²−aHigh²=(a−aHigh)(a+aHigh)，把主平方之外的交叉项汇入 low。
    private static (double High, double Low) SplitNorm(double a, double b, double aHigh, double bHigh)
    {
        double aLow = a - aHigh, bLow = b - bHigh;
        double high = aHigh * aHigh + bHigh * bHigh;
        double low = (aLow * (a + aHigh) + bLow * b) + bLow * bHigh;
        return (high, low);
    }

    // 乘以 2²⁷+1 后相减，通过 double 的舍入得到原数的高位部分。
    private static double Split(double value)
    {
        double expanded = SplitConstant * value;
        return expanded - (expanded - value);
    }

    // 两组各 256 格覆盖 [1,2) 和 [2,4)，每格左边界的倒平方根量化到 1/2048。
    private static double[] CreateSeeds()
    {
        // [1, 2) 与 [2, 4) 各 256 格，取左端倒根，舍入到步长 1/2048。
        var values = new double[512];
        for (int i = 0; i < values.Length; i++)
        {
            double lower = (1 + (i % 256) / 256.0) * (i < 256 ? 1 : 2);
            values[i] = Math.Round(2048 / Math.Sqrt(lower)) / 2048;
        }
        return values;
    }

    // 读取含偏置的 binary64 指数字段，供比较数量级与计算移位使用。
    private static int Exponent(double value) => (int)((Bits(value) >> 52) & 2047);
    // 只设置指数、清空小数位，构造所需范围内的精确二次幂。
    private static double PowerOfTwo(int exponent) => Value((ulong)(1023 + exponent) << 52);
    // 将 double 视作原始 64 位，用于掩码和指数判断。
    private static ulong Bits(double value) => BitConverter.DoubleToUInt64Bits(value);
    // 将位串恢复为 double，保留编辑后的指数和有效数。
    private static double Value(ulong bits) => BitConverter.UInt64BitsToDouble(bits);
}
