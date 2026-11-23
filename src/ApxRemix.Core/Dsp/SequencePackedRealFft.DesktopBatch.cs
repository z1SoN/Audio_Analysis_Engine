using System.Numerics;

namespace ApxRemix.Core.Dsp;

internal static partial class SequencePackedRealFft
{
    // 仅列出有专门桌面分解次序的采集长度；其他点数由上层另外派发。
    internal static bool SupportsDesktopBatch(int length) => length is 8064 or 11200 or 12000 or 24000;

    // 从 N 个实样本生成含 N/2+1 个复格的单边谱，先用该长度专属的混合基分解。
    internal static Complex[] ForwardDesktopBatch(ReadOnlySpan<double> input)
    {
        int length = input.Length;
        // 四种长度分别列出从外层到内层的分支数，其余点数留给八点或十六点叶节点。
        ReadOnlySpan<int> radices = length switch
        {
            8064 => [7, 4, 3, 3],
            11200 => [7, 4, 5, 5],
            12000 => [5, 3, 5, 5],
            24000 => [5, 4, 3, 5, 5],
            _ => throw new ArgumentOutOfRangeException(nameof(input))
        };
        // 用整段长度的相位网格，邻近两实点打包成一个复样本，先求 N/2 点变换。
        var phaseGrid = DesktopBatchPhaseGrids.Get(length);
        var packed = new Complex[length / 2];
        ForwardDesktopBatchStages(input, 0, 2, packed, radices, phaseGrid, length);

        // 直流与奈奎斯特是偶、奇点和的和、差。N/4 格只需共轭，内部格则成对解包。
        int half = length / 2, quarter = length / 4;
        var result = new Complex[half + 1];
        result[0] = new(packed[0].Real + packed[0].Imaginary, 0);
        result[half] = new(packed[0].Real - packed[0].Imaginary, 0);
        result[quarter] = Complex.Conjugate(packed[quarter]);
        for (int k = 1; k < quarter; k++)
        {
            // 将 k 与 N/2−k 的复数分量分离，转过一个采样的相角后同时求出这对正频格。
            int mirror = half - k;
            Complex left = packed[k], right = packed[mirror];
            double u = left.Real - right.Real, v = left.Imaginary + right.Imaginary;
            Complex root = BatchRoot(phaseGrid, length, k);
            double e = 0.5 * root.Real, f = 0.5 * (1 + root.Imaginary);
            double real = e * v + f * u, imaginary = f * v - e * u;
            result[k] = new(right.Real + real, imaginary - right.Imaginary);
            result[mirror] = new(left.Real - real, imaginary - left.Imaginary);
        }
        return result;
    }

    // 递归每深入一层就使用下一项 radix；叶节点按剩下的点数选择局部蝶形。
    private static void ForwardDesktopBatchStages(ReadOnlySpan<double> input, int offset, int stride,
        Span<Complex> output, ReadOnlySpan<int> radices, ReadOnlySpan<Complex> phaseGrid, int length)
    {
        if (radices.IsEmpty)
        {
            if (output.Length == 16)
                ForwardMixedLeaf(input, offset, stride, output);
            else
                ForwardBatchLeaf8(input, offset, stride, output);
            return;
        }

        // 对每一路从错开的原始偏移按较大步长取点；各子谱紧挨存放，便于下一层按同一 k 读取。
        int radix = radices[0];
        int width = output.Length / radix;
        for (int part = 0; part < radix; part++)
            ForwardDesktopBatchStages(input, offset + part * stride, stride * radix,
                output.Slice(part * width, width), radices[1..], phaseGrid, length);

        // 依据子变换长度换算查表步长，给各路补上起点不同造成的相移。
        int phaseStep = length / output.Length;
        Span<Complex> values = stackalloc Complex[7];
        for (int k = 0; k < width; k++)
        {
            values[0] = output[k];
            for (int part = 1; part < radix; part++)
                values[part] = output[k + width * part] * BatchRoot(phaseGrid, length, part * phaseStep * k);

            // 按当前路数使用三、四、五或七点蝶形，各输出间隔 width，形成上一层需要的排列。
            var destination = output[k..];
            switch (radix)
            {
                case 3: BatchRadixThree(values, destination, width); break;
                case 4:
                    (destination[0], destination[width], destination[2 * width], destination[3 * width])
                        = RadixFour(values[0], values[1], values[2], values[3]);
                    break;
                case 5: BatchRadixFive(values, destination, width); break;
                case 7: BatchRadixSeven(values, destination, width); break;
            }
        }
    }

    // 用第一八分圆恢复一个整圆的复旋转因子，只需比较索引并调整实虚分量及符号。
    private static Complex BatchRoot(ReadOnlySpan<Complex> phaseGrid, int length, int index)
    {
        int half = length / 2, quarter = length / 4, octant = length / 8;
        // 依次按半圆、象限、八分圆折叠，记录镜像方向，以便取数后恢复。
        bool positiveSine = index > half;
        if (positiveSine) index = length - index;
        bool negativeCosine = index > quarter;
        if (negativeCosine) index = half - index;
        bool reflect = index > octant;
        if (reflect) index = quarter - index;

        // The 12000-point plan reads every other point of the 24000-point grid.
        // 12000 共用 24000 的相位表，索引乘二仍对应同样的角度；折过 π/4 要交换正弦和余弦。
        Complex root = phaseGrid[length == 12000 ? 2 * index : index];
        if (reflect) root = new(-root.Imaginary, -root.Real);
        return new(negativeCosine ? -root.Real : root.Real, positiveSine ? -root.Imaginary : root.Imaginary);
    }

    // 按原始采样步长加载八个复样本，邻近两个实样本给出一组实部、虚部。
    private static void ForwardBatchLeaf8(ReadOnlySpan<double> input, int offset, int stride, Span<Complex> output)
    {
        Span<Complex> x = stackalloc Complex[8];
        for (int i = 0; i < x.Length; i++)
        {
            int index = offset + i * stride;
            x[i] = new(input[index], input[index + 1]);
        }

        // This mixed-plan leaf rounds rotated components before the final combinations.
        // 45° 旋转只用 1/√2；将相隔四点的输入配对，先算和差，再准备奇数格所需的旋转。
        const double root = 0.7071067811865476;
        Complex evenAc = x[0] + x[4], evenBd = x[2] + x[6];
        Complex evenDifference = x[0] - x[4], evenOther = x[6] - x[2];
        Complex oddAc = x[1] + x[5], oddBd = x[3] + x[7];
        Complex p = Scale(x[1] - x[5], root), q = Scale(x[7] - x[3], root);
        Complex sum = p + q, difference = q - p;
        Complex first = evenDifference + sum, third = evenDifference - sum;
        Complex firstRotation = Rotate(evenOther + difference), thirdRotation = Rotate(difference - evenOther);
        // 偶、奇子序列再组合，八个输出沿正变换格序铺开。
        Complex even = evenAc + evenBd, odd = oddAc + oddBd;
        Complex second = evenAc - evenBd, secondRotation = Rotate(oddBd - oddAc);
        output[0] = even + odd;
        output[1] = first - firstRotation;
        output[2] = second - secondRotation;
        output[3] = third - thirdRotation;
        output[4] = even - odd;
        output[5] = thirdRotation + third;
        output[6] = secondRotation + second;
        output[7] = firstRotation + first;
    }

    // 三点 DFT 把后两点成对，乘 cos(2π/3) 的和及乘 sin(−2π/3) 的差构成对称两格。
    private static void BatchRadixThree(ReadOnlySpan<Complex> x, Span<Complex> output, int step)
    {
        Complex sum = x[1] + x[2], difference = Scale(x[1] - x[2], -0.8660254037844386);
        Complex center = x[0] + Scale(sum, -0.5);
        output[0] = sum + x[0];
        output[step] = new(center.Real - difference.Imaginary, center.Imaginary + difference.Real);
        output[2 * step] = new(center.Real + difference.Imaginary, center.Imaginary - difference.Real);
    }

    // 五点蝶形中，两对对称输入的和对应余弦，差对应正弦；只需两组非平凡根常量。
    private static void BatchRadixFive(ReadOnlySpan<Complex> x, Span<Complex> output, int step)
    {
        const double c1 = 0.30901699437494745, c2 = -0.8090169943749473;
        const double s1 = -0.9510565162951535, s2 = -0.5877852522924732;
        Complex a = x[4] + x[1], b = x[3] + x[2];
        Complex d = x[1] - x[4], e = x[2] - x[3];
        // 两套余弦投影与两套正弦投影交叉组合，正反格的符号相反；零格直接求和。
        Complex p = new((a.Real * c1 + b.Real * c2) + x[0].Real,
            (b.Imaginary * c2 + a.Imaginary * c1) + x[0].Imaginary);
        Complex q = (Scale(a, c2) + Scale(b, c1)) + x[0];
        Complex u = Scale(d, s1) + Scale(e, s2), v = Scale(d, s2) - Scale(e, s1);
        output[0] = (a + b) + x[0];
        output[step] = new(p.Real - u.Imaginary, p.Imaginary + u.Real);
        output[2 * step] = new(q.Real - v.Imaginary, q.Imaginary + v.Real);
        output[3 * step] = new(q.Real + v.Imaginary, q.Imaginary - v.Real);
        output[4 * step] = new(p.Real + u.Imaginary, p.Imaginary - u.Real);
    }

    // 七点蝶形需要三对对称输入以及三个不同角度的正余弦，输出一个直流和三对频格。
    private static void BatchRadixSeven(ReadOnlySpan<Complex> x, Span<Complex> output, int step)
    {
        const double c1 = 0.6234898018587336, c2 = -0.22252093395631434, c3 = -0.900968867902419;
        const double s1 = -0.7818314824680298, s2 = -0.9749279121818236, s3 = -0.43388373911755823;
        Complex a = x[1] + x[6], b = x[2] + x[5], c = x[3] + x[4];
        Complex d = x[1] - x[6], e = x[2] - x[5], f = x[3] - x[4];
        // 循环置换三个角度的系数建立三个余弦投影；对差分量作有符号正弦投影，成对写出结果。
        Complex p1 = ((Scale(a, c1) + Scale(b, c2)) + Scale(c, c3)) + x[0];
        Complex p2 = ((Scale(a, c2) + Scale(b, c3)) + Scale(c, c1)) + x[0];
        Complex p3 = x[0] + ((Scale(a, c3) + Scale(b, c1)) + Scale(c, c2));
        Complex u1 = (Scale(d, s1) + Scale(e, s2)) + Scale(f, s3);
        Complex u2 = (Scale(d, s2) - Scale(e, s3)) - Scale(f, s1);
        Complex u3 = (Scale(d, s3) - Scale(e, s1)) + Scale(f, s2);
        output[0] = ((a + b) + c) + x[0];
        output[step] = new(p1.Real - u1.Imaginary, u1.Real + p1.Imaginary);
        output[2 * step] = new(p2.Real - u2.Imaginary, u2.Real + p2.Imaginary);
        output[3 * step] = new(p3.Real - u3.Imaginary, u3.Real + p3.Imaginary);
        output[4 * step] = new(u3.Imaginary + p3.Real, p3.Imaginary - u3.Real);
        output[5 * step] = new(u2.Imaginary + p2.Real, p2.Imaginary - u2.Real);
        output[6 * step] = new(u1.Imaginary + p1.Real, p1.Imaginary - u1.Real);
    }

    // 整圈旋转因子由第一八分圆镜像即可得到；每种基础长度只需保存这一段。
    private static class DesktopBatchPhaseGrids
    {
        private static readonly Complex[] Grid8064 = GenerateFirstOctant(8064);
        private static readonly Complex[] Grid11200 = GenerateFirstOctant(11200);
        private static readonly Complex[] Grid24000 = GenerateFirstOctant(24000);

        // 两个专表分别给 8064 和 11200，另两种受支持长度共享 24000 表。
        internal static ReadOnlySpan<Complex> Get(int length) => length switch
        {
            8064 => Grid8064,
            11200 => Grid11200,
            _ => Grid24000
        };

        // 依次采样 0 到 π/4（含端点），每格相角相差 2π/N，以备各蝶形复用。
        private static Complex[] GenerateFirstOctant(int length)
        {
            var result = new Complex[length / 8 + 1];
            double step = Math.Tau / length;
            for (int i = 0; i < result.Length; i++)
                result[i] = Phase(i * step);
            return result;
        }

        // Each polynomial is evaluated from highest degree to lowest, rounding once per FMA.
        // 余弦是偶函数，所以系数配合 x² 的多项式，再接常数项与二次项。
        private static ReadOnlySpan<double> CosineCoefficients =>
        [
            -1.1382639816236089e-11, 2.0876146382372144e-09, -2.7557317272344893e-07,
            2.4801587298767044e-05, -0.0013888888888887398, 0.041666666666666664
        ];

        // 正弦是奇函数，所以先算 x² 的多项式，再乘 x³ 并加上线性项。
        private static ReadOnlySpan<double> SineCoefficients =>
        [
            1.5918144304485914e-10, -2.5051132068021698e-08, 2.7557316103728802e-06,
            -0.00019841269836761127, 0.0083333333333309497, -0.16666666666666666
        ];

        // 极小角直接取 (1,−x)，稍大角保留低阶项；一般区间用融合乘加减小每步舍入。
        private static Complex Phase(double x)
        {
            if (x < 1.0 / (1 << 27)) return new(1, -x);
            if (x < 1.0 / (1 << 13))
                return new(Math.FusedMultiplyAdd(-x, x * 0.5, 1),
                    -Math.FusedMultiplyAdd(-(x * x * x), 1.0 / 6, x));

            double square = x * x;
            double cos = Math.FusedMultiplyAdd(
                Math.FusedMultiplyAdd(Polynomial(square, CosineCoefficients), square, -0.5), square, 1);
            // 接近八分圆边界时另行规约角度，返回的虚部总按正向 FFT 的负正弦约定。
            double sin = x < Math.PI / 4
                ? Math.FusedMultiplyAdd(x * square, Polynomial(square, SineCoefficients), x)
                : SineAtOctantBoundary(x);
            return new(cos, -sin);
        }

        // 借 sin(x)=cos(x−π/2)，将 π/2 拆成高低部分求余角，再把余角低位的贡献加回。
        private static double SineAtOctantBoundary(double x)
        {
            // In the reference the sine at the rounded pi/4 boundary uses quadrant reduction.
            const double halfPiTail = 6.1232339957367574e-17;
            double high = x - Math.PI / 2, reduced = high - halfPiTail;
            double low = ((high - reduced) - halfPiTail) - 8.4784276603688985e-32;
            double square = reduced * reduced, half = 0.5 * square, near = 1 - half;
            double error = Math.FusedMultiplyAdd(-reduced, low, (1 - near) - half);
            return Math.FusedMultiplyAdd(Polynomial(square, CosineCoefficients), square * square, error) + near;
        }

        // 按从高次到低次的 Horner 顺序求多项式，每步用 FMA 只舍入一次。
        private static double Polynomial(double x, ReadOnlySpan<double> coefficients)
        {
            double value = coefficients[0];
            foreach (double coefficient in coefficients[1..])
                value = Math.FusedMultiplyAdd(value, x, coefficient);
            return value;
        }
    }
}
