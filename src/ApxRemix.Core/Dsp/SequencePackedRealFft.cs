using System.Numerics;

namespace ApxRemix.Core.Dsp;

internal static partial class SequencePackedRealFft
{
    // 八点蝶形中的 45° 旋转只需交换分量、加减和乘 1/√2，保留此处的 double 取值。
    private const double InverseRootTwo = 0.70710678118654746;

    // 专用实 FFT 有三种确定的长度；其他长度由上层选择另外的内核。
    public static bool Supports(int length) => length is 1024 or 1536 or 4096;

    // 每两个相邻实样本分别放到一个复数的实部和虚部，先做半长复变换，再解包为实输入的半谱。
    public static Complex[] Forward(ReadOnlySpan<double> input, bool desktop)
    {
        int length = input.Length;
        // 根表以原始长度为基准；1536 的空表标记三因子混合基路径，4096 的根表还依赖工作表选择。
        ReadOnlySpan<double> sine = length switch
        {
            1024 => QuarterSine1024,
            1536 => [],
            4096 => desktop ? DesktopSineGrid4096 : QuarterSine4096,
            _ => throw new ArgumentOutOfRangeException(nameof(input))
        };
        int half = length / 2;
        int quarter = length / 4;
        var packed = new Complex[half];
        bool mixed = length == 1536;
        // 1536 先拆三路，其余从步长 2 递归取复样本，所得 packed 均包含偶、奇两路实序列的频率信息。
        if (mixed)
            ForwardMixed(input, packed, desktop);
        else
            ForwardPacked(input, 0, 2, packed, sine, desktop);

        // Convert the complex FFT of interleaved even/odd samples to the real spectrum.
        // The order of products, sums and differences is significant at notch boundaries.
        // 第零个打包频格的实虚分别是偶、奇点之和，相加为直流，相减为奈奎斯特，中间格单独取共轭。
        var spectrum = new Complex[half + 1];
        Complex first = packed[0];
        spectrum[0] = new Complex(first.Imaginary + first.Real, 0);
        spectrum[half] = new Complex(first.Real - first.Imaginary, 0);
        double middlePhase = packed[quarter].Imaginary;
        spectrum[quarter] = new Complex(packed[quarter].Real, mixed ? -middlePhase : 0.0 - middlePhase);
        for (int k = 1; k < quarter; k++)
        {
            // k 与 N/2−k 两个打包格共同确定一对实 FFT 输出。旋转因子补上相邻奇采样相对偶采样的一点延时。
            int mirror = half - k;
            Complex left = packed[k], right = packed[mirror];
            double u = left.Real - right.Real;
            double v = left.Imaginary + right.Imaginary;
            Complex root = Root(sine, k, desktop);
            double e = 0.5 * root.Real;
            double f = 0.5 * (1 + root.Imaginary);
            double real = e * v + f * u;
            double imaginary = f * v - e * u;
            spectrum[k] = new Complex(right.Real + real, imaginary - right.Imaginary);
            spectrum[mirror] = new Complex(left.Real - real, imaginary - left.Imaginary);
        }
        return spectrum;
    }

    // offset 与 stride 在原始实数组中计数；叶节点取 index 和 index+1 当作一个复数。
    private static void ForwardPacked(ReadOnlySpan<double> input, int offset, int stride,
        Span<Complex> output, ReadOnlySpan<double> sine, bool desktop)
    {
        // 16 点叶拆成两组实变换，8 点叶直接收集八个复数做八点蝶形，递归在此结束。
        if (output.Length == 16)
        {
            ForwardMixedLeaf(input, offset, stride, output);
            return;
        }
        if (output.Length == 8)
        {
            Span<Complex> values = stackalloc Complex[8];
            for (int i = 0; i < values.Length; i++)
            {
                int index = offset + i * stride;
                values[i] = new Complex(input[index], input[index + 1]);
            }

            RadixEight(values, output, outputStride: 1, scaleFirst: false);
            return;
        }

        // Desktop uses 8 × 8 × 8 or 8 × 8 × 4 × 8 from the first pass upward.
        // 其他节点分成四路，指定桌面阶段分八路。每路以增大的 stride 取样，结果存入连续子区。
        int radix = desktop && !sine.IsEmpty && output.Length is (64 or 512 or 2048) ? 8 : 4;
        int width = output.Length / radix;
        for (int part = 0; part < radix; part++)
            ForwardPacked(input, offset + part * stride, stride * radix, output.Slice(part * width, width), sine, desktop);

        // 根表一整圈的点数除以当前子变换长度，就是相邻相位之间要跳过的表格数。
        int length = sine.IsEmpty ? MixedLength : 4 * (sine.Length - 1);
        int step = length / output.Length;
        if (radix == 8)
        {
            // 八路每个 k 将子谱先转到同一时间参考再交给八点蝶形，第零路相位为零无需乘根。
            Span<Complex> terms = stackalloc Complex[8];
            for (int k = 0; k < width; k++)
            {
                terms[0] = output[k];
                int index = k * step;
                for (int part = 1; part < terms.Length; part++)
                    terms[part] = output[k + part * width] * Root(sine, part * index, desktop);
                RadixEight(terms, output[k..], width, scaleFirst: true);
            }
            return;
        }

        // 四路分别乘零、一、二、三倍相角，再据同一个四点蝶形写回间距为 width 的四个格。
        for (int k = 0; k < width; k++)
        {
            int index = k * step;
            Complex a = output[k];
            Complex b = output[k + width] * Root(sine, index, desktop);
            Complex c = output[k + 2 * width] * Root(sine, 2 * index, desktop);
            Complex d = output[k + 3 * width] * Root(sine, 3 * index, desktop);
            (output[k], output[k + width], output[k + 2 * width], output[k + 3 * width]) = RadixFour(a, b, c, d);
        }
    }

    // 偶序号和奇序号各做四点 DFT，再把奇序号的结果乘相位因子并与偶序号相加减。
    private static void RadixEight(ReadOnlySpan<Complex> values, Span<Complex> output, int outputStride, bool scaleFirst)
    {
        var (a, b, c, d) = RadixFour(values[0], values[2], values[4], values[6]);
        var (e, f, g, h) = RadixFour(values[1], values[3], values[5], values[7]);
        // 45° 项可先缩放再加减，也可先加减再缩放；scaleFirst 保留具体分解阶段的浮点舍入次序。
        if (scaleFirst)
        {
            // Later desktop passes round the two products before recombining the lanes.
            f = Scale(f, InverseRootTwo);
            h = Scale(h, -InverseRootTwo);
            f += Rotate(f);
            h -= Rotate(h);
        }
        else
        {
            f = Scale(f + Rotate(f), InverseRootTwo);
            h = Scale(h - Rotate(h), -InverseRootTwo);
        }
        // 第二维旋转为 −i，写入时前后四格分别用和、差，共用同一对中间量。
        g = Rotate(g);
        output[0] = a + e;
        output[outputStride] = b + f;
        output[2 * outputStride] = c + g;
        output[3 * outputStride] = d + h;
        output[4 * outputStride] = a - e;
        output[5 * outputStride] = b - f;
        output[6 * outputStride] = c - g;
        output[7 * outputStride] = d - h;
    }

    // 四点 DFT 里根只有 1、−1、±i。先形成两对和差，再交换分量完成 −i 乘法。
    private static (Complex, Complex, Complex, Complex) RadixFour(Complex a, Complex b, Complex c, Complex d)
    {
        Complex u = a + c, w = a - c;
        Complex v = b + d, q = Rotate(b - d);
        return (u + v, w + q, u - v, w - q);
    }

    // (x+iy) 乘 −i 得 y−ix，因此无需计算三角函数。
    private static Complex Rotate(Complex value) => new(value.Imaginary, -value.Real);

    // 实数增益同时伸缩实、虚分量，复数的相位方向保持。
    private static Complex Scale(Complex value, double gain) => new(value.Real * gain, value.Imaginary * gain);

    // 基于根表恢复 exp(−iθ)。空表转入 1536 的混合表，其余用正弦偏移四分之一圈得到余弦。
    private static Complex Root(ReadOnlySpan<double> sine, int index, bool desktop)
    {
        if (sine.IsEmpty)
            return MixedRoot(index, desktop);
        int quarter = sine.Length - 1;
        // An eight-way stage can rotate the cosine beyond one full turn.
        // 最后一个象限的余弦索引向前绕一圈，保持查表位置落在合法范围。
        if (index >= 3 * quarter)
            return new Complex(Sine(sine, index - 3 * quarter, quarter), -Sine(sine, index, quarter));
        return new Complex(Sine(sine, index + quarter, quarter), -Sine(sine, index, quarter));
    }

    // 四个象限通过第一象限的正弦值、倒序索引和符号变换恢复。
    private static double Sine(ReadOnlySpan<double> sine, int index, int quarter)
    {
        return (index / quarter) switch
        {
            0 => sine[index],
            1 => sine[2 * quarter - index],
            2 => -sine[index - 2 * quarter],
            _ => -sine[4 * quarter - index]
        };
    }
}
