using System.Numerics;

namespace ApxRemix.Core.Dsp;

internal static partial class SequencePackedRealFft
{
    // 1536 个实点打包成 768 个复点，其中 768=3×256。这是外层需要三路蝶形的原因。
    private const int MixedLength = 1536;

    // 三路分别读取每第三个打包样本，换算回原始实数组步长为 6，各路先求 256 点子谱。
    private static void ForwardMixed(ReadOnlySpan<double> input, Span<Complex> packed, bool desktop)
    {
        int width = packed.Length / 3;
        for (int part = 0; part < 3; part++)
            ForwardPacked(input, 2 * part, 6, packed.Slice(part * width, width), [], desktop);

        // 后两路先补因起始采样偏移产生的相角，再用三次单位根 −1/2±i√3/2 合并。
        const double negativeRootThreeOverTwo = -0.8660254037844386;
        for (int k = 0; k < width; k++)
        {
            Complex a = packed[k];
            Complex b = packed[k + width] * MixedRoot(2 * k, desktop);
            Complex c = packed[k + 2 * width] * MixedRoot(4 * k, desktop);
            // b、c 的和给两侧输出相同的部分，差给符号相反的部分；直流支路直接相加。
            Complex sum = b + c;
            Complex center = a + Scale(sum, -0.5);
            Complex difference = Scale(b - c, negativeRootThreeOverTwo);
            packed[k] = sum + a;
            packed[k + width] = new(center.Real - difference.Imaginary, center.Imaginary + difference.Real);
            packed[k + 2 * width] = new(center.Real + difference.Imaginary, center.Imaginary - difference.Real);
        }
    }

    // 混合长度使用八分圆复数表，下面先把整圈索引镜像到可查区域并记录符号。
    private static Complex MixedRoot(int index, bool desktop)
    {
        bool negativeSine = index <= MixedLength / 2;
        if (!negativeSine)
            index = MixedLength - index;
        bool negativeCosine = index > MixedLength / 4;
        if (negativeCosine)
            index = MixedLength / 2 - index;
        bool reflect = index > MixedLength / 8;
        if (reflect)
            index = MixedLength / 4 - index;
        // 表项成对保存 cos 和负 sin，还原镜像后获得正向 FFT 的复旋转因子。
        ReadOnlySpan<double> octant = desktop ? DesktopMixedOctant : MixedOctant;
        double cos = octant[2 * index], sin = octant[2 * index + 1];
        if (reflect)
            (cos, sin) = (-sin, -cos);
        return new Complex(negativeCosine ? -cos : cos, negativeSine ? sin : -sin);
    }

    // 把十六个打包复数的实部序列和虚部序列分别做实 FFT；利用线性性计算 FFT(real)+i·FFT(imag)。
    private static void ForwardMixedLeaf(ReadOnlySpan<double> input, int offset, int stride, Span<Complex> output)
    {
        // This first stage computes separate 16-point real FFTs of the two packed lanes.
        Span<Complex> real = stackalloc Complex[16];
        Span<Complex> imag = stackalloc Complex[16];
        ForwardReal16(input, offset, stride, real);
        ForwardReal16(input, offset + 1, stride, imag);
        for (int k = 0; k < output.Length; k++)
            output[k] = new(real[k].Real - imag[k].Imaginary, imag[k].Real + real[k].Imaginary);
    }

    // 十六点实 FFT 先拆成四组，每组取相隔四个样本的四点，局部计算直流、交替和及正交分量。
    private static void ForwardReal16(ReadOnlySpan<double> input, int offset, int stride, Span<Complex> spectrum)
    {
        Span<double> dc = stackalloc double[4], nyquist = stackalloc double[4];
        Span<double> real = stackalloc double[4], imag = stackalloc double[4];
        for (int part = 0; part < 4; part++)
        {
            int start = offset + part * stride;
            double a = input[start], b = input[start + 4 * stride];
            double c = input[start + 8 * stride], d = input[start + 12 * stride];
            double ac = a + c, bd = b + d;
            dc[part] = ac + bd;
            nyquist[part] = ac - bd;
            real[part] = a - c;
            imag[part] = d - b;
        }

        // 三个常数对应 π/4、π/8 的三角值，各四点子谱借它们转到同一参考后可直接合并。
        const double h = 0.7071067811865476;  // Cosine of π/4 for this 16-point kernel.
        const double l = 0.3826834323650898;  // Sine of π/8.
        const double c8 = 0.9238795325112867; // Cosine of π/8.
        double dcEven = dc[0] + dc[2], dcOdd = dc[1] + dc[3];
        spectrum[0] = new(dcEven + dcOdd, 0);
        spectrum[4] = new(dc[0] - dc[2], dc[3] - dc[1]);
        spectrum[8] = new(dcEven - dcOdd, 0);

        // 局部交替和决定第 2 和第 6 格，局部正交分量则经不同角度旋转后形成奇数格。
        double p1 = nyquist[1] * h, p3 = nyquist[3] * h;
        double pSum = p1 + p3, pDifference = p1 - p3;
        spectrum[2] = new(nyquist[0] + pDifference, -(nyquist[2] + pSum));
        spectrum[6] = new(nyquist[0] - pDifference, nyquist[2] - pSum);

        double r1 = c8 * real[1] + l * imag[1], i1 = c8 * imag[1] - l * real[1];
        double r2 = h * real[2] + h * imag[2], i2 = h * imag[2] - h * real[2];
        double r3 = l * real[3] + c8 * imag[3], i3 = l * imag[3] - c8 * real[3];
        double evenRealSum = real[0] + r2, evenRealDifference = real[0] - r2;
        double evenImagSum = imag[0] + i2, evenImagDifference = imag[0] - i2;
        double oddRealSum = r1 + r3, oddRealDifference = r3 - r1;
        double oddImagSum = i1 + i3, oddImagDifference = i1 - i3;
        spectrum[1] = new(evenRealSum + oddRealSum, evenImagSum + oddImagSum);
        spectrum[3] = new(evenRealDifference - oddImagDifference, oddRealDifference - evenImagDifference);
        spectrum[5] = new(evenRealDifference + oddImagDifference, evenImagDifference + oddRealDifference);
        spectrum[7] = new(evenRealSum - oddRealSum, oddImagSum - evenImagSum);
        // 实值输入在负频率处是正频率的复共轭，补齐这些格即可供上一层当完整复谱合并。
        for (int k = 1; k < 8; k++)
            spectrum[16 - k] = Complex.Conjugate(spectrum[k]);
    }
}
