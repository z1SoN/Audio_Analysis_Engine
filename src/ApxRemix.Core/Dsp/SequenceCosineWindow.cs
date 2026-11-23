using System.Numerics;
using ApxRemix.Core.Models;
using static ApxRemix.Core.Dsp.SequenceX87FixedPoint;

namespace ApxRemix.Core.Dsp;

// The x86 window expression consumes the extended FCOS result before storing
// a binary64 weighted term. Rounding cos(x) to double first changes the taper.
internal static class SequenceCosineWindow
{
    // 按点生成余弦和窗。长度决定整圈的角步长，窗类型决定每个角频率分量的权重。
    internal static double[] Generate(int length, FftWindowType type)
    {
        bool desktop = SequenceBinEstimator.DesktopWorksheet;
        // 非桌面配置可直接取 x87 子进程的窗口；下面的定点余弦用于进程内生成。
        if (!desktop && SequenceX87Client.TryWindow(length, type, out double[] x87))
            return x87;
        // On x64 the scalar cosine is stored at binary64 before multiplication.
        // 桌面先把余弦变为 double 再乘权重，另一配置让权重先参与定点乘积，取舍低位的时机会不同。
        double Term(BigInteger cosine, double weight) =>
            desktop ? Multiply(cosine, 1) * weight : Multiply(cosine, weight);
        var values = new double[length];
        double step = 2 * Math.PI / length;
        for (int i = 0; i < length; i++)
        {
            // 令记录中点对应零弧度，项的相位为 x、2x、3x；Hann、三项窗和四项窗分别选取相应项。
            double x = i * step - Math.PI;
            var c = Cosine(x);
            values[i] = type switch {
                FftWindowType.Hann => 0.5 + Term(c, 0.5),
                FftWindowType.BlackmanHarris => 0.42 + Term(c, 0.5) + Term(Cosine(2*x), 0.08),
                FftWindowType.BlackmanHarris4 => 0.35875 + Term(c, 0.48829)
                    + Term(Cosine(2*x), 0.14128) + Term(Cosine(3*x), 0.01168),
                _ => throw new ArgumentOutOfRangeException(nameof(type))
            };
        }
        return values;
    }

}
