using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace ApxRemix.Core.Dsp;

internal static class SequenceWindowNormalization
{
    // 输入一个待加窗的系数数组，返回原窗和、能量及校正量，同时在原数组上完成缩放。
    internal static (double Sum, double Energy, double Gain, double DcCorrection, double PeakCorrection) Apply(Span<double> taper)
        => Normalize(taper, Sum(taper), SumOfSquares(taper));

    // 桌面路径固定数组地址后检查其 16 字节对齐情况，因为分组累加会随首个地址改变舍入顺序。
    internal static (double Sum, double Energy, double Gain, double DcCorrection, double PeakCorrection) ComputeDesktop(double[] taper)
    {
        var pin = GCHandle.Alloc(taper, GCHandleType.Pinned);
        try
        {
            bool startsWithUnalignedSample = (pin.AddrOfPinnedObject().ToInt64() & 15) != 0;
            return ComputeDesktopForAlignment(taper, startsWithUnalignedSample);
        }
        // 离开本次计算就解开固定，异常时也让垃圾回收器重新移动该数组。
        finally { pin.Free(); }
    }

    // 传入是否从未对齐点开始，便可用同一组系数分别复算两种地址情形。
    internal static (double Sum, double Energy, double Gain, double DcCorrection, double PeakCorrection) ComputeDesktopForAlignment(
        Span<double> taper, bool startsWithUnalignedSample)
        => Normalize(taper, Sum(taper), DesktopSumOfSquares(taper, startsWithUnalignedSample));

    // 按存储顺序累加时域系数，常量信号经窗后在直流格的幅值正比于这个和。
    private static double Sum(ReadOnlySpan<double> taper)
    {
        double sum = 0;
        foreach (double coefficient in taper) sum += coefficient;
        return sum;
    }

    // 将窗能量归一化，使每一格功率可按单边谱约定积分；输入系数就地乘 gain。
    private static (double Sum, double Energy, double Gain, double DcCorrection, double PeakCorrection) Normalize(
        Span<double> taper, double sum, double energy)
    {
        double gain = Math.Sqrt(2.0 / (taper.Length * energy));
        for (int i = 0; i < taper.Length; i++) taper[i] *= gain;

        // 缩放后恒定输入的直流响应等于 scaledSum，取倒数恢复直流；单音读数另带 √2 标度。
        double scaledSum = sum * gain;
        return (sum, energy, gain, 1.0 / scaledSum, Math.Sqrt(2.0) / scaledSum);
    }

    // 根据长度和处理器数量决定平方和的分组；浮点加法交换顺序会改变末位，因此固定分块次序。
    private static double SumOfSquares(ReadOnlySpan<double> taper)
    {
        // The original x86 MKL begins splitting a contiguous dot at 4096 samples.
        // Each block is computed independently and then added in block order.
        int count = taper.Length < 4096 ? 1 : Math.Min(Environment.ProcessorCount, (taper.Length - 1) / 2048 + 1);
        int size = taper.Length / count;
        int extra = taper.Length % count;
        int start = 0;
        double total = 0;
        for (int block = 0; block < count; block++)
        {
            // 余数平均分配给靠前的块，每个块各自求和再按起点顺序合并。
            int length = size + (block < extra ? 1 : 0);
            total += SumOfSquaresBlock(taper.Slice(start, length));
            start += length;
        }
        return total;
    }

    // 一个块用四组双通道累加器。先按既定边界循环，再把不足整块的成对数据送入第一组。
    private static double SumOfSquaresBlock(ReadOnlySpan<double> taper)
    {
        // MKL p4m3 xddot: each accumulator has an even and an odd lane.
        // Full 32-sample blocks rotate through four pairs, then the complete
        // tail pairs all accumulate into the first pair. Multiply and add
        // round separately before the final pair and horizontal reductions.
        var first = Vector128<double>.Zero;
        var second = Vector128<double>.Zero;
        var third = Vector128<double>.Zero;
        var fourth = Vector128<double>.Zero;
        int full = taper.Length & ~31;
        int paired = taper.Length & ~1;
        for (int i = 0; i < full; i += 8)
        {
            first += SquarePair(taper, i);
            second += SquarePair(taper, i + 2);
            third += SquarePair(taper, i + 4);
            fourth += SquarePair(taper, i + 6);
        }
        // 合并时先两两相加，再把两个通道横向相加；若还有单个尾点，单独加上它的平方。
        for (int i = full; i < paired; i += 2) first += SquarePair(taper, i);
        var reduced = (first + second) + (third + fourth);
        double total = reduced.GetElement(0) + reduced.GetElement(1);
        if (paired < taper.Length)
        {
            double tail = taper[paired];
            total += tail * tail;
        }
        return total;
    }

    // 相邻两个系数装在不同通道，逐元素平方，不在此处合并。
    private static Vector128<double> SquarePair(ReadOnlySpan<double> taper, int index)
    {
        var pair = Vector128.Create(taper[index], taper[index + 1]);
        return pair * pair;
    }

    // 桌面版本按实际起始对齐选首点，四组向量在末尾按与另一配置不同的顺序合并。
    private static double DesktopSumOfSquares(ReadOnlySpan<double> taper, bool startsWithUnalignedSample)
    {
        // For x64 mkl_def a taper at 8 mod 16 starts with a scalar contribution
        // to the first lane. The paired part begins at the next aligned address.
        int i = 0;
        var first = Vector128<double>.Zero;
        // 首元素错半个向量时先独立计入零号通道，下一对即可从对齐地址开始。
        if (startsWithUnalignedSample && taper.Length > 0)
        {
            first = Vector128.Create(taper[0] * taper[0], 0.0);
            i = 1;
        }
        var second = Vector128<double>.Zero;
        var third = Vector128<double>.Zero;
        var fourth = Vector128<double>.Zero;
        // 八点一组分配给四对累加器；尾部先收成对样本，可能剩余的一点归入首通道。
        for (; i + 7 < taper.Length; i += 8)
        {
            first += SquarePair(taper, i);
            second += SquarePair(taper, i + 2);
            third += SquarePair(taper, i + 4);
            fourth += SquarePair(taper, i + 6);
        }
        for (; i + 1 < taper.Length; i += 2) first += SquarePair(taper, i);
        if (i < taper.Length)
            first = first.WithElement(0, first.GetElement(0) + taper[i] * taper[i]);

        // 括号规定四组的归并顺序，最后再混合两个通道，故同样的窗能量也可能有末位差异。
        var reduced = ((first + second) + third) + fourth;
        return reduced.GetElement(0) + reduced.GetElement(1);
    }
}
