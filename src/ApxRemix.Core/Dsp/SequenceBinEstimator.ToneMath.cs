using System.Runtime.InteropServices;

namespace ApxRemix.Core.Dsp;

internal static partial class SequenceBinEstimator
{
    // 由一个复频格的方向和基频偏离整数格的距离，求记录起点的正弦相位（弧度）。
    internal static double BinPhase(double imaginary, double real, double fractionalBin)
    {
        Span<double> values = stackalloc double[] { imaginary, real, fractionalBin };
        // 优先使用工作进程的标量结果。无 MKL 时，非桌面配置再尝试独立 x87 进程及本地高精度计算。
        var native = Vector(-6, values, 1);
        if (native != null) return native[0];
        if (!DesktopWorksheet)
        {
            if (SequenceX87Client.TryPhase(imaginary, real, fractionalBin, out double phase)) return phase;
            return SequenceX86ToneMath.BinPhase(imaginary, real, fractionalBin);
        }
        // 桌面回退用 atan2 保留象限；加 π/2 改为正弦参考，再扣掉分数格的相位偏移。
        return Math.Atan2(imaginary, real) + Math.PI / 2 - Math.PI * fractionalBin;
    }

    // 在原样本缓冲中减去拟合正弦。true 表示调用方收到的数组已是残差。
    internal static bool TrySubtractSine(Span<double> samples, double rms, double phase, double cycles)
    {
        if (TryProcessToneVector(-7, samples, stackalloc double[] { rms, phase, cycles })) return true;
        // 桌面返回 false 交给上层常规算法；另一配置继续尝试 x87 或进程内模拟。
        if (DesktopWorksheet) return false;
        if (!SequenceX87Client.TrySubtractSine(samples, rms, phase, cycles))
            SequenceX86ToneMath.SubtractSine(samples, rms, phase, cycles);
        return true;
    }

    // 按分数基频对功率数组施加陷波。两个可选工作进程均未接手时由上层处理。
    internal static bool TryApplyResidualNotch(Span<double> power, double fundamentalBin)
        => TryProcessToneVector(-8, power, stackalloc double[] { fundamentalBin })
            || !DesktopWorksheet && SequenceX87Client.TryApplyNotch(power, fundamentalBin);

    // 共享协议先写命令和要覆盖的点数，再写标量参数与输入向量。没有库时原数据保持原样。
    private static bool TryProcessToneVector(int opcode, Span<double> values, ReadOnlySpan<double> arguments)
    {
        if (ActiveLibraryDirectory == null) return false;
        using var message = new MemoryStream(checked((values.Length + arguments.Length) * sizeof(double) + 8));
        using var writer = new BinaryWriter(message);
        writer.Write(opcode);
        writer.Write(values.Length);
        writer.Write(MemoryMarshal.AsBytes(arguments));
        writer.Write(MemoryMarshal.AsBytes(values));
        // 读齐与输入等长的结果才回写调用方的缓冲，避免把半条响应当成完整计算。
        var reply = Run(message, checked(values.Length * sizeof(double)));
        MemoryMarshal.Cast<byte, double>(reply).CopyTo(values);
        return true;
    }
}
