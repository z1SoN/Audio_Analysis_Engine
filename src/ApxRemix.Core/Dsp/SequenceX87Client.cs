using System.Diagnostics;
using System.Runtime.InteropServices;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

// The x86 .NET Framework JIT evaluates these C# expressions on the x87.
// Only the operations whose instruction/rounding sequence differs cross the pipe.
internal static class SequenceX87Client
{
    // 仅当 Windows 上的专用可执行文件存在时才接通该通道；一把锁保护一份常驻子进程。
    private static readonly string Executable = Path.Combine(AppContext.BaseDirectory, "SequenceX87Worker.exe");
    private static readonly bool Available = OperatingSystem.IsWindows() && File.Exists(Executable);
    private static readonly object Gate = new();
    private static Process? process;

    // 把支持的三种余弦窗映成协议中的编号；不可用时返回 false，调用者可用托管数值路径。
    internal static bool TryWindow(int length, FftWindowType type, out double[] values)
    {
        values = [];
        if (!Available) return false;
        int kind = type switch {
            FftWindowType.Hann => 1,
            FftWindowType.BlackmanHarris => 2,
            FftWindowType.BlackmanHarris4 => 3,
            _ => throw new ArgumentOutOfRangeException(nameof(type))
        };
        // 命令 1 带点数及窗种类，应答恰有 length 个逐点系数。
        using var request = new MemoryStream(12);
        using var writer = new BinaryWriter(request);
        writer.Write(1);
        writer.Write(length);
        writer.Write(kind);
        values = MemoryMarshal.Cast<byte, double>(Run(request, length)).ToArray();
        return true;
    }

    // 命令 2 请求单个相位。输入顺序固定为虚部、实部、分数格，结果以弧度表示。
    internal static bool TryPhase(double imaginary, double real, double fraction, out double phase)
    {
        phase = double.NaN;
        if (!Available) return false;
        using var request = new MemoryStream(32);
        using var writer = new BinaryWriter(request);
        writer.Write(2);
        writer.Write(1);
        writer.Write(imaginary);
        writer.Write(real);
        writer.Write(fraction);
        phase = BitConverter.ToDouble(Run(request, 1));
        return true;
    }

    // 命令 3 用有效值、初相和帧内周期数从输入样本就地减去拟合正弦。
    internal static bool TrySubtractSine(Span<double> samples, double rms, double phase, double cycles)
        => Transform(3, samples, [rms, phase, cycles]);

    // 命令 4 给每格功率应用以 fundamentalBin 为中心的陷波。
    internal static bool TryApplyNotch(Span<double> power, double fundamentalBin)
        => Transform(4, power, [fundamentalBin]);

    // 命令和样本数在前，可选参数紧随其后，最后写输入数组。读齐等长回复再覆盖原始向量。
    private static bool Transform(int operation, Span<double> values, ReadOnlySpan<double> arguments)
    {
        if (!Available) return false;
        using var request = new MemoryStream(checked(8 + (arguments.Length + values.Length) * sizeof(double)));
        using var writer = new BinaryWriter(request);
        writer.Write(operation);
        writer.Write(values.Length);
        writer.Write(MemoryMarshal.AsBytes(arguments));
        writer.Write(MemoryMarshal.AsBytes(values));
        MemoryMarshal.Cast<byte, double>(Run(request, values.Length)).CopyTo(values);
        return true;
    }

    // 用 double 元素数算应答字节数；没有 MKL 库目录参数，实际执行的是独立 x87 工作进程。
    private static byte[] Run(MemoryStream message, int length)
    {
        lock (Gate)
            return SequenceWorkerChannel.Run(ref process, Executable, null, message, checked(length * sizeof(double)));
    }
}
