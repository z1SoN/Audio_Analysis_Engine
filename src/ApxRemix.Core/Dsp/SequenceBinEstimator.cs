using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

// 两条 FFT 算出的主瓣功率只差几个末位，也可能令 floor(2*f) 相差一格。
// 有相同的 Intel 数值库时，按所选工作表的进程位数执行窗生成、实数 FFT、向量运算与重心估计。
// 不逐帧启动进程，也不让子进程持有任何声卡、AP 会话或测量配置历史。
internal static partial class SequenceBinEstimator
{
    // 两种工作表各有一个常驻子进程。Gate 保证一条请求读完应答后，下一条才可写入管道。
    private static readonly object Gate = new();
    private static readonly string Worker = Path.Combine(AppContext.BaseDirectory, "SequenceBinWorker.exe");
    private static readonly string? LibraryDirectory = FindLibrary(Worker, "x86");
    private static readonly string DesktopWorker = Path.Combine(AppContext.BaseDirectory, "SequenceDesktopWorker.exe");
    private static readonly Lazy<string?> DesktopLibrary = new(() => FindLibrary(DesktopWorker, "x64"));
    // A worksheet call is synchronous. Other acquisition and analysis threads
    // retain their own host selection, including while a worker reply is pending.
    [ThreadStatic] internal static bool DesktopWorksheet;
    private static string? ActiveLibraryDirectory => DesktopWorksheet ? DesktopLibrary.Value : LibraryDirectory;
    internal static bool UsesNativeLibrary => ActiveLibraryDirectory != null;
    private static Process? process;
    private static Process? desktopProcess;

    // 先检查 Windows 和对应位数的程序，再定位该程序能装入的 MKL 运行库。
    private static string? FindLibrary(string executable, string architecture)
    {
        if (!OperatingSystem.IsWindows() || !File.Exists(executable)) return null;
        // 显式设置的目录缺库就报错，让部署错误能定位到指定路径。
        string? configured = Environment.GetEnvironmentVariable("APXREMIX_MKL_" + architecture.ToUpperInvariant());
        if (!string.IsNullOrWhiteSpace(configured))
        {
            string path = Path.GetFullPath(configured);
            if (!File.Exists(Path.Combine(path,"mkl_rt.dll")))
                throw new DirectoryNotFoundException($"指定的 {architecture} MKL 目录缺少 mkl_rt.dll，目录为 {path}");
            return path;
        }
        return null;
    }

    // 请求整数峰位和分数频格。count 是可分析的正频格数，width 是峰两侧参与重心的格数。
    internal static bool TryEstimate(ReadOnlySpan<double> pcm, FftWindowType window, int count, int width,
        int? peakHint, out int peak, out double bin)
    {
        // 无运行库时用 false 标记没有估计，调用者改用现有频谱；NaN 避免把未算出的频率当成直流。
        peak = 0;
        bin = double.NaN;
        if (ActiveLibraryDirectory == null) return false;
        // 消息先写记录长度及窗参数，再写每个 double 样本；选峰提示缺省时用 −1 通知工作进程自行搜索。
        using var request = new MemoryStream(checked(pcm.Length * sizeof(double) + 64));
        using var writer = new BinaryWriter(request);
        writer.Write(pcm.Length);
        writer.Write(window.ToString());
        writer.Write(count);
        writer.Write(width);
        writer.Write(peakHint ?? -1);
        writer.Write(MemoryMarshal.AsBytes(pcm));
        // 应答以 int 频格开头，紧接一个 double 重心；后续乘以采样率再除以帧长才得到 Hz。
        var reply = Run(request, sizeof(int) + sizeof(double));
        peak = BitConverter.ToInt32(reply);
        bin = BitConverter.ToDouble(reply,sizeof(int));
        return true;
    }

    // Negative opcodes distinguish the independent numerical primitives from a
    // positive-length tone-estimation request on the same persistent numerical worker.
    // 请求一次性准备窗。应答依次装入两个校正标量、length 个窗系数、width 个复数直流模板点。
    internal static double[]? PrepareWindow(int length, FftWindowType type, int width)
    {
        if (ActiveLibraryDirectory == null) return null;
        using var message = new MemoryStream();
        using var writer = new BinaryWriter(message);
        writer.Write(-1);
        writer.Write(length);
        writer.Write(type.ToString());
        writer.Write(width);
        return Doubles(Run(message, checked((length + 2 * width + 2) * sizeof(double))));
    }

    // 对实采样序列做正变换。半谱含直流和最高频格，每格用两个 double 传输实部、虚部。
    internal static double[]? ForwardReal(ReadOnlySpan<double> samples) => Vector(-2, samples, 2 * (samples.Length / 2 + 1));
    // 将时域采集块与频域逆滤波器交给工作进程，返回等长的时域脉冲响应。
    internal static double[]? Deconvolve(ReadOnlySpan<double> samples, ReadOnlySpan<Complex> inverse)
        => ResponseOperation(-11, samples.Length, samples, inverse);
    // 将复数谱与逐格校正系数相乘后取模，返回一个频格一个线性幅值。
    internal static double[]? CorrectedMagnitudes(ReadOnlySpan<Complex> spectrum, ReadOnlySpan<Complex> correction)
        => ResponseOperation(-12, spectrum.Length, MemoryMarshal.Cast<Complex,double>(spectrum), correction);

    // 给出等波纹窗的一段相角区间和点数，用工作进程生成时域门的过渡段。
    internal static double[]? EquirippleTaper(int length, double start, double stop)
    {
        if (ActiveLibraryDirectory == null) return null;
        using var message = new MemoryStream();
        using var writer = new BinaryWriter(message);
        writer.Write(-14);
        writer.Write(length);
        writer.Write(start);
        writer.Write(stop);
        return Doubles(Run(message, checked(length * sizeof(double))));
    }

    // 根据参考扫频和实际激励的两个频谱设计补偿。两段输出分别用于反卷积和响应曲线校正。
    internal static void DesignResponse(int length, int rate, double firstHz, double lastHz, bool digital,
        Complex[] reference, Complex[] drive, out Complex[] inverse, out Complex[] correction)
    {
        // 没有可用库时直接在当前进程计算同名响应设计内核。
        if (ActiveLibraryDirectory == null)
        {
            SequenceResponseKernel.Design(length,rate,firstHz,lastHz,digital,reference,drive,out inverse,out correction);
            return;
        }
        // 进程间按固定顺序传输采样率、扫频界限、输入标度和两组等长复数谱。
        using var message = new MemoryStream();
        using var writer = new BinaryWriter(message);
        writer.Write(-15);
        writer.Write(length);
        writer.Write((double)rate);
        writer.Write(firstHz);
        writer.Write(lastHz);
        writer.Write(digital);
        writer.Write(MemoryMarshal.AsBytes(reference.AsSpan()));
        writer.Write(MemoryMarshal.AsBytes(drive.AsSpan()));
        // 第一段与参考谱等长，余下字节解释为第二段校正系数；在此复制以独立保存两个结果。
        int count = reference.Length;
        var bytes = Run(message, checked((count + count / 2 + 1) * 2 * sizeof(double)));
        var values = MemoryMarshal.Cast<byte,Complex>(bytes);
        inverse = values[..count].ToArray();
        correction = values[count..].ToArray();
    }

    // 两种响应运算共用传输格式。操作码决定工作端解释为反卷积还是复谱幅值校正。
    private static double[]? ResponseOperation(int opcode, int count, ReadOnlySpan<double> data, ReadOnlySpan<Complex> kernel)
    {
        if (ActiveLibraryDirectory == null) return null;
        // 预留操作码、结果点数和两段数据的空间；复数核的每点占两个 double。
        using var message = new MemoryStream(checked((data.Length + 2 * kernel.Length) * sizeof(double) + 8));
        using var writer = new BinaryWriter(message);
        writer.Write(opcode);
        writer.Write(count);
        writer.Write(MemoryMarshal.AsBytes(data));
        writer.Write(MemoryMarshal.AsBytes(kernel));
        return Doubles(Run(message, checked(count * sizeof(double))));
    }
    // 输入一对实虚分量，返回一个模平方，故输出长度为实数槽位数的一半。
    internal static double[]? SquaredMagnitudes(ReadOnlySpan<double> interleaved) => Vector(-3, interleaved, interleaved.Length / 2);
    // 把每格功率开平方得到幅值。工作进程不可用时由当前进程逐元素计算。
    internal static double[] SquareRoots(double[] values) => Vector(-4, values, values.Length) ?? Array.ConvertAll(values, Math.Sqrt);

    // 仅请求未经归一化的时域窗，长度与最终要相乘的采样块一致。
    internal static double[]? RawWindow(int length, FftWindowType type)
    {
        // The desktop Framework worker can evaluate these cosine windows without MKL.
        if (ActiveLibraryDirectory == null &&
            !(DesktopWorksheet && OperatingSystem.IsWindows() && Environment.Is64BitOperatingSystem && File.Exists(DesktopWorker)
                && type is FftWindowType.Hann or FftWindowType.BlackmanHarris or FftWindowType.BlackmanHarris4))
            return null;
        using var message = new MemoryStream();
        using var writer = new BinaryWriter(message);
        writer.Write(-5);
        writer.Write(length);
        writer.Write(type.ToString());
        return Doubles(Run(message, checked(length * sizeof(double))));
    }

    // 向量命令带输入长度；输出长度由操作确定，例如求模平方将每对分量压成一个值。
    private static double[]? Vector(int opcode, ReadOnlySpan<double> values, int resultLength)
    {
        if (ActiveLibraryDirectory == null) return null;
        using var message = new MemoryStream(checked(values.Length * sizeof(double) + 8));
        using var writer = new BinaryWriter(message);
        writer.Write(opcode);
        writer.Write(values.Length);
        writer.Write(MemoryMarshal.AsBytes(values));
        return Doubles(Run(message, checked(resultLength * sizeof(double))));
    }

    // 按工作进程写入的 double 布局读回应答，复制后数组不再依赖原始字节缓冲。
    private static double[] Doubles(byte[] bytes) => MemoryMarshal.Cast<byte,double>(bytes).ToArray();

    // 选择当前线程的工作表架构，复用相应子进程；锁覆盖写请求和读完整应答两端。
    private static byte[] Run(MemoryStream request, int responseBytes)
    {
        lock (Gate)
        {
            ref Process? active = ref (DesktopWorksheet ? ref desktopProcess : ref process);
            return SequenceWorkerChannel.Run(ref active, DesktopWorksheet ? DesktopWorker : Worker,
                ActiveLibraryDirectory, request, responseBytes);
        }
    }
}
