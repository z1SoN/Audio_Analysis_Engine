namespace ApxRemix.Core.Dsp;

internal static partial class SequenceBinEstimator
{
    // 把扫频发生器参数送往所选工作进程，返回 length 个时域样本。缺少运行库时由调用者选其他生成路径。
    internal static double[]? GenerateChirp(int length, double t0, double t3, double f1, double f2, double f3, double step,
        double signalEndTime, double? rightTaperStart)
    {
        if (ActiveLibraryDirectory == null) return null;
        using var message = new MemoryStream();
        using var writer = new BinaryWriter(message);
        // 负操作码区分扫频命令与按正帧长开头的测频命令；随后各量按工作端的读取顺序写入。
        writer.Write(-10);
        writer.Write(length);
        writer.Write(t0);
        writer.Write(t3);
        writer.Write(f1);
        writer.Write(f2);
        writer.Write(f3);
        writer.Write(step);
        writer.Write(signalEndTime);
        // 先传右端渐弱起点是否存在，仅存在时再传其数值，避免工作端读错下一条消息。
        writer.Write(rightTaperStart.HasValue);
        if (rightTaperStart.HasValue) writer.Write(rightTaperStart.Value);
        return Doubles(Run(message, checked(length * sizeof(double))));
    }
}
