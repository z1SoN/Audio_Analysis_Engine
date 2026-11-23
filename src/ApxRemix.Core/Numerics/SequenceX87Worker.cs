using System;
using System.IO;

// Compiled as x86 .NET Framework IL. The JIT lowers Math.Cos, Math.Sin and
// Math.Atan2 to x87 instructions. APx's x86 worksheet uses the same instruction path.
internal static class SequenceX87Worker
{
    // 该工作进程需要以 32 位运行。标准流承载二进制命令，返回码供宿主识别传输是否正常。
    private static int Main()
    {
        if (Environment.Is64BitProcess) return 1;
        try
        {
            using (var input = new BinaryReader(new BufferedStream(Console.OpenStandardInput(), 65536)))
            using (var output = new BinaryWriter(new BufferedStream(Console.OpenStandardOutput(), 65536)))
            {
                // 每次取一对操作码和点数；在新操作码之前遇到 EOF 表示正常断开，零长度或负长度表示协议错误。
                while (true)
                {
                    int operation;
                    try { operation = input.ReadInt32(); }
                    catch (EndOfStreamException) { return 0; }
                    int length = input.ReadInt32();
                    if (length <= 0) throw new InvalidDataException("Expected a positive length.");
                    switch (operation)
                    {
                        // 命令 1 按窗种类返回逐样本权重；命令 2 接收复频格及偏移，只回一个正弦相位。
                        case 1:
                            WriteWindow(length, input.ReadInt32(), output);
                            break;
                        case 2:
                            if (length != 1) throw new InvalidDataException("Expected one phase result.");
                            output.Write(SequenceFrameworkToneMath.BinPhase(input.ReadDouble(), input.ReadDouble(), input.ReadDouble()));
                            break;
                        // 命令 3 从时域样本减去拟合正弦，输入参数是有效值、初相和帧内周期数。
                        case 3:
                            double rms = input.ReadDouble(), phase = input.ReadDouble(), cycles = input.ReadDouble();
                            double[] samples = ReadVector(input, length);
                            SequenceFrameworkToneMath.SubtractSine(samples, rms, phase, cycles);
                            WriteVector(output, samples);
                            break;
                        // 命令 4 就地处理功率谱的陷波，然后回传每个格；每条命令完成后刷新一次。
                        case 4:
                            double fundamentalBin = input.ReadDouble();
                            double[] power = ReadVector(input, length);
                            SequenceFrameworkToneMath.ApplyResidualNotch(power, fundamentalBin);
                            WriteVector(output, power);
                            break;
                        default:
                            throw new InvalidDataException("Unknown x87 operation.");
                    }
                    output.Flush();
                }
            }
        }
        // 错误写到标准错误并返回非零码，避免错误文本混进二进制数值应答。
        catch (Exception error)
        {
            Console.Error.WriteLine(error.Message);
            return 1;
        }
    }

    // 检查三种合法窗编号，每步按 2π/length 弧度推进，从 −π 起依次写出窗权重。
    private static void WriteWindow(int length, int kind, BinaryWriter output)
    {
        if (kind < 1 || kind > 3) throw new InvalidDataException("Unknown x87 window.");
        double step = 2 * Math.PI / length;
        for (int index = 0; index < length; index++)
        {
            double phase = index * step - Math.PI;
            output.Write(WindowValue(kind, phase));
        }
    }

    // Keep each transcendental result adjacent to its multiply. Storing it in
    // binary64 before multiplying discards the x87 register's extra precision.
    // Hann、三项与四项窗按各自权重叠加基角及整数倍角的余弦；计算在该工作进程中执行。
    private static double WindowValue(int kind, double phase)
    {
        switch (kind)
        {
            case 1: return 0.5 + Math.Cos(phase) * 0.5;
            case 2: return 0.42 + Math.Cos(phase) * 0.5 + Math.Cos(2 * phase) * 0.08;
            default: return 0.35875 + Math.Cos(phase) * 0.48829
                + Math.Cos(2 * phase) * 0.14128 + Math.Cos(3 * phase) * 0.01168;
        }
    }

    // 协议中的长度是 double 个数，逐项读取以供本次原地计算。
    private static double[] ReadVector(BinaryReader input, int length)
    {
        var values = new double[length];
        for (int index = 0; index < length; index++) values[index] = input.ReadDouble();
        return values;
    }

    // 按原顺序写出全部 double；调用它的命令分派在整条回复结束时刷新。
    private static void WriteVector(BinaryWriter output, double[] values)
    {
        foreach (double value in values) output.Write(value);
    }
}
