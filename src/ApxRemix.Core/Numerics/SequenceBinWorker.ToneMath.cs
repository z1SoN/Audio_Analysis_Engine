using System.IO;

internal static partial class SequenceBinWorker
{
    // 处理相位和残差命令，在当前工作进程内调用 Framework 数学实现，共用写好的二进制应答流。
    private static void ToneMathOperation(int operation, int length, BinaryReader input, BinaryWriter output)
    {
        switch (operation)
        {
            // −6 的三个参数依次为虚部、实部和分数格；只回传一个相位标量。
            case -6:
                if (length != 3) throw new InvalidDataException("Expected complex bin and fractional bin");
                output.Write(SequenceFrameworkToneMath.BinPhase(input.ReadDouble(), input.ReadDouble(), input.ReadDouble()));
                break;
            // −7 读取 RMS、初相、帧内周期数，随后逐样本减正弦并返回整段残差。
            case -7:
                double rms = input.ReadDouble(), phase = input.ReadDouble(), cycles = input.ReadDouble();
                var samples = ReadVector(input, length);
                SequenceFrameworkToneMath.SubtractSine(samples, rms, phase, cycles);
                WriteVector(output, samples, length);
                break;
            // −8 读取基波位置和整段功率，原地应用陷波后返回同样数量的功率格。
            case -8:
                double fundamentalBin = input.ReadDouble();
                var power = ReadVector(input, length);
                SequenceFrameworkToneMath.ApplyResidualNotch(power, fundamentalBin);
                WriteVector(output, power, length);
                break;
            default: throw new InvalidDataException("Unknown transform operation");
        }
    }
}
