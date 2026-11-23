// 主路径经过带宽和计权选择后，从同一段波形读取有符号的直流和无符号的峰值。StreamingBenchPipeline 先取这两项，之后才在工作缓冲中压低基波。
namespace ApxRemix.Core.Dsp;

/// <summary>Signed DC reading after the main signal path, averaged across one reporting interval.</summary>
internal sealed class DigitalDcMeter
{
    private readonly double _a1, _a2, _scale;
    private double _z1, _z2;
    private double _sum, _peak;
    private int _count;

    // 以采样率折算直流通道的窄带低通。慢变分量可以保留，交流成分随时间衰减。
    public DigitalDcMeter(int sampleRate)
    {
        double bandwidth = 4.0 / sampleRate;
        double warp = Math.Sin((0.5 - bandwidth) * (Math.PI / 2)) / Math.Sin((0.5 + bandwidth) * (Math.PI / 2));
        double prototype = 0.17157287525381;
        double norm = 1 / (1 + (warp * warp) * prototype);
        _a1 = -2 * warp * (1 + prototype) * norm;
        _a2 = (warp * warp + prototype) * norm;
        // 用常量 C 检查增益。稳态下三个输入项加成 (1+2+1)C，反馈移到左边得到 (1+a1+a2)y=4C。
        // 乘下面的系数，恒定输入最后便按原幅值读出。刚加台阶时，稳态等式还不成立。
        // 稳态直流输入时所有延时项相等，分子三项之和为 4。用其与分母和的比值校准直流增益。
        _scale = (1 + _a1 + _a2) / 4;
    }

    // 一路记录未经本级低通的绝对峰值，另一路累加低通后的有符号输出，得到两个不同物理含义的读数。
    public void Process(ReadOnlySpan<double> samples)
    {
        foreach (double x in samples)
        {
            // 峰值看当前样本的绝对值，负尖峰也能刷新它。DC 保留符号，并用 _z1、_z2 承接之前的样本。
            // 所以同一个负台阶能让峰值立刻变大，同时让 DC 读数逐帧向负值移动。
            _peak = Math.Max(_peak, Math.Abs(x));
            double y = x + _z1;
            _sum += y;
            _z1 = -_a1 * y + _z2 + 2 * x;
            _z2 = x - _a2 * y;
        }
        _count += samples.Length;
    }

    // 以样本数归一并恢复直流增益。报告后清掉峰值与累计和，低通的记忆跨读数延续。
    public (double Dc, double Peak) Read()
    {
        var result = (_count == 0 ? 0 : _sum * (_scale / _count), _peak);
        _count = 0;
        _sum = _peak = 0;
        return result;
    }
}
