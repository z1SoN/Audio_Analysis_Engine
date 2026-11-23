// 扫频切换频率后，第一笔读数可能还带着前一频点的影响。这里根据连续到来的读数判断是否已满足稳定条件，使选取结果的规则可以脱离设备单独验证。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APxRT 10.0.0.17 single-reading settler port. Addresses 10010800/10010970/
/// 10010b40/10010d70: ceil-to-reading counts, additive time/cycles, linear-unit
/// averaging, full-window Flat test, 1250-reading cap and timeout state.
/// Verified against the installed DLL; see settling-fixtures.csv.
/// </summary>
public sealed class ApxReadingSettler
{
    private readonly SweepSettlingConfig _config;
    private readonly double _rate;
    private readonly double _timeout;
    private readonly double _relativeTolerance;
    private readonly double[] _window = new double[1250];
    private int _readings, _cursor;
    private double _elapsed;
    public double Value { get; private set; }

    // 判稳器接收定时产生的标量读数。读数率把等待秒数换成读数个数，超时也按同一时钟计算。
    public ApxReadingSettler(SweepSettlingConfig config, double readingsPerSecond, double timeoutSeconds)
    {
        _config = config;
        _rate = readingsPerSecond > 0 ? readingsPerSecond : throw new ArgumentOutOfRangeException(nameof(readingsPerSecond));
        _timeout = timeoutSeconds;
        // Preserve the original managed percentage -> dB -> native ratio path.
        _relativeTolerance = Math.Pow(10, 20 * Math.Log10(1 + config.TolerancePercent / 100) * 0.05) - 1;
    }

    // 每次送入一个线性读数及当前 Hz。返回 0 表示继续收集，1 表示满足策略，2 表示已到超时时间。
    /// <returns>0 waiting, 1 settled, 2 timeout, matching APxRT SettlerState.</returns>
    public int Add(double value, double frequencyHz)
    {
        _elapsed += 1 / _rate;
        if (_elapsed >= _timeout) return 2;
        Value = value;
        _readings++;
        // 无判稳策略时可立即接受有效读数。启用策略时，面板给定的周期数需先乘当前周期长度。
        if (_config.Algorithm == SettlingAlgorithm.None) return double.IsNaN(value) ? 0 : 1;
        double period = frequencyHz == 0 ? 0 : 1 / frequencyHz;
        int delay = Count((_config.DelayTimeMs / 1000 + _config.DelayCycles * period) * _rate);
        if (_readings < delay) return 0;
        // 窗口既要容纳指定点数，也要覆盖规定的持续时间。环形写入保留最近的数据。
        int length = Math.Min(1250, Math.Max(_config.Points,
            Count((_config.SettlingTimeMs / 1000 + _config.SettlingCycles * period) * _rate)));
        length = Math.Max(1, length);
        _window[_cursor] = value;
        _cursor = (_cursor + 1) % length;
        if (_readings - delay < length) return 0;
        // 以当前值为基准，容差取百分比允许量和绝对下限中较大者，低电平时仍有可用比较范围。
        bool finite = !double.IsNaN(value);
        // AP tests every preceding point against the newest reading, not merely
        // each adjacent pair. Floor and the relative limit are in linear units.
        double tolerance = Math.Max(value * _relativeTolerance, _config.Floor);
        double sum = finite ? value : 0;
        int count = finite ? 1 : 0;
        bool flat = finite;
        // 扫描其余历史点，缺失值会使平坦判据失败，计算临时平均时跳过缺失项。
        for (int i = 0; i < length - 1; i++)
        {
            double previous = _window[(_cursor + i) % length];
            if (double.IsNaN(previous)) { flat = false; continue; }
            sum += previous;
            count++;
            if (Math.Abs(previous - value) > tolerance) flat = false;
        }
        // Flat 成立就报告当前读数，其他情况下给出现有有效点的均值。Average 仍须窗口填满有效值才能完成。
        Value = _config.Algorithm == SettlingAlgorithm.Flat && flat
            ? value : count == 0 ? double.NaN : sum / count;
        return (_config.Algorithm == SettlingAlgorithm.Average ? finite && count == length : flat) ? 1 : 0;
    }

    // 将非负的读数个数近似向上取整，并限定在历史缓冲区容量以内。
    private static int Count(double value) => Math.Min(1250, (int)(value + 0.999999999999));
}
