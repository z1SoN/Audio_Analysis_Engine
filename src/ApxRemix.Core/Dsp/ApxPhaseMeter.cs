// 把两个交流调理器、RMS 检波器和过零检测器接成可单独验证的相位表。Process 推进采样，Read 结算读数；设备回调在何处切成一帧，由外层安排。
namespace ApxRemix.Core.Dsp;

/// <summary>
/// Independent reproduction of the original frequency/phase path: AcCoupling ->
/// continuous RMS envelope -> 8% hysteresis crossings -> sine interpolation ->
/// time/frequency-to-phase conversion. All states survive Process/Read calls.
/// Signal-path HP/LP/weighting belongs upstream, not inside this component.
/// </summary>
public sealed class ApxPhaseMeter
{
    private readonly int _sampleRate;
    private readonly ApxAcCouplingFilter _ac1, _ac2;
    private readonly ApxRmsDetector _rms1, _rms2;
    private readonly ApxFrequencyPhaseDetector _phase1 = new(), _phase2 = new();
    private double _previousPeak1, _previousPeak2;
    // 左右声道有各自的耦合、RMS 和计数状态。采样率给交越计时，读数率参与确定 RMS 的动态响应。
    public ApxPhaseMeter(int sampleRate, double readingsPerSecond = 20)
    {
        _sampleRate = sampleRate;
        _ac1 = new(sampleRate); _ac2 = new(sampleRate);
        _rms1 = new(sampleRate, readingsPerSecond); _rms2 = new(sampleRate, readingsPerSecond);
    }
    // 同步处理相等长度的两通道。先滤除直流，再从同一交流波形分别估幅和记录过零时刻。
    public void Process(ReadOnlySpan<double> left, ReadOnlySpan<double> right)
    {
        if (left.Length != right.Length) throw new ArgumentException("Channel length mismatch.");
        var l = new double[left.Length]; var r = new double[right.Length];
        _ac1.Process(left, l); _ac2.Process(right, r);
        _rms1.Process(l); _rms2.Process(r);
        _phase1.ProcessBlock(l, _sampleRate, _previousPeak1);
        _phase2.ProcessBlock(r, _sampleRate, _previousPeak2);
    }
    // 报告一次 RMS 和交越时刻，同时为下一次处理留下峰值估计。乘 √2 依据正弦的峰值/RMS 关系。
    public ApxPhaseReading Read()
    {
        double rms1 = _rms1.Read(), rms2 = _rms2.Read();
        _previousPeak1 = rms1 * Math.Sqrt(2); _previousPeak2 = rms2 * Math.Sqrt(2);
        var a = _phase1.Read(); var b = _phase2.Read();
        // 两路都有频率估计时才有相对相位，参考一路的频率把交越秒差转换成角度。
        return new(a.FrequencyHz, b.FrequencyHz, a.CrossingTimeSeconds, b.CrossingTimeSeconds, rms1, rms2,
            a.FrequencyHz > 0 && b.FrequencyHz > 0
                ? ApxFrequencyPhaseDetector.RelativePhaseDeg(a.CrossingTimeSeconds, b.CrossingTimeSeconds, a.FrequencyHz) : null);
    }
}

public sealed record ApxPhaseReading(double Ch1FrequencyHz, double Ch2FrequencyHz, double Ch1CrossingTime,
    double Ch2CrossingTime, double Ch1RmsLinear, double Ch2RmsLinear, double? Ch2RelativeToCh1Deg);
