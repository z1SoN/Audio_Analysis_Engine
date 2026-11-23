// 这里解决的是时间问题。上游传来交流采样及上一读数的幅度尺度，本类保存合格上升沿的时刻和周期数；调用方再把两通道的时间差换成相位。
namespace ApxRemix.Core.Dsp;

/// <summary>
/// Independent port of APxRT 10.0.0.17 frequency/phase crossing detector.
/// Native addresses: 10007f60 (hysteresis/crossing capture), 10007ca1
/// (frequency refinement), 10007dfb (sine crossing interpolation).
/// The amplitude argument is the previous reading's RMS-equivalent sine peak.
/// </summary>
public sealed class ApxFrequencyPhaseDetector
{
    private bool _armed, _hasAnchor, _hasCrossing, _hasFrequency;
    private double _time, _previousSample, _anchor, _frequency;
    private int _cycles;
    private Crossing _crossing, _lastComplete;
    private readonly record struct Crossing(double Negative, double Positive, int Cycles,
        double Anchor, double End, double Peak, double SamplePeriod);

    // 连续流入口直接在零电平处找上升交越。峰值估计来自上一读数，因而同样的一包样本可能有不同触发门限。
    public void ProcessBlock(ReadOnlySpan<double> samples, int sampleRate, double previousReadingPeak)
        => ProcessBlock(samples, sampleRate, previousReadingPeak, 0);

    // 快照和连续流共用这个计数器。dc 是交越前要减去的偏置，_time 以秒跨包单调累加。
    private void ProcessBlock(ReadOnlySpan<double> samples, int sampleRate, double previousReadingPeak, double dc)
    {
        double period = 1.0 / sampleRate;
        // 一次可信的上升沿需要先低于负门限、跨零，再高于正门限。门限取上一读数等效峰值的 8%，
        // 零附近轻微抖动时，符号可能变来变去，却难以完成这整段行程。
        double threshold = previousReadingPeak * 0.08;
        foreach (double sample in samples)
        {
            double value = sample - dc;
            _time += period;
            // 先跌过负门限才武装。仅在零附近来回抖动的信号无法反复完成整个门限序列。
            if (!_armed)
            {
                if (value < -threshold) _armed = true;
            }
            else
            {
                // 上一负样本和当前非负样本夹住了过零时刻，先保存二者幅度和时标，等正门限确认。
                if (value >= 0 && _previousSample < 0)
                    _crossing = new(-_previousSample, value, _cycles + 1, _anchor, _time, previousReadingPeak, period);
                if (value > threshold)
                {
                    _armed = false;
                    // 上一个锚点存在时才有周期可计。首次确认只建立计时起点，以后的确认把完整周期纳入估计。
                    if (_hasAnchor)
                    {
                        _cycles++;
                        _hasCrossing = true;
                        _lastComplete = _crossing;
                    }
                    else
                    {
                        _hasAnchor = true;
                        _anchor = _time;
                    }
                }
            }
            _previousSample = value;
        }
    }

    // 短记录先用 FFT 定位。若可见周期足够多，才尝试用两个半段之间的交越时差细化频率。
    internal static double RefineSnapshotFrequency(ReadOnlySpan<double> samples, int sampleRate, double dc,
        double peak, double spectralHz, double binWidth)
    {
        // Seed the same crossing-to-crossing clock used by the live meter. Two reads
        // give both ends of the second interval a sub-sample crossing timestamp.
        if (!(spectralHz > 0) || spectralHz * samples.Length < 4d * sampleRate) return spectralHz;
        var counter = new ApxFrequencyPhaseDetector();
        int middle = samples.Length / 2;
        counter.ProcessBlock(samples[..middle], sampleRate, peak, dc);
        var start = counter.Read();
        counter.ProcessBlock(samples[middle..], sampleRate, peak, dc);
        var end = counter.Read();
        // With noise or multiple tones the crossing counter can lock to another
        // component. In that case preserve the dominant spectral reading.
        // 后半段既要出现新的交越，又要与谱峰相差不超过一个频格，否则沿用频域估计。
        return start.FrequencyHz > 0 && end.CrossingTimeSeconds > start.CrossingTimeSeconds &&
            Math.Abs(end.FrequencyHz - spectralHz) <= binWidth ? end.FrequencyHz : spectralHz;
    }

    // 以最近一次完成的交越为窗口末端，频率等于窗口内周期数除以经过的秒数。读取也会移动下一窗口的计时锚点。
    public (double FrequencyHz, double CrossingTimeSeconds) Read()
    {
        if (!_hasCrossing) return (0, 0);
        var c = _lastComplete;
        double initial = _hasFrequency ? _frequency : c.Cycles / (c.End - c.Anchor);
        double crossing = c.End - Interpolate(c, initial);
        _frequency = c.Cycles / (crossing - c.Anchor);
        _anchor = crossing;
        _hasFrequency = true;
        _cycles = 0;
        return (_frequency, crossing);
    }

    // 若局部波形近似正弦，靠近零点的样本满足 |x|/峰值 = sin(2πfΔt)。用它把整采样时标细分为秒。
    private static double Interpolate(Crossing c, double frequency)
    {
        if (c.Peak == 0 || frequency == 0) return 0.5 * c.SamplePeriod;
        bool closerToCurrent = c.Negative > c.Positive;
        double small = closerToCurrent ? c.Positive : c.Negative;
        double ratio = small > c.Peak ? 1 : small / c.Peak;
        // 零点通常落在两个采样点之间。局部近似正弦时，|x|/峰值给出 sin(角度)，反正弦得弧度，
        // 再除以角速度 2πf，才得到离零点的秒数。若修正量超过一个采样间隔，后面改用两点直线插值。
        // 小幅度的一侧较接近交越。若反推时差超出两样本之间的区间，就用两端幅度的线性比例作回退。
        double correction = Math.Asin(ratio) / (2 * Math.PI * frequency);
        if (correction <= c.SamplePeriod)
            return closerToCurrent ? correction : c.SamplePeriod - correction;
        return c.SamplePeriod * c.Positive / (c.Positive + c.Negative);
    }

    // 相同频率下，测量声道较早过零代表相位超前。秒差乘参考 Hz 得周期差，再折回 [−180°,180°)。
    public static double RelativePhaseDeg(double referenceCrossing, double measuredCrossing, double referenceFrequency)
    {
        // 同频时，若被测上升沿早到 0.25 ms，在 1 kHz 下就是领先 0.25 周，即 +90°。
        // 下面折回半开区间 [−180°,180°)，因此只凭相位读数无法辨别相差的整周期数。
        double cycles = (referenceCrossing - measuredCrossing) * referenceFrequency;
        double degrees = (cycles - Math.Floor(cycles)) * 360;
        return degrees >= 180 ? degrees - 360 : degrees;
    }
}
