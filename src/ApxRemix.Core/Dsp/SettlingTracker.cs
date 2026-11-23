// 界面使用 dB 和稳定帧数，底层判定器接受线性读数。这个适配层把界面参数和结果接到 ApxReadingSettler，让算法与表头使用各自合适的单位。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>UI adapter: dB readings to the verified AP linear-reading settler.</summary>
public sealed class SettlingTracker
{
    private double _toleranceDb;
    private int _points;
    private SettlingAlgorithm _algorithm;
    private ApxReadingSettler _settler = null!;
    private double? _previous;
    private int _count;
    public double AveragedValue { get; private set; } = double.NaN;

    // 在显示层以 dB 指定波动容限，底层判稳器则逐帧比较线性幅度。
    public SettlingTracker(double toleranceDb = 0.05, int requiredStableFrames = 5,
        SettlingAlgorithm algorithm = SettlingAlgorithm.Flat)
    {
        _toleranceDb = toleranceDb; _points = requiredStableFrames; _algorithm = algorithm;
        Reset();
    }
    // 相同设置沿用原有历史。只有判据发生变化才开新窗口，避免同一窗口混用不同标准。
    public void Configure(SettlingAlgorithm algorithm, double toleranceDb, int requiredPoints)
    {
        if (_algorithm == algorithm && _toleranceDb == toleranceDb && _points == requiredPoints) return;
        _algorithm = algorithm; _toleranceDb = toleranceDb; _points = requiredPoints;
        Reset();
    }
    // 向判稳器送入一帧。极低电平或非有限值会清除历史，否则记录相邻两帧间的绝对 dB 差供显示。
    public SettlingStatus Update(double currentDb)
    {
        if (!double.IsFinite(currentDb) || currentDb <= -150)
        {
            Reset(); return new(false, 0, double.NaN, "NO SIGNAL");
        }
        double delta = _previous is { } previous ? Math.Abs(currentDb - previous) : 0;
        _previous = currentDb;
        // 在线性幅度域判稳和求平均后转回 dB。此处按点数判稳，周期等待为零，所以传入的 1 kHz 不改变所需点数。
        int state = _settler.Add(Math.Pow(10, currentDb / 20), 1000);
        AveragedValue = SignalGeneratorDsp.LinearToDbfs(_settler.Value);
        _count = Math.Min(_count + 1, Math.Max(1, _points));
        return new(state == 1, _count, delta, state == 1 ? "SETTLED" : "SETTLING");
    }
    // 重新创建空判稳窗口，将允许的 dB 差换成线性百分比，例如幅度允许增加一倍约对应 6.02 dB。
    public void Reset()
    {
        _count = 0; _previous = null; AveragedValue = double.NaN;
        _settler = new(new SweepSettlingConfig {
            Algorithm = _algorithm, DelayTimeMs = 0, DelayCycles = 0,
            SettlingTimeMs = 0, SettlingCycles = 0, Points = Math.Max(1, _points),
            TolerancePercent = (Math.Pow(10, Math.Max(0, _toleranceDb) / 20) - 1) * 100, Floor = 0
        }, 1, double.PositiveInfinity);
    }
}
