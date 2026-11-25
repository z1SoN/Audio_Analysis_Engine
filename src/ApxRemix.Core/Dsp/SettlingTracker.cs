// 界面使用 dB 和稳定帧数，底层判定器接受线性读数。支持完整 APx 逐仪表判稳参数（时间/周期延时、判稳窗、Floor 与超时 T 标记）。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>UI adapter: dB or linear readings to the verified AP linear-reading settler.</summary>
public sealed class SettlingTracker
{
    private readonly record struct DetailedSettlingSnapshot(
        SettlingAlgorithm Algorithm,
        double DelayTimeMs,
        double DelayCycles,
        double SettlingTimeMs,
        double SettlingCycles,
        int Points,
        double TolerancePercent,
        double Floor);

    private double _toleranceDb;
    private int _points;
    private SettlingAlgorithm _algorithm;
    private SweepSettlingConfig? _detailedConfig;
    private DetailedSettlingSnapshot? _detailedSnapshot;
    private double _readingRateHz = 1.0;
    private double _timeoutSeconds = double.PositiveInfinity;
    private ApxReadingSettler _settler = null!;
    private double? _previous;
    private int _count;
    public double AveragedValue { get; private set; } = double.NaN;
    public bool TimedOut { get; private set; }

    public SettlingTracker(double toleranceDb = 0.05, int requiredStableFrames = 5,
        SettlingAlgorithm algorithm = SettlingAlgorithm.Flat)
    {
        _toleranceDb = toleranceDb; _points = requiredStableFrames; _algorithm = algorithm;
        Reset();
    }

    public void Configure(SettlingAlgorithm algorithm, double toleranceDb, int requiredPoints)
    {
        if (_detailedConfig is null && _algorithm == algorithm && _toleranceDb == toleranceDb && _points == requiredPoints) return;
        _detailedConfig = null;
        _detailedSnapshot = null;
        _algorithm = algorithm; _toleranceDb = toleranceDb; _points = requiredPoints;
        _readingRateHz = 1.0;
        _timeoutSeconds = double.PositiveInfinity;
        Reset();
    }

    /// <summary>使用完整 APxRT 判稳参数（时间/周期延时、判稳时间/周期、点数、相对容差 %、线性下限 Floor、读数率与超时秒数）。仅在参数实际改变时复位。</summary>
    public void ConfigureDetailed(SweepSettlingConfig config, double readingRateHz, double timeoutSeconds)
    {
        double rate = readingRateHz > 0 ? readingRateHz : 1.0;
        double timeout = timeoutSeconds > 0 ? timeoutSeconds : double.PositiveInfinity;
        var snap = new DetailedSettlingSnapshot(
            config.Algorithm,
            config.DelayTimeMs,
            config.DelayCycles,
            config.SettlingTimeMs,
            config.SettlingCycles,
            Math.Max(1, config.Points),
            config.TolerancePercent,
            config.Floor);
        if (_detailedSnapshot == snap && _readingRateHz == rate && _timeoutSeconds == timeout)
            return;

        _detailedSnapshot = snap;
        _detailedConfig = new SweepSettlingConfig
        {
            Algorithm = snap.Algorithm,
            DelayTimeMs = snap.DelayTimeMs,
            DelayCycles = snap.DelayCycles,
            SettlingTimeMs = snap.SettlingTimeMs,
            SettlingCycles = snap.SettlingCycles,
            Points = snap.Points,
            TolerancePercent = snap.TolerancePercent,
            Floor = snap.Floor
        };
        _algorithm = snap.Algorithm;
        _points = snap.Points;
        _readingRateHz = rate;
        _timeoutSeconds = timeout;
        Reset();
    }

    public SettlingStatus Update(double currentDb, double frequencyHz = 1000.0)
    {
        if (!double.IsFinite(currentDb) || currentDb <= -150)
        {
            Reset(); return new(false, 0, double.NaN, "NO SIGNAL");
        }
        double delta = _previous is { } previous ? Math.Abs(currentDb - previous) : 0;
        _previous = currentDb;
        int state = _settler.Add(Math.Pow(10, currentDb / 20), frequencyHz > 0 ? frequencyHz : 1000.0);
        AveragedValue = SignalGeneratorDsp.LinearToDbfs(_settler.Value);
        TimedOut = state == 2;
        _count = Math.Min(_count + 1, Math.Max(1, _points));
        string label = state switch { 1 => "SETTLED", 2 => "TIMEOUT (T)", _ => "SETTLING" };
        return new(state == 1, _count, delta, label);
    }

    public void Reset()
    {
        _count = 0; _previous = null; AveragedValue = double.NaN; TimedOut = false;
        var cfg = _detailedConfig ?? new SweepSettlingConfig
        {
            Algorithm = _algorithm,
            DelayTimeMs = 0,
            DelayCycles = 0,
            SettlingTimeMs = 0,
            SettlingCycles = 0,
            Points = Math.Max(1, _points),
            TolerancePercent = (Math.Pow(10, Math.Max(0, _toleranceDb) / 20) - 1) * 100,
            Floor = 0
        };
        _settler = new(cfg, _readingRateHz, _timeoutSeconds);
    }
}

/// <summary>
/// Bench 逐仪表、逐声道独立判稳银行（对齐 APxRT InitSettling / AddNewReadingToSettler / GetSettledValues）。
/// </summary>
public sealed class BenchMeterSettlingBank
{
    private readonly Dictionary<(BenchMeterKind Meter, int Channel), ApxReadingSettler> _settlers = new();
    private readonly Dictionary<(BenchMeterKind Meter, int Channel), (int State, double Value)> _latest = new();

    public void Configure(BenchMeterKind meter, int channel, SweepSettlingConfig config,
        double readingsPerSecond, double timeoutSeconds)
    {
        var key = (meter, channel);
        _settlers[key] = new ApxReadingSettler(config, readingsPerSecond, timeoutSeconds);
        _latest[key] = (0, double.NaN);
    }

    public (int State, double Value) AddReading(BenchMeterKind meter, int channel, double linearValue, double frequencyHz)
    {
        var key = (meter, channel);
        if (!_settlers.TryGetValue(key, out var settler))
            return (1, linearValue);
        int state = settler.Add(linearValue, frequencyHz);
        var result = (state, settler.Value);
        _latest[key] = result;
        return result;
    }

    public (int State, double Value) GetLatest(BenchMeterKind meter, int channel) =>
        _latest.TryGetValue((meter, channel), out var value) ? value : (0, double.NaN);

    public bool AllSettled => _latest.Count > 0 && _latest.Values.All(v => v.State == 1);
    public bool AnyTimedOut => _latest.Values.Any(v => v.State == 2);
}
