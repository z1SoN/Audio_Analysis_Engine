namespace ApxRemix.Core.Models;

public enum BenchSweepSource
{
    GeneratorFrequency, GeneratorLevel, GeneratorPhase, GeneratorOffset,
    ExternalFrequency, DualToneA, DualToneB, ImdFrequency1, ImdFrequency2,
    DfdMeanFrequency, DfdDifferenceFrequency
}

public static class SweepNesting
{
    public static BenchSweepSource[] Options(BenchSweepSource primary)
    {
        // 主轴来自外部时本机不能控制它。内部调节量还要排除与主轴重合的选择，否则两层循环会同时写同一发生器参数。
        if (primary == BenchSweepSource.ExternalFrequency) return [];
        var sources = new List<BenchSweepSource> { BenchSweepSource.GeneratorLevel, BenchSweepSource.GeneratorOffset, BenchSweepSource.GeneratorPhase };
        sources.Remove(primary);
        var otherTone = primary switch
        {
            BenchSweepSource.DualToneA => BenchSweepSource.DualToneB,
            BenchSweepSource.DualToneB => BenchSweepSource.DualToneA,
            BenchSweepSource.ImdFrequency1 => BenchSweepSource.ImdFrequency2,
            BenchSweepSource.ImdFrequency2 => BenchSweepSource.ImdFrequency1,
            BenchSweepSource.DfdMeanFrequency => BenchSweepSource.DfdDifferenceFrequency,
            BenchSweepSource.DfdDifferenceFrequency => BenchSweepSource.DfdMeanFrequency,
            _ => BenchSweepSource.GeneratorFrequency
        };
        if (otherTone != primary) sources.Add(otherTone);
        return sources.ToArray();
    }

    // 同一个副轴值要随类型解释为 dBFS、Hz、度或满量程比例。曲线标签与参数校验使用同一套单位。
    public static string Unit(BenchSweepSource source) => source switch
    {
        BenchSweepSource.GeneratorOffset => "FS",
        BenchSweepSource.GeneratorLevel => "dBFS",
        BenchSweepSource.GeneratorPhase => "°",
        _ => "Hz"
    };
}
