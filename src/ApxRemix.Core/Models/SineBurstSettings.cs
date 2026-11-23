namespace ApxRemix.Core.Models;

public enum BurstLengthUnit { Cycles, Seconds }

public sealed record SineBurstSettings
{
    public BurstLengthUnit Unit { get; init; }
    public int HighCycles { get; init; } = 10;
    public int IntervalCycles { get; init; } = 40;
    public double HighSeconds { get; init; } = .01;
    public double IntervalSeconds { get; init; } = .04;
    public bool LowMuted { get; init; } = true;
    public double LowDb { get; init; } = -40;

    public (double High, double Interval) SampleLengths(int rate,double phaseStep)
    {
        // 若以秒指定，乘 Fs 得到持续点数。总间隔至少比高电平段多一个样本，才能为低电平段留出位置。
        if (Unit == BurstLengthUnit.Seconds)
        {
            double high = Math.Max(1,HighSeconds*rate);
            return (high,Math.Max(high+1,IntervalSeconds*rate));
        }
        // 每点推进 phaseStep 弧度，2π/phaseStep 就是一周期的样本数，乘周期数即可得到高电平段和重复间隔。
        double samplesPerCycle = 2*Math.PI / phaseStep;
        int cycles = Math.Clamp(HighCycles,1,16777214);
        return (cycles*samplesPerCycle, Math.Clamp(IntervalCycles,cycles+1,16777215)*samplesPerCycle);
    }
}
