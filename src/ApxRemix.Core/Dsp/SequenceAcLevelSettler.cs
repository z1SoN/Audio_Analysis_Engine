using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>Sequence Level AC settling. All times are input sample-clock times in seconds.</summary>
public sealed class SequenceAcLevelSettler(SettlingAlgorithm algorithm, double stimulusCyclesSeconds)
{
    // 每个实例从空历史开始。window 用采样时钟的秒数衡量窗口，readings 留下每次读数覆盖的起点和时长。
    // APx500 State, default Level: 60 ms or three stimulus cycles, ±1%; Average: 200 ms.
    private readonly double window = Math.Max(.06, stimulusCyclesSeconds);
    private readonly List<(double Start, double Duration, double Value)> readings = [];
    public double Value { get; private set; } = double.NaN;
    public bool Finished { get; private set; }
    public bool TimedOut { get; private set; }

    // 送入一条覆盖 [end-duration, end] 的线性电平。Finished 一旦置位，后续采集就不能改写本次结果。
    public void Add(double value, double end, double duration)
    {
        // 非有限值或负值不能作为有效值参与平方平均，历史和计时都保持原样。
        if (Finished || !double.IsFinite(value) || value < 0) return;
        readings.Add((end - duration, duration, value));
        // 关闭判稳时第一条有效读数就能交付。
        if (algorithm == SettlingAlgorithm.None)
        {
            Value = value;
            Finished = true;
            return;
        }
        int first = readings.Count - 1;
        // Average 从最新一条往回累加各帧时长，满 200 ms 就对选中的电平求均方根。
        if (algorithm == SettlingAlgorithm.Average)
        {
            double elapsed = 0;
            for (; first >= 0; first--)
            {
                elapsed += readings[first].Duration;
                if (elapsed + 1e-12 < .2) continue;
                Finish(first, false);
                break;
            }
            return;
        }
        // 判稳至少需要三条记录。从当前电平往回找连续落在当前值 ±1% 内的区间。
        if (readings.Count < 3) return;
        for (; first >= 0; first--)
        {
            if (Math.Abs(readings[first].Value - value) > .01 * value) break;
            if (readings.Count - first >= 3 && end - readings[first].Start + 1e-12 >= window)
            {
                Finish(first, false);
                return;
            }
        }
        // 从首条记录算起已过十个窗口仍未判稳，退回到最近约五个窗口作超时读数。
        if (end - readings[0].Start + 1e-12 < 10*window) return;
        first = readings.Count - 1;
        while (first > 0 && end - readings[first].Start + 1e-12 < 5*window) first--;
        Finish(first, true);
    }

    // Finish 的 first 是纳入统计的最早一条。每条电平等权参与均方根，时长仅负责选择记录。
    private void Finish(int first, bool timeout)
    {
        double power = 0;
        for (int i = first; i < readings.Count; i++) power += readings[i].Value * readings[i].Value;
        Value = Math.Sqrt(power / (readings.Count - first));
        Finished = true;
        TimedOut = timeout;
    }
}
