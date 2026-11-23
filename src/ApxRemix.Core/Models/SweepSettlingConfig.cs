// 相邻两个扫频点之间会有过渡。把容差、等待时间和取样要求装进配置，采集流程和稳定判定器就能共用同一组条件来决定何时记录该点。
namespace ApxRemix.Core.Models;

/// <summary>AP Bench settling controls; tolerance is percent, floor is linear full scale.</summary>
public sealed class SweepSettlingConfig
{
    public SettlingAlgorithm Algorithm { get; set; } = SettlingAlgorithm.Flat;
    // 固定毫秒表示与音调无关的等待，而固定周期数会让低频等待更久。例如 3 周期在 100 Hz 是 30 ms，在 1 kHz 是 3 ms。
    public double DelayTimeMs { get; set; } = 20;
    public double DelayCycles { get; set; } = 3;
    public double SettlingTimeMs { get; set; } = 60;
    public double SettlingCycles { get; set; }
    // Points 和容差约束一串读数的波动；接近零时相对变化很容易被放大，因此另存线性满量程单位的底限。
    public int Points { get; set; } = 3;
    public double TolerancePercent { get; set; } = 1;
    public double Floor { get; set; } = 0.000001;
}
