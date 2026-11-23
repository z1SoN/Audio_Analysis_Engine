namespace ApxRemix.Core.Models;

public enum ReferenceDomain { Analog, Digital }

// Analog amplitudes use Vrms. Digital amplitudes use a fraction of full scale.
// 同一个幅度值在模拟域按 Vrms 解释，在数字域按满量程比例解释。把两组参考分别保存，才不会在换域时误用单位。
public sealed record ReferenceBank
{
    // A、B 独立保存电平基准及偏置，SPL 参考同时需要输入幅度与已知声压级（dB SPL）。
    public double A { get; init; } = 1;
    public double OffsetA { get; init; }
    public double B { get; init; } = 1;
    public double OffsetB { get; init; }
    public double Spl1 { get; init; } = 0.01;
    public double Spl1Level { get; init; } = 94;
    public double Spl2 { get; init; } = 0.01;
    public double Spl2Level { get; init; } = 94;
    public double Generator { get; init; } = 0.1;
}

// 在起止激励电平之间按设定的间隔数寻找目标，例如在固定频率上查找 THD+N 达到指定百分比的工作点。激励大小由发生器设定，达到目标与否由输入读数决定。
public sealed record RegulationOptions
{
    public string Metric { get; init; } = "RMS Level";
    public string Goal { get; init; } = "Target value";
    public string Channel { get; init; } = "Specific channel";
    public double StartDbfs { get; init; } = -40;
    public double StopDbfs { get; init; } = -6;
    public int Intervals { get; init; } = 2;
    public double FrequencyHz { get; init; } = 1000;
    public double TargetThdnPercent { get; init; } = 1;
}

// 电平显示、SPL 换算与调节要共用所选的参考条件；修改参考可以改变显示值而不改变原始样本。
public sealed record SignalReferenceSettings
{
    public RegulationOptions Regulation { get; init; } = new();
    public ReferenceBank Analog { get; init; } = new();
    public ReferenceBank Digital { get; init; } = new();
    public ReferenceDomain Input { get; init; }
    public ReferenceDomain Output { get; init; }
    // 同一电压换算为功率时，负载越小算得功率越大。输入与输出各保存 dBm、瓦数显示采用的阻值，单位都是 Ω。
    public double InputDbmOhms { get; init; } = 600;
    public double OutputDbmOhms { get; init; } = 600;
    public double InputWattsOhms { get; init; } = 8;
    public double OutputWattsOhms { get; init; } = 8;
    public bool AcousticOutput { get; init; }
    // 声学输出用 Pa/V 把电压换成声压。声压级相对 20 µPa 换算，选择的校准频率与目标声压级另行保存。
    public double AcousticPascalsPerVolt { get; init; } = 1;
    public double AcousticFrequencyHz { get; init; } = 1000;
    public double AcousticTargetSpl { get; init; } = 94;
    public double FrequencyHz { get; init; } = 1000;
}
