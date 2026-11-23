namespace ApxRemix.Core.Models;

public enum GeneratedSignal { A, B }
public enum SmpteStimulus { Smpte4To1, Smpte10To1, Smpte1To1, Din }

public sealed record GeneratorWaveSettings
{
    // A、B 是两种发生器成分，各输出通道可各自选取；文件通道在这里按用户看到的 1、2 编号。
    public GeneratedSignal FreqCh1 { get; init; }
    public GeneratedSignal FreqCh2 { get; init; } = GeneratedSignal.B;
    public GeneratedSignal PhaseCh1 { get; init; }
    public GeneratedSignal PhaseCh2 { get; init; } = GeneratedSignal.B;
    public GeneratedSignal ImdCh1 { get; init; }
    public GeneratedSignal ImdCh2 { get; init; } = GeneratedSignal.B;
    public int FileCh1 { get; init; } = 1;
    public int FileCh2 { get; init; } = 2;
    // 位数决定数字序列可用的码字宽度，步长保持按样本计算。常量码可由两路各自设置，或让第二路跟踪第一路。
    public int DigitalBits { get; init; } = 24;
    public int SamplesPerStep { get; init; } = 1;
    public bool TrackConstant { get; init; } = true;
    public int ConstantCh1 { get; init; } = 0x40000000;
    public int ConstantCh2 { get; init; } = 0x40000000;
    public SmpteStimulus SmpteType { get; init; } = SmpteStimulus.Smpte4To1;
    public DualToneRoutingMode ImdMode { get; init; }
    public double SmpteLowHz { get; init; } = 60;
    public double SmpteHighHz { get; init; } = 7000;
    public double DinLowHz { get; init; } = 250;
    public double DinHighHz { get; init; } = 8000;
    // Preserve the original Remix 19 + 20 kHz stimulus when an older project has no settings.
    public double DfdMeanHz { get; init; } = 19500;
    public double DfdDifferenceHz { get; init; } = 1000;

    // CCIF 用中心频率和两音间隔反推出实际两音。例如 19.5 kHz、间隔 1 kHz 对应 19 与 20 kHz；SMPTE 则另外规定幅度比。
    public (double A,double B,double BtoA) Tones(GeneratorWaveform wave) => wave switch
    {
        GeneratorWaveform.ImdCcif => (DfdMeanHz-DfdDifferenceHz/2,DfdMeanHz+DfdDifferenceHz/2,1),
        _ when SmpteType == SmpteStimulus.Din => (DinLowHz,DinHighHz,.25),
        _ => (SmpteLowHz,SmpteHighHz,SmpteType switch {SmpteStimulus.Smpte1To1=>1,SmpteStimulus.Smpte10To1=>.1,_=>.25})
    };

    public bool CanGenerateTones(GeneratorWaveform wave,int rate)
    {
        var (a,b,_)=Tones(wave);
        // 两音须互异、有正频率并低于 Fs/2，否则离散样本不足以唯一表示配置的双音。
        return a>0 && b>a && b<rate*.5;
    }
    public static bool IsDigital(GeneratorWaveform w) =>
        w is GeneratorWaveform.BitTest or GeneratorWaveform.ConstantValue or GeneratorWaveform.WalkingOnes or GeneratorWaveform.WalkingZeros;

    // 数字图样在 32 位字的高位对齐；用位掩码保留设定位数，低位清零，便于检查通路是否真的保留了测试字。
    public int TruncateWord(int word) => word & (-1 << (32-Math.Clamp(DigitalBits,8,24)));
}
