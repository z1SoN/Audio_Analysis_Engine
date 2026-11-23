using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed partial class SignalGeneratorDsp
{
    private GeneratorWaveform _patternKind;
    private uint _bitTestState;
    private long _walkPosition;

    // 重新选择测试图样时恢复初始位状态，让接收端有确定的起点去辨认位移和丢位。
    private void ResetPattern(GeneratorWaveform waveform)
    {
        _patternKind=waveform;
        _bitTestState=0xfffffe;
        _walkPosition=0;
    }

    private void FillDigitalPattern(GeneratorConfig config,Span<double> left,Span<double> right)
    {
        var settings=config.WaveSettings;
        int bits=Math.Clamp(settings.DigitalBits,8,24);
        int repeat=Math.Max(1,settings.SamplesPerStep);
        long period=(long)repeat*bits;
        int a=settings.TruncateWord(settings.ConstantCh1);
        int b=settings.TrackConstant ? a : settings.TruncateWord(settings.ConstantCh2);
        const double fullScale=2147483648.0;
        // 直接构造数字码字。Bit Test 按状态产生图样，Walking 模式让一位沿字长移动；Walking Zeros 再按位取反。
        for(int i=0;i<left.Length;i++)
        {
            if (config.Waveform==GeneratorWaveform.BitTest)
            {
                // 24-bit CRC feedback, aligned to the high bits of a signed PCM word.
                // 反馈移位寄存器根据最高位选择是否异或抽头掩码，截成 24 位后左对齐到 32 位 PCM 字。
                uint taps=(_bitTestState&0x800000)!=0 ? 0x8c76efu : 0;
                _bitTestState=((_bitTestState<<1)^taps)&0xffffff;
                a=b=settings.TruncateWord(unchecked((int)(_bitTestState<<8)));
            }
            else if(config.Waveform!=GeneratorWaveform.ConstantValue)
            {
                int word=1<<(int)((_walkPosition/repeat)%bits);
                if(config.Waveform==GeneratorWaveform.WalkingZeros) word=~word;
                a=b=word<<(32-bits);
                if(++_walkPosition>=period) _walkPosition=0;
            }
            // 带符号的 32 位码字除以 2³¹ 得归一化 PCM。整字图样这样可沿用浮点音频接口，同时保留各有效位。
            left[i]=config.Channel1Enabled ? a/fullScale : 0;
            right[i]=config.Channel2Enabled ? b/fullScale : 0;
        }
    }

    private void FillTwoTones(GeneratorConfig cfg,double incA,double incB,double ratio,DualToneRoutingMode routing,
        GeneratedSignal first,GeneratedSignal second,double amp1,double amp2,double phase2,Span<double> left,Span<double> right)
    {
        double Sample(GeneratedSignal route,double amplitude,double phase)
        {
            // 分路时每路各取一个音，求和时按 1+ratio 缩放。两个正弦偶尔会峰值同向，相加仍要服从总幅度预算。
            if (routing==DualToneRoutingMode.Split)
                return route==GeneratedSignal.A ? amplitude*Math.Sin(_phasePrimary+phase) :
                    Math.Min(1,amplitude*ratio)*Math.Sin(_phaseSecondary+phase);
            return amplitude*(Math.Sin(_phasePrimary+phase)+ratio*Math.Sin(_phaseSecondary+phase))/(1+ratio);
        }
        // 两个振荡器每采样各推进一次，两路输出只是在公共振荡器上取不同的音和相位。
        for(int i=0;i<left.Length;i++)
        {
            left[i]=cfg.Channel1Enabled ? Sample(first,amp1,0) : 0;
            right[i]=cfg.Channel2Enabled ? Sample(second,amp2,phase2) : 0;
            _phasePrimary=WrapPhase(_phasePrimary+incA);
            _phaseSecondary=WrapPhase(_phaseSecondary+incB);
        }
    }
}
