using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public readonly record struct DigitalWordChannel(uint? Data, uint? Activity, double? ErrorRate);
public readonly record struct DigitalWordReading(int SampleRate, bool ExactPcm, DigitalWordChannel Left, DigitalWordChannel Right);

/// <summary>Input word monitors before any EQ, gain, weighting or resampling.</summary>
public sealed class DigitalWordMeter
{
    private struct Channel
    {
        private uint first, changes;
        private int badWords;
        private bool valid, hasReference;
        private uint? data, activity;
        private double? errorRate;
        // Data 随新样本更新。Activity 与错误率依赖报告窗口，缺失参考时无法提供错误率。
        public readonly DigitalWordChannel Reading => new(data,activity,hasReference ? errorRate : null);

        // 从归一化样本恢复 32 位字。以窗口第一个字为参照，对每一位检查它是否曾翻转。
        public void Add(double sample,double[]? reference,int i,DigitalPatternReference? pattern,bool start)
        {
            data=ToWord(sample);
            if (start)
            {
                first=data.GetValueOrDefault(); changes=0; badWords=0;
                valid=true; hasReference=true;
            }
            // XOR 把不同的位标为 1，OR 将变化累积到窗口末尾。无法转换的样本使本窗活动统计失效。
            if (data is uint word) changes |= word ^ first;
            else valid=false;
            // 仅针对可用的参考模式比较整字。某些全零或有效位全一的常量模式在这里禁用错误率。
            uint? expected=reference != null && i < reference.Length ? ToWord(reference[i]) : null;
            if (expected == null || pattern == null) { hasReference=false; return; }
            int shift=32-Math.Clamp(pattern.Depth,1,32);
            uint allOnes=uint.MaxValue << shift;
            if (pattern.Constant && (expected.Value==0 || expected.Value==allOnes)) hasReference=false;
            if (data!=expected) badWords++;
        }
        // 活动掩码标示窗口内曾改变的位。错误率是错误样本字数除以字数，每个错字只计一次。
        public void End(int count)
        {
            activity=valid ? changes : null;
            errorRate=valid && hasReference ? (double)badWords/count : null;
        }
    }
    private int rate, cursor;
    private bool exact, dual;
    private Channel left, right;
    // 将当前两路的状态连同采样率、精确 PCM 标识一起交给显示层。没有独立第二声道时右侧使用缺省值。
    public DigitalWordReading Reading => new(rate,exact,left.Reading,dual ? right.Reading : default);

    // 按采样率换算约 250 ms 的统计窗口。是否有独立右声道、字精度标识或采样率改变都会开始新窗。
    public void Add(StereoAudioBlock block)
    {
        if (block.SampleRate<=0) return;
        if (rate!=block.SampleRate || exact!=block.ExactPcm || dual!=block.HasIndependentDualChannels)
        {
            cursor=0; left=default; right=default; rate=block.SampleRate;
            exact=block.ExactPcm; dual=block.HasIndependentDualChannels;
        }
        int period=Math.Max(1,(int)Math.Round(rate/4.0));
        for (int i=0, count=Math.Min(block.Left.Length,block.Right.Length);i<count;i++)
        {
            var reference=block.DigitalPattern;
            left.Add(block.Left[i],reference?.Left,i,reference,cursor==0);
            if (dual) right.Add(block.Right[i],reference?.Right,i,reference,cursor==0);
            // 音频块可以跨越报告边界。恰好收满才结算活动和错误率，余下样本自然进入下一窗。
            if (++cursor==period)
            {
                left.End(period); right.End(period); cursor=0;
            }
        }
    }

    // 将 [−1,1] 换算为有符号 32 位定点数后保留位图。+1 饱和到最大正数，越界或非有限输入返回缺失。
    private static uint? ToWord(double sample)
    {
        if (!double.IsFinite(sample) || sample is < -1 or > 1) return null;
        return unchecked((uint)(int)Math.Clamp(Math.Round(sample*2147483648.0),int.MinValue,int.MaxValue));
    }
}
