using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed record StereoSweepDrive(double[] Left,double[] Right,bool LeftEnabled,bool RightEnabled)
{
    // With a single driven output, both inputs are referenced to that output.
    // 优先返回被选通道实际播放的波形。单路激励时另一输入也须相对唯一激励分析，不能拿一段静音作分母。
    public double[] Reference(bool ch2) => (ch2 && RightEnabled || !LeftEnabled) ? Right : Left;

    // Reconstruct the samples actually sent to the DAC on the ADC clock. Independently
    // regenerating a Fast Sweep at another rate can move its integer-cycle boundaries.
    public StereoSweepDrive AtSampleRate(int sourceRate,int targetRate)
    {
        if (sourceRate <= 0 || targetRate <= 0) throw new ArgumentOutOfRangeException(nameof(sourceRate));
        if (sourceRate == targetRate) return this;
        int count = checked((int)Math.Round(Left.Length * (double)targetRate / sourceRate));
        var left = new double[count];
        var right = ReferenceEquals(Left,Right) ? left : new double[count];
        // 降采样前限制可通过频带，避免新奈奎斯特频率以上的能量折回；用采样率的最大公约数确定重复的小数相位数。
        double cutoff = Math.Min(1.0,(double)targetRate/sourceRate);
        int radius = (int)Math.Ceiling(32/cutoff);
        int divisor=sourceRate, remainder=targetRate;
        while (remainder != 0) (divisor,remainder) = (remainder,divisor%remainder);
        // 相位数取率比约分后的输出端分子，超过 4096 时量化到 4096 档，避免为每一种分数位置建立过大的核表。
        int phaseCount=Math.Min(4096,targetRate/divisor);
        bool separateRight=RightEnabled && !ReferenceEquals(left,right);
        var phases = new Dictionary<int,double[]>();
        for (int i=0;i<count;i++)
        {
            long position = (long)i*sourceRate;
            int center = (int)(position/targetRate);
            int phase = (int)((position%targetRate)*phaseCount/targetRate);
            // 同一小数相位的截窗 sinc 插值权重可以复用。总权重归一为一，使恒定输入在完整窗内保持恒定。
            if (!phases.TryGetValue(phase,out var taps))
            {
                taps = new double[2*radius];
                double sum=0;
                for (int j=0;j<taps.Length;j++)
                {
                    double distance = j-radius+1-phase/(double)phaseCount;
                    double angle = Math.PI*distance;
                    double window = .42+.5*Math.Cos(angle/radius)+.08*Math.Cos(2*angle/radius);
                    taps[j] = window*(Math.Abs(angle)<1e-12 ? cutoff : Math.Sin(angle*cutoff)/angle);
                    sum+=taps[j];
                }
                for (int j=0;j<taps.Length;j++) taps[j]/=sum;
                phases.Add(phase,taps);
            }
            int start = center-radius+1;
            int first = Math.Max(0,-start), end = Math.Min(taps.Length,Left.Length-start);
            // 两路沿相同输入时间位置、使用相同权重重采样，保持通道间原有相对时序。
            double a=0,b=0;
            for (int j=first;j<end;j++)
            {
                if (LeftEnabled) a += Left[start+j]*taps[j];
                if (separateRight) b += Right[start+j]*taps[j];
            }
            left[i]=a;
            if (separateRight) right[i]=b;
        }
        return new(left,right,LeftEnabled,RightEnabled);
    }

    public static StereoSweepDrive Generate(ContinuousSweepConfig cfg,int rate,bool leftEnabled=true,bool rightEnabled=true)
    {
        if (!leftEnabled && !rightEnabled) throw new ArgumentException("Enable at least one sweep output.");
        var settings = cfg.Channels;
        var table = settings.EqTable;
        if (settings.EqMode != SweepGeneratorEqMode.None && !FrequencyGainTable.ValidTable(table))
            throw new ArgumentException("Sweep EQ needs an ordered frequency and gain table.");
        // 需要保留的是送往各通道的实际参考。先解开通道跟踪关系，再分别套用电平、均衡和直流偏置。
        double rightLevel = settings.TrackFirst ? cfg.LevelDbfs : settings.LevelCh2Dbfs;
        double rightDc = settings.TrackFirst ? settings.DcCh1 : settings.DcCh2;
        double[] GenerateChannel(double level,double dc)
        {
            if (!double.IsFinite(level) || !double.IsFinite(dc) || Math.Abs(dc) > 1)
                throw new ArgumentException("Invalid sweep level or DC offset.");
            var channel = cfg.Clone();
            channel.LevelDbfs = level;
            double[] samples;
            if (settings.EqMode == SweepGeneratorEqMode.None) samples = ContinuousSweepAnalyzer.GenerateExcitationChirp(channel,rate);
            else
            {
                // 绝对 EQ 把表的 dB 值直接作为该点 dBFS；相对 EQ 把表值叠加到该声道设定电平，再换成线性幅度。
                double BaseAt(double hz) => SignalGeneratorDsp.DbfsToLinearPeak(
                    (settings.EqMode == SweepGeneratorEqMode.Absolute ? 0 : level) + FrequencyGainTable.GainAtHz(table!,hz));
                // 离散 Fast Sweep 可直接调整各音的计划幅值；连续扫频则由样本时刻推回瞬时频率，再查该频率的 EQ 增益。
                if (cfg.ChirpType == ContinuousChirpType.FastSweep)
                {
                    var plan = BenchFastSweep.Plan(channel,rate);
                    samples = FastSweepGenerator.Generate(plan with { Amplitudes = Array.ConvertAll(plan.FrequenciesHz,BaseAt) }).Samples;
                }
                else
                {
                    channel.LevelDbfs = 0;
                    samples = ContinuousSweepAnalyzer.GenerateExcitationChirp(channel,rate);
                    double duration = Math.Clamp(cfg.DurationSeconds,0.1,15);
                    bool ap = cfg.ChirpType == ContinuousChirpType.ApChirp;
                    double start = Math.Clamp(cfg.StartFrequencyHz,5,rate*.40);
                    double stop = Math.Max(start+10,ContinuousSweepAnalyzer.CheckStopFrequency(cfg.StopFrequencyHz,rate));
                    stop = ap ? Math.Min(rate*.49, stop*1.05) : ContinuousSweepAnalyzer.LogChirpEndFrequency(start, stop, rate);
                    double logRate = Math.Log(stop/start)/duration;
                    double hz = start * Math.Exp(logRate * (ap ? -Math.Min(.05,duration*.1) : 0));
                    double step = Math.Exp(logRate/rate);
                    for (int i=0; i<samples.Length; i++, hz*=step) samples[i] *= BaseAt(hz);
                }
            }
            // 信号与直流偏置相加后再检查峰值。即便交流幅度本身低于满量程，带偏置的一侧也可能超出范围。
            for (int i=0; i<samples.Length; i++)
            {
                samples[i] += dc;
                if (!double.IsFinite(samples[i]) || Math.Abs(samples[i]) > 1.00000001)
                    throw new ArgumentException("Sweep level, EQ and DC offset exceed digital full scale.");
            }
            return samples;
        }
        double[]? left = leftEnabled ? GenerateChannel(cfg.LevelDbfs,settings.DcCh1) : null;
        double[]? right = !rightEnabled ? null : left != null && rightLevel == cfg.LevelDbfs && rightDc == settings.DcCh1
            ? left : GenerateChannel(rightLevel,rightDc);
        return new(left ?? new double[right!.Length], right ?? new double[left!.Length], leftEnabled,rightEnabled);
    }

}
