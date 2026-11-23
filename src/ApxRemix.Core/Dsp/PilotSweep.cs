using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed record PilotLocation(int End, double Frequency, double Level);

public static class PilotSweep
{
    public static double[] Generate(double[] chirp, int rate, PilotSweepOptions options)
    {
        Validate(options,rate);
        int tone = (int)Math.Round(rate*options.DurationMs/1000);
        int gap = (int)Math.Round(rate*options.SilenceMs/1000);
        int tail = (int)Math.Ceiling((tone+gap+chirp.Length)*.065 + rate*(.05+options.AdditionalSilenceMs/1000));
        var result = new double[tone+gap+chirp.Length+tail];
        // 用已知幅度和频率的前导音在录音中留下时间标记。靠近末尾逐渐减弱，再按约定间隔接入正式激励。
        double peak = Math.Pow(10,options.LevelDbfs/20);
        int taper = (int)Math.Round(rate*.005);
        for (int i=0; i<tone; i++)
        {
            double envelope = i < tone-taper ? 1 : .5*(1-Math.Cos(Math.PI*(tone-i)/taper));
            result[i] = peak*envelope*Math.Sin(2*Math.PI*options.Frequency*i/rate);
        }
        chirp.CopyTo(result,tone+gap);
        return result;
    }

    private static void Validate(PilotSweepOptions p,int rate)
    {
        // 门限、导频长度及频带必须容许检测到足够多的完整周期，否则单次脉冲也可能被错当成标记。
        if (!(p.Frequency >= 200 && p.Frequency <= .4*rate && p.DurationMs >= 100000/p.Frequency &&
            p.DurationMs <= 5000 && p.SilenceMs >= 10 && p.SilenceMs <= 5000 && p.AdditionalSilenceMs >= 0 &&
            p.AdditionalSilenceMs <= 5000 && p.Iterations is >= 1 and <= 1000 && p.LevelDbfs is >= -120 and <= 0 &&
            p.ThresholdDbfs is >= -140 and <= 0 && p.TimeoutSeconds is > 0 and <= 600))
            throw new ArgumentException("Check pilot frequency, level, duration (at least 100 cycles), silence, iterations and timeout.");
    }

    // Qualify consecutive complete cycles, measuring the frequency across many crossings.
    // A Schmitt threshold prevents noise near zero from restarting the cycle counter.
    public static PilotLocation? Locate(double[] signal,int from,int rate,PilotSweepOptions options)
    {
        Validate(options,rate);
        double threshold = Math.Pow(10,options.ThresholdDbfs/20);
        double period = rate / options.Frequency;
        int minCycles = Math.Max(40,(int)(.72*options.DurationMs/1000*options.Frequency));
        int end = (int)Math.Min(signal.Length,from + (options.TimeoutSeconds+options.DurationMs/1000)*rate);
        double first=0, previous=0, peak=0, cyclePeak=0;
        int cycles=0;
        bool armed=false;
        PilotLocation? candidate=null;
        for (int i=Math.Max(1,from); i<end; i++)
        {
            double y=signal[i];
            // 最近一次合格候选之后若已超过两个预期周期未见交越，返回该候选作为导频末端。
            if (candidate!=null && i-previous>period*2) return candidate;
            cyclePeak = Math.Max(cyclePeak,Math.Abs(y));
            // 负向越过门限才武装下一次上升过零，避免在零附近的噪声摆动被重复计为周期。
            if (y < -threshold*.65) armed=true;
            if (!armed || y<=0 || signal[i-1]>0) continue;
            // 用零点两边的样本线性插值定位交越时刻，随后用多个周期间隔估频，避免单个采样点的取整误差主导结果。
            double crossing=i-y/(y-signal[i-1]);
            armed=false;
            double interval=crossing-previous;
            if (interval<.81*period || interval>1.20*period || cyclePeak<threshold*.9)
            {
                if (candidate!=null) return candidate;
                cycles=0; first=crossing; peak=0;
            }
            else
            {
                cycles++;
                peak=Math.Max(peak,cyclePeak);
                double hz=cycles*rate/(crossing-first);
                // 连续周期数、频率误差和幅度同时通过才接受候选；检测到导频结束后才能用约定间隔定位主扫频。
                if (cycles>=minCycles && Math.Abs(hz/options.Frequency-1)<=.0301 && peak>=threshold)
                    candidate=new((int)Math.Round(crossing+rate/hz),hz,peak);
            }
            previous=crossing; cyclePeak=0;
        }
        return candidate;
    }

    // 从检测的导频末端推算扫频起点，名义静音时间先扣 7 ms，再按标称与实测导频之比折算录音样本数。
    public static int ChirpStart(PilotLocation pilot, int rate, PilotSweepOptions options) =>
        pilot.End + (int)Math.Round(rate*(options.SilenceMs/1000-.007)*options.Frequency/pilot.Frequency);

    public static double[] Segment(double[] signal,int start,int nominalLength,int tail,int rate,double clockRatio,bool correct)
    {
        int needed=(int)Math.Ceiling(nominalLength/clockRatio);
        // 按录放时钟比估计需要截取的点数；启用校正时重采样，使两段记录的时间尺度一致。
        if (start<0 || needed>signal.Length-start) throw new InvalidDataException("The pilot was detected but the sweep recording is incomplete.");
        int length=Math.Min(signal.Length-start,needed+(int)Math.Ceiling(tail/clockRatio));
        var segment=signal.AsSpan(start,length);
        return correct ? SignalGeneratorDsp.ResampleCubicHermite(segment,Math.Max(1,(int)Math.Round(rate/clockRatio)),rate) : segment.ToArray();
    }
}
