// FastSweep 先确定每个恒频切片。这个分析器接手一个通道的单个切片，分离较高频成分，再比较峰值和 RMS；峰值统计能显出只占少数采样点的异常脉冲。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed record RubAndBuzzSweep(BenchTrace CrestFactor, BenchTrace PeakRatio, BenchTrace? Loudness = null);

public sealed record RubAndBuzzReading(double[] Filtered, int SkippedSamples, double CrestFactor, double PeakRatio);

/// <summary>Evaluates one recorded constant-frequency Fast Sweep slice, one channel at a time.</summary>
public static class RubAndBuzzAnalyzer
{
    public static RubAndBuzzReading Analyze(ReadOnlySpan<double> samples, int sampleRate,
        double frequencyHz, double highPassFactor, double discardCycles)
    {
        // 擦碰产生的短促高频残差可能被很强的基波掩盖。先按基频的倍数定位高通，再跳过规定周期数，让分析对准稳态。
        if (sampleRate < 12) throw new ArgumentOutOfRangeException(nameof(sampleRate));
        if (!double.IsFinite(frequencyHz) || frequencyHz <= 0 || frequencyHz >= sampleRate * 0.5)
            throw new ArgumentOutOfRangeException(nameof(frequencyHz));
        if (!double.IsFinite(highPassFactor) || highPassFactor <= 0)
            throw new ArgumentOutOfRangeException(nameof(highPassFactor));
        if (!double.IsFinite(discardCycles) || discardCycles < 0)
            throw new ArgumentOutOfRangeException(nameof(discardCycles));
        // 恒频段里的每周期占 fs/f 个采样点。48 kHz 下测 2 kHz，每周期 24 点，再丢 2 个周期就是 48 点。
        // 这次丢弃从所选切片的开头数，和发生器在切片之前预留的丢弃时长可以分别设置。
        var skip = Math.Round(discardCycles * sampleRate / frequencyHz, MidpointRounding.AwayFromZero);
        if (skip >= samples.Length)
            throw new ArgumentException("The slice must contain samples after the discard interval.", nameof(samples));

        var cutoffHz = Math.Clamp(frequencyHz * highPassFactor, 5.0, sampleRate * 0.495);
        var filter = StreamingIirCascade.CreateButterworthHighPass(6, cutoffHz, sampleRate);
        var residual = samples.ToArray();
        // 新建高通从零状态起步，让包括丢弃段在内的所有样本先走过滤波器，再从 first 开始统计。
        // 如果先剪掉这段样本才启动滤波器，启动瞬态会再次落进统计区。
        filter.ProcessInPlace(residual);

        int first=(int)skip;
        var (crest,ratio)=MeasureRatios(samples[first..],residual.AsSpan(first));
        return new(residual,first,crest,ratio);
    }

    private static (double Crest,double Ratio) MeasureRatios(ReadOnlySpan<double> input, ReadOnlySpan<double> residual)
    {
        // 同一个残差峰值分别除以残差 RMS 和输入 RMS。前者回答残差是否尖锐，后者回答它相对整个测试信号有多大。
        double peak=0, residualEnergy=0, inputEnergy=0;
        for(int i=0;i<input.Length;i++)
        {
            peak=Math.Max(peak,Math.Abs(residual[i]));
            residualEnergy+=residual[i]*residual[i];
            inputEnergy+=input[i]*input[i];
        }
        return (residualEnergy>0 ? peak/Math.Sqrt(residualEnergy/input.Length) : double.NaN,
            inputEnergy>0 ? peak/Math.Sqrt(inputEnergy/input.Length) : double.NaN);
    }

    // The chirp high-pass follows the sample clock continuously. Keep the six filter
    // poles' histories across coefficient updates; restarting each band creates false peaks.
    public static RubAndBuzzSweep? AnalyzeLogChirp(ContinuousSweepResult result,ContinuousSweepConfig cfg,double factor)
    {
        // 仅在结果含原始响应和参考波形、且扫频采用对数变频时能执行跟踪高通。缺少任一输入就没有可重算的残差曲线。
        if(cfg.ChirpType!=ContinuousChirpType.LogChirp || result.Acquired is not {} audio || result.Reference is not {} reference) return null;
        return AnalyzeLogChirp(audio,reference,result.SampleRate,result.EstimatedDelaySamples,
            result.ResponsePoints.Select(p=>p.FrequencyHz),cfg,factor);
    }

    public static RubAndBuzzSweep? AnalyzeLogChirp(double[] audio,double[] reference,int rate,int delay,
        IEnumerable<double> frequencies,ContinuousSweepConfig cfg,double factor)
    {
        if(cfg.ChirpType!=ContinuousChirpType.LogChirp) return null;
        if(!double.IsFinite(factor) || factor is <5 or >30) throw new ArgumentOutOfRangeException(nameof(factor));
        double duration=Math.Clamp(cfg.DurationSeconds,.1,15);
        int expected=Math.Max(2048,(int)Math.Round(rate*duration));
        int origin=delay;
        // 把扫频在录音里的开始时刻定位后，用指数关系将时间映射成频率；同一段滑动高通才会一直跟随基波。
        if(reference.Length!=expected) origin+=SweepAlignment.FindDelay(ContinuousSweepAnalyzer.GenerateLogChirp(cfg,rate),reference);
        double start=Math.Clamp(cfg.StartFrequencyHz,5,rate*.4);
        double stop=Math.Clamp(cfg.StopFrequencyHz,start+10,rate*.47);
        double end=ContinuousSweepAnalyzer.LogChirpEndFrequency(start,stop,rate);
        double slope=Math.Log(end/start)/(rate*duration);
        int first=Math.Max(0,-origin), last=Math.Min(expected,audio.Length-origin);
        // 高通截止会随基波上升。超过可实现的截止上限后缩短分析范围，避免把后半段当作具有相同选择性的测量。
        last=Math.Min(last,(int)Math.Floor(Math.Log(rate*.475/(factor*start))/slope));
        if(last-first<32) return null;
        var input=audio.AsSpan(origin+first,last-first);
        double[] residual=input.ToArray();
        int block=Math.Clamp((int)(Math.Log(2)/(96*slope)),1,64);
        StreamingIirCascade? prior=null;
        for(int i=0;i<residual.Length;i+=block)
        {
            int count=Math.Min(block,residual.Length-i);
            // 每段采用中心时刻对应的截止频率，接续上一段滤波器的历史，避免把每次换系数产生的启动峰当成机械擦碰。
            double cutoff=start*Math.Exp((first+i+count*.5)*slope)*factor;
            var filter=StreamingIirCascade.CreateButterworthHighPass(6,cutoff,rate);
            if(prior!=null) filter.CopyHistoryFrom(prior);
            filter.ProcessInPlace(residual.AsSpan(i,count));
            prior=filter;
        }
        var hz=new List<double>();
        var crest=new List<double>();
        var ratio=new List<double>();
        var loudnessPhons=new List<double>();
        foreach(double f in frequencies)
        {
            // 将请求的测量频率反算为扫频中的时刻，仅用两侧均有足够样本的窗口计算峰值、RMS 与感知响度。
            if(!double.IsFinite(f) || f<=0) continue;
            int center=(int)Math.Round(Math.Log(f/start)/slope)-first;
            // Use at least one cycle around each frequency, with a sixth-octave
            // frequency interval where the sweep moves slowly.
            // 半窗至少容纳半个周期，另一项对应扫过十二分之一倍频程的点数；左右两侧合起来约覆盖六分之一倍频程。
            int half=Math.Max((int)Math.Ceiling(rate/(2*f)),(int)Math.Round(Math.Log(2)/(12*slope)));
            if(center-half<0 || center+half>=input.Length) continue;
            var resSlice = residual.AsSpan(center-half,2*half+1);
            var values=MeasureRatios(input.Slice(center-half,2*half+1),resSlice);
            hz.Add(f); crest.Add(values.Crest); ratio.Add(values.Ratio);
            loudnessPhons.Add(ComputeSliceLoudnessPhons(resSlice, rate));
        }
        if(hz.Count==0) return null;
        var x=hz.ToArray();
        return new(new(x,crest.ToArray()),new(x,ratio.ToArray()),new(x,loudnessPhons.ToArray()));
    }

    private static double ComputeSliceLoudnessPhons(ReadOnlySpan<double> residualSlice, int sampleRate)
    {
        int n = 1;
        while (n < residualSlice.Length && n < 2048) n <<= 1;
        if (n < 64) n = 64;
        var padded = new double[n];
        residualSlice[..Math.Min(residualSlice.Length, n)].CopyTo(padded);
        var (linPeak, _, _, _) = FftEngine.ComputeSingleSidedSpectrum(padded, n, FftWindowType.Hann);
        int bins = linPeak.Length;
        var freqs = new double[bins];
        double df = sampleRate / (2.0 * bins);
        for (int i = 0; i < bins; i++) freqs[i] = i * df;
        var ear = ApxRubAndBuzzLoudness.ApplyEarWeighting(linPeak, sampleRate);
        var bark = ApxRubAndBuzzLoudness.ApplyAuditoryFilterBands(freqs, ear);
        if (bark.Length == 0) return 0.0;
        var spread = ApxRubAndBuzzLoudness.ApplyFrequencySpreading(bark);
        ApxRubAndBuzzLoudness.CalculateLoudness(spread, out _, out double sones);
        if (!(sones > 0.0) || !double.IsFinite(sones)) return 0.0;
        // ISO 532B / Zwicker sone-to-phon conversion (1 sone = 40 phon, +10 phon per doubling).
        return sones >= 1.0 ? 40.0 + 10.0 * Math.Log2(sones) : 40.0 * Math.Pow(sones + 0.0005, 0.35);
    }
}
