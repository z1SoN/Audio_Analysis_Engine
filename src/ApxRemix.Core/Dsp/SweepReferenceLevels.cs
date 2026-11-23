using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

internal static class SweepReferenceLevels
{
    // Measure each cycle of the actual reference. A DC offset cancels between its extrema.
    // Three sampled points around each extremum recover its amplitude even near Nyquist.
    public static BenchTrace? Measure(ReadOnlySpan<double> samples,int rate,double startHz,double stopHz)
    {
        if (samples.Length < 4 || startHz <= 0 || stopHz <= startHz) return null;
        int low=0, high=samples.Length-1;
        // 去掉静音首尾，只从参考确实有波形的区间估计发生电平。
        while (low < high && samples[low] == 0) low++;
        while (high > low && samples[high] == 0) high--;
        int guard = Math.Min(rate/200,(high-low)/20);
        low = Math.Max(1,low+guard);
        high = Math.Min(samples.Length-2,high-guard);
        var bands = new SortedDictionary<int,List<double>>();
        int previous=-1, trough=-1;
        double previousTime=0;
        for (int i=low;i<=high;i++)
        {
            if (samples[i] <= samples[i-1] && samples[i] < samples[i+1]) trough=i;
            if (samples[i] < samples[i-1] || samples[i] <= samples[i+1]) continue;
            // 峰顶可能落在两采样点之间。用峰附近三点作抛物线近似求峰时刻，相邻峰的时间差给出当时频率。
            double curvature = samples[i-1] - 2*samples[i] + samples[i+1];
            double peakTime = i + (samples[i-1]-samples[i+1])/(2*curvature);
            if (previous >= 0 && trough > previous)
            {
                double hz = rate / (peakTime-previousTime);
                if (hz >= startHz && hz <= stopHz && hz < rate*.49)
                {
                    // 以峰谷之差的一半求交流幅度，可以消除固定直流偏置。把读数按频率归组，后面可抵抗个别峰位估计偏差。
                    double w = 2*Math.PI*hz/rate;
                    double peak = Extremum(samples,i,w);
                    double bottom = Extremum(samples,trough,w);
                    double amplitude = (peak-bottom)*.5;
                    if (amplitude>1e-9 && double.IsFinite(amplitude))
                    {
                        // 各周期归到以起始频率为基准的 1/12 倍频程频带。每带取电平中位数，个别峰位估计偏差不易拉偏整个频带。
                        int band = (int)Math.Round(12*Math.Log2(hz/startHz));
                        if (!bands.TryGetValue(band,out var values)) bands.Add(band,values=[]);
                        values.Add(20*Math.Log10(amplitude));
                    }
                }
            }
            previous=i;
            previousTime=peakTime;
        }
        if (bands.Count<3) return null;
        var hzAxis = new double[bands.Count];
        var db = new double[bands.Count];
        int j=0;
        foreach (var (band,values) in bands)
        {
            values.Sort();
            hzAxis[j] = startHz*Math.Pow(2,band/12.0);
            db[j++] = values[values.Count/2];
        }
        // Full cycles supply the sine level without the partial-cycle bias of an RMS
        // over the whole sweep. Exclude the tapered edge bands of a uniform source.
        var inner = db.Skip(1).SkipLast(1).Order().ToArray();
        // 若内部频带的电平变化足够小，一个代表值就能描述激励，否则保留逐频率的参考曲线。
        if (inner[(inner.Length-1)*9/10]-inner[(inner.Length-1)/10] < .35)
            return new([hzAxis[0]], [inner[inner.Length / 2]]);
        return new(hzAxis,db);
    }

    private static double Extremum(ReadOnlySpan<double> samples,int i,double w)
    {
        // 局部近似单频正弦时，可由中央及左右样本反推正交分量，再求连续波形的极值。直接拿离散最高点通常会低估峰值。
        double center = samples[i];
        double cosine = (center - .5*(samples[i-1]+samples[i+1]))/(1-Math.Cos(w));
        double sine = (samples[i+1]-samples[i-1])/(2*Math.Sin(w));
        return center-cosine+Math.CopySign(Math.Sqrt(cosine*cosine+sine*sine),cosine);
    }
}
