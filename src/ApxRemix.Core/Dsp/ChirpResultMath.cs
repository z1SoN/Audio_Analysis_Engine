using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public static class ChirpResultMath
{
    public static (BenchTrace? Channel1, BenchTrace? Channel2) PhaseTraces(ContinuousPhaseCapture measured,
        ContinuousPhaseMode mode, double? fitMinHz = null, double? fitMaxHz = null)
    {
        if (mode == ContinuousPhaseMode.RelativeToCh1)
        {
            var first = measured.Channel1;
            var second = measured.Channel2;
            // 通道相对相位只对同一激励频率有意义。两个横轴一致可逐点相减，不一致时沿有序频率轴前进，只在共有点上做差。
            if (first == null || second == null) return (null,null);
            if (first.X.AsSpan().SequenceEqual(second.X))
            {
                var values=new double[first.X.Length];
                for(int i=0;i<values.Length;i++) values[i]=second.Y[i]-first.Y[i];
                return (null,new(second.X,values));
            }
            // Staggered channels can cross different FFT padding boundaries. Use only the
            // shared measured bins when subtracting their unwrapped phases.
            var commonHz=new List<double>(); var difference=new List<double>();
            for (int a=0,b=0;a<first.X.Length && b<second.X.Length;)
            {
                if (first.X[a]<second.X[b]) a++;
                else if (first.X[a]>second.X[b]) b++;
                else { commonHz.Add(first.X[a]); difference.Add(second.Y[b++]-first.Y[a++]); }
            }
            return commonHz.Count == 0 ? (null,null) : (null,new(commonHz.ToArray(),difference.ToArray()));
        }

        BenchTrace? Display(BenchTrace? trace)
        {
            // 原始展开相位可以保留整圈。需要折叠就归约到单圈，需要扣除线性相位趋势则以有效点拟合斜率和截距。
            if (trace == null || mode == ContinuousPhaseMode.Unwrapped) return trace;
            if (mode == ContinuousPhaseMode.Wrapped)
                return new(trace.X, Array.ConvertAll(trace.Y, FrequencyResponseSweeper.NormalizeAngleDeg));
            // Fit absolute phase against frequency (Hz), independent of the displayed logarithmic axis.
            var fit = Enumerable.Range(0,trace.X.Length).Where(i => double.IsFinite(trace.Y[i]) &&
                trace.X[i] >= (fitMinHz ?? 0) && trace.X[i] <= (fitMaxHz ?? double.PositiveInfinity)).ToArray();
            if (fit.Length < 2) return null;
            double meanX = fit.Average(i => trace.X[i]), meanY = fit.Average(i => trace.Y[i]);
            double variance = fit.Sum(i => Math.Pow(trace.X[i] - meanX, 2));
            if (variance == 0) return null;
            // 在真实 Hz 轴上对相位角作最小二乘拟合，斜率用度/Hz 表示。纯传播延时产生直线，相减后可观察其余频率依赖。
            double slope = fit.Sum(i => (trace.X[i] - meanX) * (trace.Y[i] - meanY)) / variance;
            return new(trace.X, trace.Y.Select((y,i) => y - (meanY + slope * (trace.X[i] - meanX))).ToArray());
        }
        return (Display(measured.Channel1), Display(measured.Channel2));
    }

    public static BenchTrace CrossCorrelation(double[] reference, double[] recording, int rate)
    {
        int length = checked(reference.Length + recording.Length - 1);
        if (reference.Length == 0 || recording.Length == 0 || rate < 1) throw new ArgumentException("Missing sweep data.");
        int n = checked((int)BitOperations.RoundUpToPowerOf2((uint)length));
        var x = new Complex[n];
        var y = new Complex[n];
        // 延迟估计观察两路随平移量的相似程度。频域的 Y·conj(X) 逆变换给出所有平移位置的相关，再用两路能量的几何平均归一化。
        double xx = 0, yy = 0;
        for (int i = 0; i < reference.Length; i++) { x[i] = reference[i]; xx += reference[i] * reference[i]; }
        for (int i = 0; i < recording.Length; i++) { y[i] = recording[i]; yy += recording[i] * recording[i]; }
        FftEngine.ForwardInPlace(x);
        FftEngine.ForwardInPlace(y);
        for (int i = 0; i < n; i++) y[i] *= Complex.Conjugate(x[i]);
        FftEngine.InverseInPlace(y);
        double norm = Math.Sqrt(xx * yy);
        double[] time = new double[length], coefficient = new double[length];
        for (int i = 0; i < length; i++)
        {
            int lag = i - reference.Length + 1;
            time[i] = 1000.0 * lag / rate;
            coefficient[i] = norm > 0 ? Math.Clamp(y[(lag + n) % n].Real / norm, -1, 1) : double.NaN;
        }
        return new(time, coefficient);
    }

    // -d(arg H)/dω = Re{FFT(n*h)/FFT(h)} in samples. Include the physical time
    // origin of the stored IR, even when the graph phase was aligned to its peak.
    public static BenchTrace GroupDelay(double[] impulse, double originMs, int rate, double[] frequencies, double[]? gate = null, Complex[]? responseSpectrum = null)
    {
        int n = responseSpectrum?.Length ?? (int)BitOperations.RoundUpToPowerOf2((uint)Math.Max(4096, impulse.Length * 2));
        var h = responseSpectrum ?? new Complex[n];
        var moment = new Complex[n];
        // 群延迟是 −dφ/dω。傅里叶变换中对频率求导等价于时域样本乘时间，因此同时变换 h[n] 和 n·h[n] 即可计算，最后加回时间轴原点。
        for (int i = 0; i < impulse.Length; i++)
        {
            double sample = impulse[i] * (gate is null ? 1 : gate[i]);
            if (responseSpectrum == null) h[i] = sample;
            moment[i] = sample * i;
        }
        if (responseSpectrum == null) FftEngine.ForwardInPlace(h);
        FftEngine.ForwardInPlace(moment);
        var delay = new double[frequencies.Length];
        for (int i = 0; i < delay.Length; i++)
        {
            // 将 Hz 换到最近频格。分母 H 几乎为零时相位导数没有可靠估计；其余点取复比值实部，由样本换成毫秒。
            int bin = (int)Math.Round(frequencies[i] * n / rate);
            delay[i] = bin is > 0 && bin < n / 2 && h[bin].Magnitude > 1e-8
                ? originMs + 1000.0 / rate * (moment[bin] / h[bin]).Real : double.NaN;
        }
        return new(frequencies, delay);
    }

    // At output frequency k*f, the separated impulse represents Y_k(k*f)/X(k*f).
    // Restore the reference amplitude at that frequency, including a shaped sweep.
    internal static Dictionary<int,BenchTrace> LogProducts(double[] impulse, int peak, int rate, ContinuousSweepConfig cfg,
        IReadOnlyList<FrequencyResponsePoint> points)
    {
        var result = new Dictionary<int,BenchTrace>();
        double low = Math.Clamp(cfg.StartFrequencyHz, 5, rate * .40);
        double high = Math.Max(low + 10, ContinuousSweepAnalyzer.CheckStopFrequency(cfg.StopFrequencyHz, rate));
        double end = cfg.ChirpType == ContinuousChirpType.ApChirp ? Math.Min(rate * .49, high * 1.05) : high;
        // 指数扫频使 m 次谐波在反卷积后相对主峰提前固定样本数，偏移正比于 ln(m)。每一阶截取自己的峰区，并避开相邻阶。
        double samplesPerNeper = Math.Clamp(cfg.DurationSeconds, .1, 15) * rate / Math.Log(end / low);
        var frequencies = points.Select(p => p.FrequencyHz).ToArray();
        var reference = new BenchTrace(frequencies, points.Select(p => p.ExcitationToneDbfs).ToArray());
        for (int order = 2; order <= 20; order++)
        {
            int offset = (int)Math.Round(samplesPerNeper * Math.Log(order));
            int half = Math.Min((int)(.025 * rate), (int)(.44 * samplesPerNeper * Math.Log(1 + 1.0 / order)));
            if (half < 12 || offset + half >= impulse.Length / 2) continue;
            var window = new Complex[(int)BitOperations.RoundUpToPowerOf2((uint)Math.Max(4096, 4 * half))];
            int taper = Math.Max(2, half / 4);
            for (int j = -half; j <= half; j++)
            {
                int edge = half - Math.Abs(j);
                double weight = edge >= taper ? 1 : .5 - .5 * Math.Cos(Math.PI * edge / taper);
                window[j + half] = weight * impulse[(peak - offset + j + impulse.Length) % impulse.Length];
            }
            FftEngine.ForwardInPlace(window);
            var levels = new double[points.Count];
            // 基频 f 的第 m 次谐波在 mf；检查带宽后才报告该频率的量，防止越界分量被折回到图中。
            for (int i = 0; i < levels.Length; i++)
            {
                double target = order * frequencies[i];
                int bin = (int)Math.Round(target * window.Length / rate);
                // 分离出的高阶脉冲给出相对参考的响应；要恢复第 m 次产物的 dBFS，需加上参考在 mf 而非 f 处的电平。
                levels[i] = target <= high * .98 && target < rate * .49 && points[i].IsValid && frequencies[i] * half * 2 / rate >= 1
                    ? 20 * Math.Log10(Math.Max(1e-15, window[bin].Magnitude)) + FrequencyGainTable.GainAtHz(reference, target)
                    : double.NaN;
            }
            if (levels.Any(double.IsFinite)) result[order] = new(frequencies, levels);
        }
        return result;
    }

    // Remove the fundamental before measuring its harmonics on a finite tone slice.
    // A Hann window limits leakage when the selected acquisition ends between cycles.
    internal static void FastProducts(ReadOnlySpan<double> samples, double hz, int rate, int point, Dictionary<int,double[]> levels)
    {
        double w = Math.Tau * hz / rate;
        double cc=0, ss=0, cs=0, yc=0, ys=0;
        double mean=0;
        foreach (double value in samples) mean+=value;
        mean/=samples.Length;
        // 先将基波投影到正弦和余弦上，求解两系数后从样本中扣除基波；有限区间内这两个基底不一定正交，因此保留交叉项。
        for (int i=0;i<samples.Length;i++)
        {
            double c=Math.Cos(w*i), s=Math.Sin(w*i), v=samples[i]-mean;
            cc+=c*c; ss+=s*s; cs+=c*s; yc+=v*c; ys+=v*s;
        }
        double det=cc*ss-cs*cs;
        if(det<=1e-15) return;
        double a=(yc*ss-ys*cs)/det, b=(ys*cc-yc*cs)/det;
        var residual=new double[samples.Length];
        double weight=0;
        for(int i=0;i<residual.Length;i++)
        {
            double win=.5-.5*Math.Cos(Math.Tau*i/(samples.Length-1));
            residual[i]=(samples[i]-mean-a*Math.Cos(w*i)-b*Math.Sin(w*i))*win;
            weight+=win;
        }
        foreach(var (order, values) in levels)
        {
            // 只关心整数倍频率，可在残差上用二阶递推逐个估计目标分量，无须为每个阶次计算整张频谱。
            if (order*hz>=rate*.49) continue;
            double coeff=2*Math.Cos(w*order), v1=0, v2=0;
            foreach (double value in residual) { double v=value+coeff*v1-v2; v2=v1; v1=v; }
            double amplitude=2*Math.Sqrt(Math.Max(0,v1*v1+v2*v2-coeff*v1*v2))/weight;
            values[point]=20*Math.Log10(Math.Max(1e-15,amplitude));
        }
    }
}
