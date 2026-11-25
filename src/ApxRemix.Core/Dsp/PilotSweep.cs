using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed record PilotLocation(int End, double Frequency, double Level);

public static class PilotSweep
{
    public static double[] Generate(double[] chirp, int rate, PilotSweepOptions options)
    {
        Validate(options, rate);
        int tone = (int)Math.Round(rate * options.DurationMs / 1000.0, MidpointRounding.AwayFromZero);
        int gap = (int)Math.Round(rate * options.SilenceMs / 1000.0, MidpointRounding.AwayFromZero);
        int tail = (int)Math.Ceiling((tone + gap + chirp.Length) * 0.065 + rate * (0.05 + options.AdditionalSilenceMs / 1000.0));
        var result = new double[tone + gap + chirp.Length + tail];

        // APx PilotTone.Create (Om.decompiled.cs:18259-18276):
        // 5 ms 尾部平滑使用 CosineSquaredWindow (0.5 + 0.5*cos(pi*k/N))^2（k=1..N, N = round(fs*0.005) - 1）。
        double peak = Math.Pow(10.0, options.LevelDbfs / 20.0);
        int taper = Math.Max(1, (int)Math.Round(rate * 0.005, MidpointRounding.AwayFromZero) - 1);
        int taperStart = tone - taper;
        double omega = Math.Tau * options.Frequency / rate;
        for (int i = 0; i < tone; i++)
        {
            double sample = peak * Math.Sin(i * omega);
            if (i >= taperStart)
            {
                double c = 0.5 + 0.5 * Math.Cos(Math.PI * (i - taperStart + 1) / taper);
                sample *= c * c;
            }
            result[i] = sample;
        }
        chirp.CopyTo(result, tone + gap);
        return result;
    }

    private static void Validate(PilotSweepOptions p, int rate)
    {
        // 门限、导频长度及频带必须容许检测到足够多的完整周期，否则单次脉冲也可能被错当成标记。
        if (!(p.Frequency >= 200 && p.Frequency <= 0.4 * rate && p.DurationMs >= 100000 / p.Frequency &&
            p.DurationMs <= 5000 && p.SilenceMs >= 10 && p.SilenceMs <= 5000 && p.AdditionalSilenceMs >= 0 &&
            p.AdditionalSilenceMs <= 5000 && p.Iterations is >= 1 and <= 1000 && p.LevelDbfs is >= -120 and <= 0 &&
            p.ThresholdDbfs is >= -140 and <= 0 && p.TimeoutSeconds is > 0 and <= 600))
            throw new ArgumentException("Check pilot frequency, level, duration (at least 100 cycles), silence, iterations and timeout.");
    }

    /// <summary>
    /// 对齐 APx500 导频检测链路（<c>State.decompiled.cs:37611-37639</c> + <c>ChirpParams.MeasureFreq</c>,
    /// <c>Om.decompiled.cs:28711-28808</c>）：先经 3 阶 Butterworth 带通（f1/f0 = 2/3）抑噪，
    /// 定位导频起始并跳过 15 周期滤波暂态后，采用峰值自适应双门限施密特触发与 arcsin 过零插值测量频率与末端样本序号。
    /// </summary>
    public static PilotLocation? Locate(double[] signal, int from, int rate, PilotSweepOptions options)
    {
        Validate(options, rate);
        int startIdx = Math.Max(0, from);
        if (startIdx >= signal.Length) return null;

        double threshold = Math.Pow(10.0, options.ThresholdDbfs / 20.0);
        double fMin = options.Frequency * 0.97;
        double fMax = options.Frequency * 1.03;
        double centerFreq = Math.Sqrt(fMin * fMax);
        int toneNominal = (int)Math.Round(rate * options.DurationMs / 1000.0, MidpointRounding.AwayFromZero);
        int endIdx = (int)Math.Min(signal.Length, startIdx + (long)Math.Ceiling((options.TimeoutSeconds + options.DurationMs / 1000.0) * rate));
        int spanLen = endIdx - startIdx;
        if (spanLen < 64) return null;

        // 1. 去直流并施加 3 阶 Butterworth 带通（f1/f0 = 2/3，与 APx FilterDesigns.CreateButterBandpassFilter(1, 3, f0, 2/3, fs) 一致）
        var filtered = new double[spanLen];
        double mean = 0.0;
        for (int i = 0; i < spanLen; i++) mean += signal[startIdx + i];
        mean /= spanLen;
        for (int i = 0; i < spanLen; i++) filtered[i] = signal[startIdx + i] - mean;

        var bp3 = CreateThirdOrderButterworthBandpass(centerFreq, 2.0 / 3.0, rate);
        bp3.ProcessInPlace(filtered);

        // 2. 在带通输出中搜索首次超过触发门限的导频突发起点（兼容前置静音与多轮迭代录音）
        int burstStart = -1;
        for (int i = 0; i < spanLen; i++)
        {
            if (Math.Abs(filtered[i]) >= threshold)
            {
                burstStart = i;
                break;
            }
        }
        if (burstStart < 0) return null;

        // 3. 跳过前 15 个周期以避开带通滤波器的启动暂态（State.decompiled.cs:37611）
        int skipSamples = (int)Math.Ceiling(15.0 * rate / fMin);
        int measStart = burstStart + skipSamples;
        if (measStart >= spanLen) return null;

        // 限制本次测频窗口至当前导频时长（留出 +10% 容差应对时钟偏移）
        int maxBurstWindow = Math.Min(spanLen - measStart, (int)Math.Ceiling(toneNominal * 1.10) - skipSamples + (int)Math.Ceiling(2.0 * rate / fMin));
        if (maxBurstWindow < 16) return null;

        // 4. 运行 APx ChirpParams.MeasureFreq（arcsin 过零插值 + 尾部丢弃 3 周期避开 5 ms 渐隐）
        double measuredHz = MeasureFreqArcsin(filtered.AsSpan(measStart, maxBurstWindow), fMin, fMax, rate, threshold, out int relLastSample, out double peakLevel, out int countedCycles);
        int minCycles = Math.Max(30, (int)(0.55 * options.DurationMs / 1000.0 * options.Frequency));
        if (!double.IsFinite(measuredHz) || countedCycles < minCycles || peakLevel < threshold ||
            Math.Abs(measuredHz / options.Frequency - 1.0) > 0.0301)
        {
            return null;
        }

        int absEnd = startIdx + measStart + relLastSample;
        return new PilotLocation(absEnd, measuredHz, peakLevel);
    }

    /// <summary>对齐 <c>ChirpParams.MeasureFreq</c>（Om.decompiled.cs:28711-28808）。</summary>
    internal static double MeasureFreqArcsin(
        ReadOnlySpan<double> x, double fMin, double fMax, double fs, double triggerLevel,
        out int lastSampleNum, out double maxPeak, out int cycleCount)
    {
        int timeoutSamples = (int)Math.Ceiling(2.0 * fs / fMin);
        int countdown = timeoutSamples;
        double hystFactor = 0.65 * Math.Sin(Math.PI / 2.0 * (1.0 - 2.0 * fMax / fs));
        int state = 0; // 0 = waiting for negative threshold, 1 = waiting for positive threshold
        double dynThresh = triggerLevel * hystFactor;
        double prevSample = 1.0;
        double zcPrev = 0.0;
        double zcCurr = 0.0;
        int zcIdx = 0;
        maxPeak = 0.0;
        double firstCrossing = 0.0;
        int totalCrossings = 0;
        var lastFourCrossings = new double[4];
        var lastFourSampleNums = new int[4];

        for (int i = 0; i < x.Length; i++)
        {
            double s = x[i];
            double absS = Math.Abs(s);
            if (absS > maxPeak) maxPeak = absS;
            if (maxPeak > triggerLevel) dynThresh = maxPeak * hystFactor;

            if (firstCrossing != 0.0)
            {
                countdown--;
                if (countdown <= 0) break;
            }

            if (s >= 0.0 && prevSample < 0.0)
            {
                zcPrev = prevSample;
                zcCurr = s;
                zcIdx = i;
            }
            prevSample = s;

            if (state == 0)
            {
                if (s < -dynThresh) state = 1;
            }
            else if (s > dynThresh)
            {
                state = 0;
                double crossing = (zcIdx - 1) + ArcsinZeroCrossingOffset(zcPrev, zcCurr, maxPeak);
                if (firstCrossing == 0.0)
                {
                    firstCrossing = crossing;
                }
                else
                {
                    for (int k = 3; k >= 1; k--)
                    {
                        lastFourCrossings[k] = lastFourCrossings[k - 1];
                        lastFourSampleNums[k] = lastFourSampleNums[k - 1];
                    }
                    lastFourCrossings[0] = crossing;
                    lastFourSampleNums[0] = zcIdx - 1;
                    totalCrossings++;
                }
                countdown = timeoutSamples;
            }
        }

        cycleCount = totalCrossings - 4 + 1;
        if (cycleCount <= 0)
        {
            lastSampleNum = 0;
            return double.NaN;
        }
        lastSampleNum = lastFourSampleNums[0];
        double hz = fs * cycleCount / (lastFourCrossings[3] - firstCrossing);
        return double.IsFinite(hz) ? hz : double.NaN;
    }

    private static double ArcsinZeroCrossingOffset(double yPrev, double yCurr, double peak)
    {
        if (peak <= 0.0) return -yPrev / (yCurr - yPrev);
        double a0 = Math.Asin(Math.Clamp(yPrev / peak, -1.0, 1.0));
        double a1 = Math.Asin(Math.Clamp(yCurr / peak, -1.0, 1.0));
        double denom = a0 - a1;
        return denom != 0.0 ? a0 / denom : 0.0;
    }

    private static StreamingIirCascade CreateThirdOrderButterworthBandpass(double centerHz, double f1ToF0Ratio, int sampleRate)
    {
        double f0 = Math.Clamp(centerHz, 5.0, sampleRate * 0.5 * 0.99);
        double w0 = Math.Tan(Math.PI * f0 / sampleRate);
        double w1 = Math.Tan(Math.PI * (f0 * f1ToF0Ratio) / sampleRate);
        double bw = (w0 * w0 / w1) - w1;
        if (bw <= 1e-12) return StreamingIirCascade.Bypass();

        static StreamingIirCascade.BiquadCoeffs FromNormalizedBpS(double bwNum, double c1, double c0)
        {
            double d = 1.0 + c1 + c0;
            return new StreamingIirCascade.BiquadCoeffs(
                B0: bwNum / d,
                B1: 0.0,
                B2: -bwNum / d,
                A1: 2.0 * (c0 - 1.0) / d,
                A2: (1.0 - c1 + c0) / d);
        }

        var s0 = FromNormalizedBpS(bw, bw, w0 * w0);
        Complex bp = new(-0.5 * bw, 0.5 * Math.Sqrt(3.0) * bw);
        Complex disc = Complex.Sqrt(bp * bp - 4.0 * w0 * w0);
        Complex rootA = 0.5 * (bp + disc);
        Complex rootB = 0.5 * (bp - disc);
        var s1 = FromNormalizedBpS(bw, -2.0 * rootA.Real, rootA.Real * rootA.Real + rootA.Imaginary * rootA.Imaginary);
        var s2 = FromNormalizedBpS(bw, -2.0 * rootB.Real, rootB.Real * rootB.Real + rootB.Imaginary * rootB.Imaginary);

        var cascade = new StreamingIirCascade([s0, s1, s2], 1.0);
        double pwrAtF0 = cascade.EvaluatePowerGain(f0, sampleRate);
        double gain = pwrAtF0 > 1e-30 ? 1.0 / Math.Sqrt(pwrAtF0) : 1.0;
        return new StreamingIirCascade([s0, s1, s2], gain);
    }

    // MeasureFreq 返回的 pilot.End 位于 5 ms 渐隐段内（约距导频名义末端 4 ms）。
    // 静音段扣减 3 ms（与末端 4 ms 合计给出 7 ms 前导余量，对应 OpenLoopPilotPanel 的 +7 ms 切片长度）。
    public static int ChirpStart(PilotLocation pilot, int rate, PilotSweepOptions options) =>
        pilot.End + (int)Math.Round(rate * (options.SilenceMs / 1000.0 - 0.003) * options.Frequency / pilot.Frequency);

    public static double[] Segment(double[] signal, int start, int nominalLength, int tail, int rate, double clockRatio, bool correct)
    {
        int needed = (int)Math.Ceiling(nominalLength / clockRatio);
        // 按录放时钟比估计需要截取的点数；启用校正时重采样，使两段记录的时间尺度一致。
        if (start < 0 || needed > signal.Length - start) throw new InvalidDataException("The pilot was detected but the sweep recording is incomplete.");
        int length = Math.Min(signal.Length - start, needed + (int)Math.Ceiling(tail / clockRatio));
        var segment = signal.AsSpan(start, length);
        return correct ? ApxSampleRateConverter.Resample(segment, rate / clockRatio, rate) : segment.ToArray();
    }
}
