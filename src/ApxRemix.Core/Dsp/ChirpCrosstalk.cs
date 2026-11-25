using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public static class ChirpCrosstalk
{
    /// <summary>
    /// 对齐 APx500 <c>ChirpParams.OPPE837xSG</c>：
    /// 在居中冲激响应中围绕错开通道的串扰峰截取 ±1 ms（<c>2·⌊Fs/1000⌋</c> 样本），
    /// 拟合 5 阶多项式扣除平滑基底后分离带内串扰脉冲，并将基底与带外高频残余写回原冲激响应，
    /// 使直通通道保持完整线性窗长而不被串扰峰截短。
    /// </summary>
    internal static double[]? ExciseStaggeredImpulse(double[] centeredIr, int directPeak, int offsetSamples, int sampleRate, double stopHz)
    {
        if (offsetSamples == 0 || sampleRate <= 0) return null;
        int halfWidth = Math.Max(8, (int)Math.Floor(sampleRate / 1000.0));
        int len = halfWidth * 2;
        int start = directPeak + offsetSamples - halfWidth;
        if (start < 0 || start + len > centeredIr.Length) return null;

        var slice = new double[len];
        Array.Copy(centeredIr, start, slice, 0, len);
        var poly = ChirpResultMath.FitPolynomial5(slice);
        var detrended = new double[len];
        for (int i = 0; i < len; i++) detrended[i] = slice[i] - poly[i];

        // 带外高频保留并叠加回多项式基底，仅切除扫频通带内的串扰脉冲能量
        var spec = new Complex[len];
        for (int i = 0; i < len; i++) spec[i] = new Complex(detrended[i], 0.0);
        SequenceTransform.Forward(spec);

        int f2Bin = (int)Math.Ceiling(stopHz / sampleRate * len);
        int zeroBins = Math.Clamp((int)Math.Min(Math.Floor(0.8 * f2Bin), f2Bin - 5), 0, len / 2 + 1);
        for (int k = 0; k < zeroBins; k++)
        {
            spec[k] = Complex.Zero;
            if (k > 0 && k < len - k) spec[len - k] = Complex.Zero;
        }
        SequenceTransform.Inverse(spec);

        for (int i = 0; i < len; i++)
            centeredIr[start + i] = spec[i].Real + poly[i];

        return detrended;
    }

    /// <summary>
    /// 对齐 APx500 <c>ChirpParams.GetXtalkSum</c> 与 <c>ReprocessImpulseResults.ReprocessXtalk</c>：
    /// 对切除出的串扰冲激切片施加 TrackPeak Equiripple 非对称窗并变换至 <c>harmFftLen</c>，
    /// 与直通线性响应幅值逐频求比（自 max(1000 Hz, startHz) 至 stopHz）。
    /// </summary>
    internal static BenchTrace? FromExcisedImpulse(
        double[] xtalkSlice,
        Complex[] linearSpec,
        int linFftLen,
        int harmFftLen,
        int sampleRate,
        double startHz,
        double stopHz)
    {
        if (xtalkSlice.Length < 4 || harmFftLen < 64) return null;
        int peakInSlice = xtalkSlice.Length / 2;
        int sliceStart = xtalkSlice.Length > harmFftLen
            ? Math.Clamp(peakInSlice - harmFftLen / 2, 0, xtalkSlice.Length - harmFftLen)
            : 0;
        int used = Math.Min(xtalkSlice.Length, harmFftLen);
        var asymWin = SequenceFrequencyResponse.CreateImpulseWindow(used, peakInSlice - sliceStart, 0.90, 0.95);

        var buf = new Complex[harmFftLen];
        for (int i = 0; i < used; i++)
            buf[i] = new Complex(xtalkSlice[sliceStart + i] * asymWin[i], 0.0);
        SequenceTransform.Forward(buf);

        double minHz = Math.Max(1000.0, startHz);
        int halfBins = harmFftLen / 2;
        int firstBin = Math.Clamp((int)Math.Floor(minHz / sampleRate * harmFftLen), 1, halfBins);
        int lastBin = Math.Clamp((int)Math.Ceiling(stopHz / sampleRate * harmFftLen), firstBin, halfBins);
        int count = lastBin - firstBin + 1;
        if (count <= 0) return null;

        double harmDf = (double)sampleRate / harmFftLen;
        double linDf = (double)sampleRate / linFftLen;
        int linMaxBin = linFftLen / 2 - 1;
        var freqs = new double[count];
        var ratioDb = new double[count];

        for (int i = 0; i < count; i++)
        {
            int k = firstBin + i;
            double hz = k * harmDf;
            freqs[i] = hz;
            double xtalkMag = buf[k].Magnitude;
            double pos = Math.Clamp(hz / linDf, 1.0, linMaxBin);
            int lo = (int)pos;
            double fundMag = linearSpec[lo].Magnitude;
            if (pos != lo && lo + 1 <= linMaxBin)
                fundMag += (pos - lo) * (linearSpec[lo + 1].Magnitude - fundMag);
            ratioDb[i] = double.IsFinite(fundMag) && fundMag > 1e-12
                ? 20.0 * Math.Log10(Math.Max(xtalkMag / fundMag, 1e-15))
                : double.NaN;
        }
        return new BenchTrace(freqs, ratioDb);
    }

    public static BenchTrace FromStaggeredImpulse(double[] impulse, int directPeak, int offset, double[] frequencies, int sampleRate)
    {
        int radius = Math.Abs(offset) / 2;
        // 每窗半径取两峰样本间距绝对值的一半向下取整，偶数间距时相邻窗恰在零权重端点相遇。响应拖尾太长仍可串入另一窗。
        if (radius < 2 || 2 * radius >= impulse.Length) throw new ArgumentOutOfRangeException(nameof(offset));
        double[] Gated(int center)
        {
            var output = new double[2 * radius + 1];
            for (int i = -radius; i <= radius; i++)
            {
                // 在每个峰周围使用相同的渐变窗，减轻截断边沿带来的频谱扩散，再比较两段的频谱。
                double edge = Math.Max(0, 2.0 * Math.Abs(i) / radius - 1);
                double taper = 0.5 + 0.5 * Math.Cos(Math.PI * edge);
                output[i + radius] = impulse[(center + i + 2 * impulse.Length) % impulse.Length] * taper;
            }
            return output;
        }
        return Ratio(Gated(directPeak), Gated(directPeak + offset), frequencies, sampleRate);
    }

    // Compare the same input with its own output excited and with only the other output excited.
    // Band power avoids phase cancellation when the two sweeps have different capture latencies.
    public static BenchTrace Ratio(double[] driven, double[] leakage, double[] frequencies, int sampleRate)
    {
        int length = (int)BitOperations.RoundUpToPowerOf2((uint)Math.Max(driven.Length, leakage.Length));
        if (length < 2 || sampleRate <= 0) throw new ArgumentException("Two complete sweep captures are required.");
        double[] Power(double[] samples)
        {
            var fft = new Complex[length];
            // 用相同长度 FFT 求两路各频点功率，保证后面相同下标代表相同频率。
            for (int i = 0; i < samples.Length; i++) fft[i] = samples[i];
            FftEngine.ForwardInPlace(fft);
            var power = new double[length / 2 + 1];
            for (int i = 0; i < power.Length; i++) power[i] = fft[i].Magnitude * fft[i].Magnitude;
            return power;
        }
        var direct = Power(driven);
        var coupled = Power(leakage);
        double RatioAt(double frequency)
        {
            // 在目标频率附近分别累加泄漏和直通功率。功率比用 10log₁₀，直通功率为零时比值无定义。
            double center = frequency * length / sampleRate;
            // 两条记录各自在目标频率上下各 1/48 倍频程内积分；比较功率可避免跨记录相位差直接抵消幅值。
            int low = Math.Clamp((int)Math.Round(center * Math.Pow(2, -1.0 / 48)), 1, direct.Length - 1);
            int high = Math.Clamp((int)Math.Round(center * Math.Pow(2, 1.0 / 48)), low, direct.Length - 1);
            double reference = 0, leak = 0;
            for (int i = low; i <= high; i++) { reference += direct[i]; leak += coupled[i]; }
            return reference > 0 ? 10 * Math.Log10(leak / reference) : double.NaN;
        }
        return new(frequencies, Array.ConvertAll(frequencies, RatioAt));
    }
}
