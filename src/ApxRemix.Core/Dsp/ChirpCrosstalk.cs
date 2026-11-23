using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public static class ChirpCrosstalk
{
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
