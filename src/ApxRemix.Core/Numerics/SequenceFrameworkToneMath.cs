using System;
using System.Numerics;

internal static class SequenceFrameworkToneMath
{
    // Keep the complete phase expression in the same x86 floating-point context.
    // atan2 保留复数方向的象限；加 π/2 改成正弦参考，再减去落在分数格造成的相移，结果以弧度计。
    internal static double BinPhase(double imaginary, double real, double fractionalBin)
    {
        double angle = Math.Atan2(imaginary, real);
        return angle + Math.PI / 2.0 - Math.PI * fractionalBin;
    }

    // RMS 乘 √2 恢复正弦峰值，初相每点推进 2π·cycles/N，重建同相波形并从样本中扣除。
    internal static void SubtractSine(double[] samples, double rms, double phase, double cycles)
    {
        double amplitude = rms * Math.Sqrt(2.0);
        double step = cycles * 2.0 * Math.PI / samples.Length;
        for (int i = 0; i < samples.Length; i++, phase += step) samples[i] -= Math.Sin(phase) * amplitude;
    }

    // 残差谱输入为每格功率。基频不为正时无法按频率比定位陷波，此时直接返回。
    internal static void ApplyResidualNotch(double[] power, double fundamentalBin)
    {
        if (fundamentalBin <= 0) return;
        // 陷波只作用在基频一半至两倍之间；中心取最近整数格，中心及左右各三格取零增益。
        int first = Math.Max(0, (int)Math.Ceiling(fundamentalBin * 0.5));
        int last = Math.Min(power.Length - 1, (int)Math.Floor(fundamentalBin * 2));
        int center = (int)Math.Round(fundamentalBin);
        const int halfWidth = 3;
        for (int k = first; k <= last; k++)
        {
            double gain = 0;
            if (k < center - halfWidth || k > center + halfWidth)
            {
                // 边缘格用 k/fundamentalBin 作无量纲频率，求传递函数分子与三个二阶分母的幅值比。
                double ratio = k / fundamentalBin;
                var s = new Complex(0, ratio * (Math.PI * 2));
                double s2 = -ratio * ratio * (Math.PI * 2) * (Math.PI * 2);
                gain = Math.Pow(Math.Abs(s2 + 39.47860224), 3);
                gain /= (s2 + s * -1.4414 + 19.5226).Magnitude;
                gain /= (s2 + s * -2.9152 + 79.8613).Magnitude;
                gain /= (s2 + s * -5.65 + 39.4855).Magnitude;
            }
            // 从幅值增益到功率增益需平方，窗口外各格没有进入循环，保持其原有功率。
            power[k] *= gain * gain;
        }
    }
}
