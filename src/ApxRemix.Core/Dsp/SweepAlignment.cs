// 录音第零点通常与发出扫频的第零点不同。这里把两条波形换到频域求相关，给切窗步骤一个以录音采样点为单位的延迟，避免把前一个频点切进来。
using System.Numerics;

namespace ApxRemix.Core.Dsp;

public static class SweepAlignment
{
    /// <summary>Signed recording delay from the absolute peak of linear cross-correlation.</summary>
    public static int FindDelay(ReadOnlySpan<double> reference, ReadOnlySpan<double> recording)
    {
        // 想象把参考波形逐点平移，每个位置都乘上录音并求和。FFT 能一次算出这些相关值。
        // 先补零到至少“两个长度之和减一”，才能把录音末尾和开头分开，得到线性相关。
        long length = (long)reference.Length + recording.Length - 1;
        if (reference.IsEmpty || recording.IsEmpty || length > (1 << 29))
            throw new ArgumentException("Alignment requires two nonempty waveforms that fit in an FFT.");
        int n = (int)BitOperations.RoundUpToPowerOf2((uint)length);
        var stimulus = new Complex[n];
        var capture = new Complex[n];
        for (int i = 0; i < reference.Length; i++) stimulus[i] = reference[i];
        for (int i = 0; i < recording.Length; i++) capture[i] = recording[i];
        FftEngine.ForwardInPlace(stimulus);
        FftEngine.ForwardInPlace(capture);
        // 把录音谱乘以参考谱的共轭，逆变换得各个延时下的相似度。取绝对值可识别极性反转但时间相同的信号。
        for (int i = 0; i < n; i++) capture[i] *= Complex.Conjugate(stimulus[i]);
        FftEngine.InverseInPlace(capture);
        double peak = 0;
        int best = 0;
        // 只搜索两段记录实际能重叠的延时范围，负延时存于循环 FFT 缓冲的末端。
        for (int lag = recording.Length - 1; lag > -reference.Length; lag--)
        {
            // 若录音反相，正确位置会出现深的负峰；取绝对值也能找到它。正 lag 表示录音晚到。
            // 这次搜索给整段录音一个偏移量。若两只采样时钟越走越偏，末尾的时间误差仍会留下来。
            double value = Math.Abs(capture[lag < 0 ? lag + n : lag].Real);
            if (value > peak) { peak = value; best = lag; }
        }
        if (peak == 0) throw new ArgumentException("Silent signals have no identifiable delay.");
        return best;
    }
}
