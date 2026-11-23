using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public static class FrequencyGainTable
{
    public static bool ValidTable(BenchTrace? curve)
    {
        // 对数频轴要求频率正且严格递增，每个频率还要对应一个有限的增益；否则二分定位或对数插值失去定义。
        if (curve?.X is not {Length: > 0} x || curve.Y == null || curve.Y.Length != x.Length) return false;
        for (int i=0; i<x.Length; i++)
            if (!double.IsFinite(x[i]) || x[i]<=0 || i>0 && x[i]<=x[i-1] || !double.IsFinite(curve.Y[i])) return false;
        return true;
    }
    public static double GainAtHz(BenchTrace curve,double hz)
    {
        int index = Array.BinarySearch(curve.X,hz);
        // 命中原表就返回原值，边界以外使用最近端点。两点之间按频率比的对数求权重，所以一个倍频程的中点是几何平均频率。
        if (index >= 0) return curve.Y[index];
        int high = ~index;
        if (high == 0) return curve.Y[0];
        if (high == curve.X.Length) return curve.Y[^1];
        int low = high-1;
        // 例如 100→400 Hz 的中点是 200 Hz，分数为 ln(2)/ln(4)=1/2，此处 dB 增益就取两端算术平均。
        double fraction = Math.Log(hz/curve.X[low])/Math.Log(curve.X[high]/curve.X[low]);
        return curve.Y[low]+fraction*(curve.Y[high]-curve.Y[low]);
    }
}
