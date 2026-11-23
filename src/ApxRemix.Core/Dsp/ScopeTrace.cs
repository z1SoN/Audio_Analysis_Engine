namespace ApxRemix.Core.Dsp;

public static class ScopeTrace
{
    public const int Radius = 32;
    private const int Phases = 512;
    private static readonly double[][] Weights = CreateWeights();

    // 为示波显示预计算分数采样位置的插值核。带限波形可用 sinc 重建，用有限长度的窗把计算限制在附近样本。
    private static double[][] CreateWeights()
    {
        var table = new double[Phases + 1][];
        for (int p = 0; p <= Phases; p++)
        {
            var row = table[p] = new double[2 * Radius];
            for (int k = 0; k < row.Length; k++)
            {
                // x 是查询位置距该采样点的间隔数。x=0 时 sinc 的极限为 1，直接赋值避免 0/0。
                double x = (double)p / Phases + Radius - 1 - k;
                double a = Math.PI * x;
                double window = .42 + .5 * Math.Cos(a / Radius) + .08 * Math.Cos(2 * a / Radius);
                row[k] = x == 0 ? 1 : window * Math.Sin(a) / a;
            }
        }
        return table;
    }

    // 在以样本数表示的位置上估计波形幅度。整数索引直接取原样本，范围外用 NaN 表明没有数据。
    public static double Interpolate(double[] data, double position)
    {
        if (!double.IsFinite(position) || position < 0 || position > data.Length - 1) return double.NaN;
        int center = (int)Math.Floor(position);
        double phase = (position - center) * Phases;
        int p = Math.Min((int)phase, Phases - 1);
        if (phase == 0) return data[center];
        int start = center - Radius + 1;
        double blend = phase - p, sum = 0, weights = 0;
        // 只使用记录内的有效邻点，在相邻两个预计算相位间平滑过渡，再除以实际权重之和，使常量输入保持常量。
        for (int i = Math.Max(0, start); i < Math.Min(data.Length, start + 2 * Radius); i++)
        {
            int k = i - start;
            double w = Weights[p][k] + blend * (Weights[p + 1][k] - Weights[p][k]);
            sum += w * data[i];
            weights += w;
        }
        return sum / weights;
    }

    // Both traces use this same position on the capture clock, including when Ch2 triggers.
    // 在可显示完整时间跨度的位置中倒序找最近的上升交越。指定电平优先，否则以整段均值为触发电平。
    public static (double Start, bool Locked) FindStart(double[] reference, double span, bool interpolate, double level = double.NaN)
    {
        if (!(span > 0) || span >= reference.Length - 1) return (0,false);
        int spare = reference.Length - 1 - (int)Math.Ceiling(span);
        if (spare <= 0) return (0, false);
        int margin = Math.Min(Radius, (spare - 1) / 2);
        int last = spare - margin;
        double mean = double.IsFinite(level) ? level : reference.Average();
        for (int i = last; i > margin; i--)
        {
            double a = reference[i - 1], b = reference[i];
            if (!(a < mean && b >= mean)) continue;
            // 先用相邻样本定位交越区间。快速路径按两点直线求交点，插值路径对重建的波形二分细化位置。
            if (!interpolate) return (i - 1 + (mean - a) / (b - a), true);
            double lo = i - 1, hi = i;
            for (int n = 0; n < 15; n++)
            {
                double mid = (lo + hi) / 2;
                if (Interpolate(reference, mid) < mean) lo = mid; else hi = mid;
            }
            return ((lo + hi) / 2, true);
        }
        // 无合适交越时仍提供可显示完整波形的起点，Locked 告知调用方此次未锁定。
        return (last, false);
    }
}
