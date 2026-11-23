// 要用一段有限录音估计失真，基波可能落在两个 FFT 格之间。先在时域拟合基波并生成逐点残差，AudioMetricsAnalyzer 就可以继续统计残差中的谐波与噪声。
namespace ApxRemix.Core.Dsp;

/// <summary>Windowed least-squares sine removal for distortion measurements.</summary>
internal static class SineFit
{
    internal sealed record Result(double FrequencyBins, double Cosine, double Sine, double Dc)
    {
        // 用拟合得到的余弦、正弦和直流重建同长度波形，再逐点相减。残余保留未被这个单音模型解释的成分。
        public double[] Residual(ReadOnlySpan<double> samples)
        {
            var residual = new double[samples.Length];
            double step = 2 * Math.PI * FrequencyBins / samples.Length;
            for (int i = 0; i < samples.Length; i++)
            {
                // 相位零点取记录中点，须与求解时的时间基准相同。FrequencyBins 表示这段记录内经过的周期数。
                double angle = step * (i - samples.Length / 2.0);
                residual[i] = samples[i] - Cosine * Math.Cos(angle) - Sine * Math.Sin(angle) - Dc;
            }
            return residual;
        }
    }

    // 在粗估频率附近寻找使加窗残差平方和最小的单音与直流。输入权重应与样本一一对应。
    public static Result Fit(ReadOnlySpan<double> samples, double initialBins, double[] weights, bool fixedFrequency)
    {
        double bins = initialBins, a = 0, b = 0, dc = 0;
        Span<double> basis = stackalloc double[4];
        for (int iteration = 0; iteration < 7; iteration++)
        {
            // 第一轮先求三个线性参数。有非零振幅后才能从相位随时间的变化估计频率修正。
            int dimensions = iteration == 0 || fixedFrequency ? 3 : 4;
            var normal = new double[dimensions, dimensions + 1];
            for (int i = 0; i < samples.Length; i++)
            {
                double time = (i - samples.Length / 2.0) / samples.Length;
                double angle = 2 * Math.PI * bins * time;
                double c = Math.Cos(angle), s = Math.Sin(angle);
                basis[0] = c; basis[1] = s; basis[2] = 1;
                // 第四列是波形对频率格数的导数，表示频率略变时该点改变多少。各列与误差按窗平方累加成正规方程。
                basis[3] = 2 * Math.PI * time * (-a * s + b * c);
                double error = samples[i] - a * c - b * s - dc;
                double weight = weights[i] * weights[i];
                for (int row = 0; row < dimensions; row++)
                {
                    normal[row, dimensions] += weight * basis[row] * error;
                    for (int col = 0; col < dimensions; col++)
                        normal[row, col] += weight * basis[row] * basis[col];
                }
            }
            // 求解得到的是参数增量，累加到上一轮解。奇异时保留已取得的解，固定频率只需一次线性求解。
            if (!Solve(normal, dimensions)) break;
            a += normal[0, dimensions]; b += normal[1, dimensions]; dc += normal[2, dimensions];
            if (dimensions == 4)
            {
                // 将每一步限制在四分之一格，累计限制在粗估的半格邻域，使迭代始终围绕同一候选峰。
                double correction = Math.Clamp(normal[3, dimensions], -0.25, 0.25);
                bins = Math.Clamp(bins + correction, initialBins - 0.5, initialBins + 0.5);
                if (Math.Abs(correction) < 1e-11) break;
            }
            else if (fixedFrequency) break;
        }
        return new Result(bins, a, b, dc);
    }

    // 用带主元的消元法处理增广矩阵。成功时最后一列就是各未知量的解。
    private static bool Solve(double[,] matrix, int n)
    {
        for (int column = 0; column < n; column++)
        {
            // 先用本列绝对值最大的可选元素作除数。若其仍太小，数据不足以可靠地区分这些参数。
            int pivot = column;
            for (int row = column + 1; row < n; row++)
                if (Math.Abs(matrix[row, column]) > Math.Abs(matrix[pivot, column])) pivot = row;
            if (Math.Abs(matrix[pivot, column]) < 1e-20) return false;
            for (int j = column; j <= n; j++)
                (matrix[pivot, j], matrix[column, j]) = (matrix[column, j], matrix[pivot, j]);
            double scale = matrix[column, column];
            // 主元行除到主元为 1，再从其余各行减去相应倍数，逐列把左侧化为单位矩阵。
            for (int j = column; j <= n; j++) matrix[column, j] /= scale;
            for (int row = 0; row < n; row++)
            {
                if (row == column) continue;
                double factor = matrix[row, column];
                for (int j = column; j <= n; j++) matrix[row, j] -= factor * matrix[column, j];
            }
        }
        return true;
    }
}
