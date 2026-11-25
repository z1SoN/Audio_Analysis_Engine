using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

// Sequence 会把下一帧长度选成例如 1536 点。把它补成 2048 点再直接读同一格，
// 读到的频率就从 k·fs/1536 变成了 k·fs/2048。这里始终保留采集帧自己的频率网格。
internal static class SequenceTransform
{
    // 从 10.0.0.17 的 GoodLengths 公共函数逐个查询一个倍频程。更长的档位按 2 倍平移。
    // 允许的常规帧长先列出 1024 到 2048 之间的一组基本长度，更长的帧由它们乘二次幂得到。
    private static readonly int[] Lengths = [1024,1040,1056,1080,1100,1120,1152,1176,1200,1232,
        1280,1296,1344,1372,1400,1440,1500,1512,1536,1568,1600,1620,1664,1728,1760,1872,1920,1944,2016];
    // 短帧另列支持点数，仅在调用者允许少于 1024 点时使用。
    private static readonly int[] ShortLengths = [2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,
        36,40,42,44,48,50,52,54,56,60,64,66,70,72,78,80,84,88,90,96,98,100,104,108,110,112,
        120,126,128,130,132,140,144,150,154,156,160,162,168,176,180,192,196,200,208,216,220,224,
        234,240,250,252,256,260,264,270,280,288,294,300,308,312,320,324,336,350,360,378,384,
        392,400,416,432,440,468,480,486,504,512,520,528,540,550,560,576,588,600,616,640,648,
        672,686,700,720,750,756,768,784,800,810,832,864,880,936,960,972,1008];

    // 选大于等于目标的最短可用长度。小目标先查短表，其余按倍数从小到大搜索，转换溢出时抛错。
    internal static int NextLength(int target, bool allowShort = false)
    {
        if (allowShort && target < 1024)
            foreach (int length in ShortLengths)
                if (length >= target) return length;
        for (long scale = 1; ; scale *= 2)
            foreach (int length in Lengths)
                if (length * scale >= target) return checked((int)(length * scale));
    }

    // 直接覆盖输入为未归一化的复频谱；二次幂长度可原地 FFT，其他长度用 Bluestein 算法转成卷积。
    internal static void Forward(Complex[] data, bool stableRoots = false)
    {
        if (FftEngine.IsPowerOfTwo(data.Length)) { FftEngine.ForwardInPlace(data, stableRoots); return; }
        // Bluestein 恒等式把 nk 写成三个平方项。中间的一项变为卷积，临时补零只用于做卷积。
        int n = data.Length;
        // 线性卷积至少需要 2n−1 点，向上取二次幂，余下位置置零以免两端绕回相加。
        int size = checked((int)BitOperations.RoundUpToPowerOf2(checked((uint)(2L * n - 1))));
        var input = new Complex[size];
        var kernel = new Complex[size];
        for (int i = 0; i < n; i++)
        {
            // 由 i² 的相角生成单位复数，用共轭调制输入；核同时摆在正、负索引对应位置。
            double angle = Math.PI * ((long)i * i % (2L * n)) / n;
            kernel[i] = Complex.FromPolarCoordinates(1, angle);
            if (i > 0) kernel[size - i] = kernel[i];
            input[i] = data[i] * Complex.Conjugate(kernel[i]);
            data[i] = Complex.Conjugate(kernel[i]);
        }
        // 卷积等于频谱逐点相乘后逆变换，再乘一次先前保存的啁啾因子，即还原所求 n 点 DFT。
        FftEngine.ForwardInPlace(input, stableRoots);
        FftEngine.ForwardInPlace(kernel, stableRoots);
        for (int i = 0; i < size; i++) input[i] *= kernel[i];
        FftEngine.InverseInPlace(input, stableRoots);
        for (int i = 0; i < n; i++) data[i] *= input[i];
    }

    // 利用逆变换等于“取共轭、正变换、再取共轭”；normalize 启用时除以点数恢复时域标度。
    internal static void Inverse(Complex[] data, bool normalize = true, bool stableRoots = false)
    {
        for (int i = 0; i < data.Length; i++) data[i] = Complex.Conjugate(data[i]);
        Forward(data, stableRoots);
        for (int i = 0; i < data.Length; i++)
        {
            data[i] = Complex.Conjugate(data[i]);
            if (normalize) data[i] /= data.Length;
        }
    }

    // 各种窗展宽一根纯音谱线的程度不同；该表以频格为单位给出后续峰带积分的半宽。
    internal static int HalfWidth(FftWindowType window) => window switch
    {
        FftWindowType.Rectangular or FftWindowType.NoneMoveToBinCenter => 1,
        FftWindowType.Hann or FftWindowType.BlackmanHarris => 3,
        FftWindowType.BlackmanHarris4 => 4,
        FftWindowType.FlatTop or FftWindowType.Dolph150 => 5,
        FftWindowType.Dolph200 => 6,
        FftWindowType.Dolph250 => 7,
        FftWindowType.Equiripple => 7,
        _ => throw new ArgumentOutOfRangeException(nameof(window))
    };

    // 逐点从样本减给定直流量再加窗，整段变换后返回复数全谱，保留幅度与相位。
    internal static Complex[] Windowed(ReadOnlySpan<double> samples, FftEngine.WindowData window, double dc = 0,
        bool stableRoots = false)
    {
        var bins = new Complex[samples.Length];
        for (int i = 0; i < bins.Length; i++) bins[i] = (samples[i] - dc) * window.Coefficients[i];
        Forward(bins, stableRoots);
        return bins;
    }

    // 用 N² 及窗的功率增益把未归一化 FFT 换成均方单位。单边谱内部格双倍计功率，两端各只算一次。
    internal static double[] Power(Complex[] bins, int count, FftEngine.WindowData window)
    {
        // 一格代表这一小段频带贡献的均方值。正、负频率配成一对，直流和偶数长度的奈奎斯特格各自只有一份。
        double scale = 1.0 / ((double)bins.Length * bins.Length * window.NoisePowerGain);
        var power = new double[count];
        for (int k = 0; k < count; k++)
            power[k] = (bins[k].Real * bins[k].Real + bins[k].Imaginary * bins[k].Imaginary)
                * scale * (k == 0 || k * 2 == bins.Length ? 1 : 2);
        return power;
    }

    // 以每格功率为权求频格重心，邻域到边界时对称缩小。全零区域因权重总和为零没有有效重心。
    internal static double Centroid(double[] power, int center, int width)
    {
        int radius = Math.Min(width, Math.Min(center, power.Length - 1 - center));
        double moment = 0, mass = 0;
        for (int k = center - radius; k <= center + radius; k++)
        {
            mass += power[k];
            moment += k * power[k];
        }
        return moment / mass;
    }

    // 克隆功率谱以保留原数据；遮掉直流附近的主瓣和基频整数倍附近的格，估计剩余的非周期量。
    internal static double NonPeriodicLevel(double[] power, double center, int width)
    {
        // 先划掉直流和周期分量所占的频格，再用随频率增加的权重估计剩余噪声。
        var masked = (double[])power.Clone();
        Array.Clear(masked, 0, Math.Min(width + 1, masked.Length));
        // 中心离直流足够远时最多去掉二十次谐波，否则只去基波；每次遮罩都限制在数组边界。
        if (double.IsFinite(center))
            for (int order = 1; order <= (Math.Round(center) > width ? 20 : 1); order++)
            {
                int start = Math.Max(0, (int)Math.Round(order * center) - width);
                if (start >= masked.Length) break;
                Array.Clear(masked, start, Math.Min(2 * width + 1, masked.Length - start));
            }
        // 剩余功率随格序号线性加权，高频权重大于低频；开方后才能与主峰的幅值作比。
        double weighted = 0;
        for (int k = 0; k < masked.Length; k++) weighted += masked[k] * k / (masked.Length - 1);
        return Math.Sqrt(weighted);
    }
}
