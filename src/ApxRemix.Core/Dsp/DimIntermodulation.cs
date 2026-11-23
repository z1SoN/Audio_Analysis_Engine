using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

public sealed class DimStimulus
{
    private const int TableSize = 8192;
    private readonly double[] square;
    public GeneratorWaveform Waveform { get; }
    public int Rate { get; }
    public double SquareHz { get; }
    public double SineHz { get; }

    // 判定发生器选择是否属于此处支持的四种方波加正弦 DIM 激励。
    public static bool IsDim(GeneratorWaveform waveform) =>
        waveform is GeneratorWaveform.Dim30 or GeneratorWaveform.Dim100 or GeneratorWaveform.DimB or GeneratorWaveform.DimB8;

    // 最高的正弦频率必须严格低于 Fs/2，否则采样点无法可靠表示该载波。
    public static bool SupportsRate(GeneratorWaveform waveform, int rate) =>
        rate > (waveform switch { GeneratorWaveform.DimB8 => 16000, GeneratorWaveform.DimB => 28000, _ => 30000 });

    // 根据模式选方波基频、叠加正弦及模拟 RC 截止频率（Hz）。后面预生成一个周期的方波供实时查表。
    public DimStimulus(GeneratorWaveform waveform, int rate)
    {
        Waveform = waveform;
        Rate = rate;
        (SquareHz, SineHz, double cutoff) = waveform switch
        {
            GeneratorWaveform.Dim30 => (3150, 15000, 30000),
            GeneratorWaveform.Dim100 => (3150, 15000, 100000),
            GeneratorWaveform.DimB => (2960, 14000, 30000),
            GeneratorWaveform.DimB8 => (2960, 8000, 30000),
            _ => throw new ArgumentOutOfRangeException(nameof(waveform))
        };
        if (rate <= 2 * SineHz) throw new ArgumentException("The sample rate must exceed twice the DIM sine frequency.");
        // Band-limit the ideal RC-filtered square to the simulator Nyquist band.
        var spectrum = new Complex[TableSize];
        // 对称方波的傅里叶级数只含奇次谐波，幅度按 1/h 衰减。每个谐波既要低于输出奈奎斯特，也要能装进表。
        for (int h = 1; h * SquareHz < rate * .5 && h < TableSize / 2; h += 2)
        {
            // 乘以 1/(1+jf/fc) 同时衰减和滞后各次谐波。补齐共轭负频率后，逆变换得到实值波形。
            Complex rc = Complex.One / new Complex(1, h * SquareHz / cutoff);
            spectrum[h] = new Complex(0, -2d * TableSize / (Math.PI * h)) * rc;
            spectrum[TableSize - h] = Complex.Conjugate(spectrum[h]);
        }
        FftEngine.InverseInPlace(spectrum);
        // 以查表波形本身的最大绝对值归一，保证后续方波与正弦的系数具有明确的相对峰值。
        double peak = spectrum.Max(z => Math.Abs(z.Real));
        square = Array.ConvertAll(spectrum, z => z.Real / peak);
    }

    // 两个相位均用弧度，分别取样后按 4:1 峰值比例相加，得到单个归一化激励样本。
    public double Sample(double squarePhase, double sinePhase) => .8*SquareSample(squarePhase)+.2*Math.Sin(sinePhase);

    // 把弧度换成表内小数下标。相邻两点线性插值，掩码按 2 的幂表长回绕，跨周期接缝也适用。
    public double SquareSample(double squarePhase)
    {
        double at = squarePhase * TableSize / Math.Tau;
        int index = (int)Math.Floor(at);
        double a = square[index & (TableSize - 1)];
        double b = square[(index + 1) & (TableSize - 1)];
        return a + (at - index) * (b - a);
    }
}

internal static class DimEmulation
{
    // Frequency and amplitude calibration points of the legacy DIM residual meter.
    private static readonly double[] Frequencies = [42,110.3,120.3,131.1,141.4,154.2,164.5,177.4,183.2,193.4,201.9,234.9,250.6,264.5,285.2,328.2,416.2,569.1,997.7,2392,2418,2470,2552,2812,3000,96000];
    private static readonly double[] GainsDb = [-60,-34.8,-33.2,-32.6,-33.7,-38.3,-45.7,-60,-60,-45.8,-36.6,-19.5,-15.6,-13.2,-10.6,-6.9,-2.8,-.6,0,0,-.2,-2.3,-15,-60,-200,-200];
    private static readonly double[] Derivatives = Slopes();

    // 求每个频率节点处 dB 对 Hz 的斜率，供后面分段三次插值，同向斜率才能延续趋势。
    private static double[] Slopes()
    {
        int n = Frequencies.Length;
        var h = new double[n-1];
        var d = new double[n-1];
        var slopes = new double[n];
        for (int i = 0; i < n-1; i++)
        {
            h[i] = Frequencies[i+1] - Frequencies[i];
            d[i] = (GainsDb[i+1]-GainsDb[i]) / h[i];
        }
        // 相邻割线异号意味着局部极值，此时导数留为零。同号时取加权调和平均，降低节点之间的过冲。
        for (int i = 1; i < n-1; i++)
        {
            if (d[i-1] * d[i] <= 0) continue;
            double w1 = 2*h[i]+h[i-1], w2 = h[i]+2*h[i-1];
            slopes[i] = (w1+w2)/(w1/d[i-1]+w2/d[i]);
        }
        // 端点只有一侧数据可用，按两条邻近割线外推并限制符号和大小，以延续端部曲线的走向。
        static double End(double h1, double h2, double d1, double d2)
        {
            double value = ((2*h1+h2)*d1-h1*d2)/(h1+h2);
            if (Math.Sign(value) != Math.Sign(d1)) return 0;
            return Math.Sign(d1) != Math.Sign(d2) && Math.Abs(value) > 3*Math.Abs(d1) ? 3*d1 : value;
        }
        slopes[0] = End(h[0],h[1],d[0],d[1]);
        slopes[^1] = End(h[^1],h[^2],d[^1],d[^2]);
        return slopes;
    }

    // 查出该频率处的模型功率权重。先在 dB 刻度上按端点值和导数插值，再以 10^(dB/10) 转回功率比。
    private static double PowerWeight(double hz)
    {
        int index = Array.BinarySearch(Frequencies, hz);
        if (index >= 0) return Math.Pow(10, GainsDb[index]/10);
        // 二分查找的按位补码给出插入位置。夹住区间索引，区间外的频率沿端段公式外推。
        int j = Math.Clamp(~index - 1, 0, Frequencies.Length - 2);
        double h = Frequencies[j+1]-Frequencies[j];
        double t = (hz-Frequencies[j])/h, u = 1-t;
        double db = u*u*((1+2*t)*GainsDb[j]+t*h*Derivatives[j])
            + t*t*((3-2*t)*GainsDb[j+1]-u*h*Derivatives[j+1]);
        return Math.Pow(10, db/10);
    }

    private static readonly object WeightGate = new();
    private static (int Rate, int Length, double[] Weights) _cachedWeights;

    // 对均匀 FFT 频格预先计算各点权重，只有采样率与变换长度都一致时才能复用旧表。
    private static double[] Weights(int length, int rate)
    {
        lock (WeightGate)
        {
            if (_cachedWeights.Rate != rate || _cachedWeights.Length != length)
            {
                double df = (double)rate/length;
                _cachedWeights = (rate, length, Enumerable.Range(0, length/2+1).Select(k => PowerWeight(k*df)).ToArray());
            }
            return _cachedWeights.Weights;
        }
    }

    // 分析近期的一段 PCM。确认 DIM 激励存在后，先消去方波的低阶分量，再按模型频响积分残余功率。
    public static ImdChannelResult Analyze(ReadOnlySpan<double> samples, int rate, int length)
    {
        const string label = "DIM Emulation";
        // 无法确认载波、输入采样率或记录长度时返回缺失读数并附原因，避免把无效数据解释为零失真。
        ImdChannelResult Invalid(string reason) => new(false, label, double.NaN, double.NaN, reason);
        if (length < 2048 || rate <= 16000 || rate > 192000) return Invalid("DIM requires the carrier and an input bandwidth of 90 kHz or less.");
        var data = samples[^length..];
        var window = FftEngine.CreateWindow(length, FftWindowType.BlackmanHarris4);
        var fft = SequenceTransform.Windowed(data, window);
        double df = (double)rate / length;
        var power = SequenceTransform.Power(fft, length / 2 + 1, window);
        // 在最强谱峰周围取功率重心。根据其靠近 2960 或 3150 Hz 判断激励族，并检查偏差和峰值大小。
        int peak = 1;
        for (int k=2; k<power.Length; k++) if (power[k] > power[peak]) peak = k;
        double squareBin = SequenceTransform.Centroid(power, peak, 4);
        double squareHz = squareBin * df;
        bool broadcast = Math.Abs(squareHz-2960) < Math.Abs(squareHz-3150);
        if (!(power[peak] > 1e-12) || !double.IsFinite(squareHz) || Math.Abs(squareHz-(broadcast ? 2960 : 3150)) > 45)
            return Invalid("A DIM square-wave carrier is required.");
        // 单个正弦经加窗后会占据几格。把中心及左右四格的能量相加，靠近边缘时返回零。
        double AtTone(double frequency)
        {
            int centre = (int)Math.Round(frequency / df);
            if (centre < 5 || centre+4 >= power.Length) return 0;
            double energy = 0;
            for (int k = centre-4; k <= centre+4; k++) energy += power[k];
            return energy;
        }
        // 除方波基频外必须看到相应正弦和方波三次谐波，降低单音被误认成有效 DIM 的概率。
        double sinePower = broadcast ? Math.Max(AtTone(8000), AtTone(14000)) : AtTone(15000);
        if (sinePower < power[peak]*1e-4 || AtTone(3*squareHz) < power[peak]*1e-4)
            return Invalid("The DIM sine component or square-wave harmonics are missing.");

        double total = power.Sum();
        var residual = data.ToArray();
        // 对奈奎斯特以下的第 1、3、5 次分量逐一测复振幅。复共轭投影记录幅度和相位，足以重建该正弦。
        for (int h=1; h<=5 && h*squareHz<rate*.5; h+=2)
        {
            double increment = Math.Tau * h * squareBin / length;
            Complex advance = Complex.FromPolarCoordinates(1, increment);
            Complex phase = Complex.One, projection = Complex.Zero;
            for (int i=0; i<length; i++)
            {
                projection += residual[i] * window.Coefficients[i] * Complex.Conjugate(phase);
                phase *= advance;
            }
            // 以窗的相干增益恢复单边幅度，再逐点减去复正弦的实部，避免方波本身的强分量进入残余积分。
            projection *= 2 / (length * window.CoherentGain);
            phase = Complex.One;
            for (int i=0; i<length; i++)
            {
                residual[i] -= (projection * phase).Real;
                phase *= advance;
            }
        }
        // 对剩余波形重新变换，加权残余功率除以原始总功率。33 是这里 DIM 模型采用的比例系数，开方后得到幅度比。
        var residualSpectrum = SequenceTransform.Windowed(residual, window);
        var residualPower = SequenceTransform.Power(residualSpectrum, power.Length, window);
        double weightedPower = 0;
        var weights = Weights(length, rate);
        for (int k=0; k<power.Length; k++) weightedPower += residualPower[k] * weights[k];
        double ratio = Math.Sqrt(33 * weightedPower / total);
        return new(true,label,100*ratio,20*Math.Log10(Math.Max(1e-15,ratio)));
    }
}
