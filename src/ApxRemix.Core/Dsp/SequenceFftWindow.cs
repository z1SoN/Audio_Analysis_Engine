using System.Numerics;
using System.Runtime.InteropServices;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

// Sequence 工作表先校正窗的能量，再做 FFT。交流耦合只抵消直流附近的窗谱。
internal sealed class SequenceFftWindow
{
    private static readonly Dictionary<(FftWindowType, bool), SequenceFftWindow> LastPerType = new();
    private readonly double[] coefficients;
    private readonly Complex[] dcShape;
    public int HalfWidth { get; }
    public double DcCorrection { get; }
    public double PeakCorrection { get; }

    // 相同工作表和窗型可复用最近一个长度的系数；帧长变化会改变采样网格，需要重新生成。
    public static SequenceFftWindow Get(int length, FftWindowType type)
    {
        lock (LastPerType)
        {
            var cacheKey = (type, SequenceBinEstimator.DesktopWorksheet);
            if (!LastPerType.TryGetValue(cacheKey, out var window) || window.coefficients.Length != length)
                LastPerType[cacheKey] = window = new SequenceFftWindow(length, type);
            return window;
        }
    }

    // 构造后同时保存加窗系数和直流泄漏的形状，后续每一帧共享它们。
    public SequenceFftWindow(int length, FftWindowType type)
    {
        HalfWidth = SequenceTransform.HalfWidth(type);
        int width = Math.Min(HalfWidth, length / 2 + 1);
        // 可用工作进程时一次取回归一化后的窗，开头两个标量用于从直流格或单音峰恢复幅值。
        var prepared = SequenceBinEstimator.PrepareWindow(length, type, width);
        if (prepared != null)
        {
            DcCorrection = prepared[0];
            PeakCorrection = prepared[1];
            coefficients = prepared[2..(length + 2)];
            dcShape = MemoryMarshal.Cast<double, Complex>(prepared.AsSpan(length + 2)).ToArray();
            return;
        }
        // 托管分支从原始窗出发，按当前工作表的累加顺序求能量归一化和两个修正因子。
        var window = FftEngine.CreateWindow(length, type);
        coefficients = window.Coefficients;
        var normalization = SequenceBinEstimator.DesktopWorksheet
            ? SequenceWindowNormalization.ComputeDesktop(coefficients)
            : SequenceWindowNormalization.Apply(coefficients);
        DcCorrection = normalization.DcCorrection;
        PeakCorrection = normalization.PeakCorrection;

        // 对全 1 的常量信号加窗等于直接变换窗系数；按直流幅值归一化后，就知道直流会泄漏到哪些邻格。
        var shape = ForwardManaged(coefficients);
        dcShape = shape[..width];
        dcShape[0] = new Complex(dcShape[0].Real, 0);
        for (int k = 0; k < dcShape.Length; k++) dcShape[k] *= DcCorrection;
    }

    // 每个样本乘以对应时刻的窗系数，然后做实数 FFT。输入长度不同将无法逐点配对。
    public Complex[] Transform(ReadOnlySpan<double> samples)
    {
        if (samples.Length != coefficients.Length) throw new ArgumentException("Window length mismatch.", nameof(samples));
        var weighted = new double[samples.Length];
        for (int i = 0; i < weighted.Length; i++) weighted[i] = samples[i] * coefficients[i];
        // 工作端返回交错的实虚分量，缺库时由本地实现生成相同形状的复数半谱。
        var interleaved = SequenceBinEstimator.ForwardReal(weighted);
        if (interleaved != null) return MemoryMarshal.Cast<double, Complex>(interleaved).ToArray();
        return ForwardManaged(weighted);
    }

    // 先挑当前工作表与帧长支持的专用蝶形，其余长度走通用复数 FFT。
    private static Complex[] ForwardManaged(double[] weighted)
    {
        if (SequenceBinEstimator.DesktopWorksheet && SequencePackedRealFft.SupportsDesktopBatch(weighted.Length))
            return SequencePackedRealFft.ForwardDesktopBatch(weighted);
        if (SequencePackedRealFft.Supports(weighted.Length))
            return SequencePackedRealFft.Forward(weighted, SequenceBinEstimator.DesktopWorksheet);
        // 实信号的负频率可由正频率共轭恢复，因此只交付从 0 到最高频的半谱。
        var fft = Array.ConvertAll(weighted, v => new Complex(v, 0));
        SequenceTransform.Forward(fft, stableRoots: true);
        // A real transform has no quadrature component at DC or an even record's Nyquist bin.
        fft[0] = new Complex(fft[0].Real, 0);
        if (fft.Length % 2 == 0) fft[fft.Length / 2] = new Complex(fft[fft.Length / 2].Real, 0);
        return fft[..(fft.Length / 2 + 1)];
    }

    // 用测得的零频实部乘直流模板，从主瓣覆盖的各复频格减去等量泄漏。
    public void SubtractDc(Complex[] spectrum)
    {
        double dcBin = spectrum[0].Real;
        spectrum[0] = new Complex(0, spectrum[0].Imaginary - dcBin * dcShape[0].Imaginary);
        for (int k = 1; k < dcShape.Length; k++) spectrum[k] -= dcBin * dcShape[k];
    }

    // 每个复频格先取模再平方。结果用于累加，所以此处还保留功率单位。
    public static double[] Power(Complex[] spectrum, int length, int count)
    {
        var power = SequenceBinEstimator.SquaredMagnitudes(MemoryMarshal.Cast<Complex, double>(spectrum.AsSpan(0, count)));
        // 缺库时工作表配置选择本地模长算法，两条路径都在平方前先形成一个幅值。
        if (power == null)
        {
            power = new double[count];
            bool desktop = SequenceBinEstimator.DesktopWorksheet;
            for (int k = 0; k < count; k++)
            {
                var bin = spectrum[k];
                double magnitude = desktop ? SequenceMagnitude.ComputeDesktop(bin.Real, bin.Imaginary)
                    : SequenceMagnitude.Compute(bin.Real, bin.Imaginary);
                power[k] = magnitude * magnitude;
            }
        }
        // 直流和偶数帧的奈奎斯特格各自只有一个实分量；将其功率折半以配合其余单边谱的标度。
        power[0] *= 0.5;
        if (count > length / 2) power[length / 2] *= 0.5;
        return power;
    }
}
