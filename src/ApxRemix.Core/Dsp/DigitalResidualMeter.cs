// 输入是主路径的波形，输出是陷波和残余计权后的线性 RMS。StreamingBenchReading 另取交流参考来算 THD+N 百分比，所以在这里还看不到最终表头的分母。
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>Digital-domain residual from a tuned sixth-order notch and a continuous RMS detector.</summary>
internal sealed class DigitalResidualMeter
{
    private readonly int _sampleRate;
    private readonly ApxRmsDetector _rms;
    private readonly ApxAcCouplingFilter _residualAc;
    private StreamingIirCascade _notch = StreamingIirCascade.Bypass();
    private StreamingIirCascade _weight = StreamingIirCascade.Bypass();
    private AnalyzerWeighting _weighting;
    private double _center;

    // 单声道残余仪表默认使用 Bench 的常规刷新节拍。
    public DigitalResidualMeter(int sampleRate) : this(sampleRate, StreamingBenchPipeline.DefaultReadingsPerSecond) { }

    // 按采样率和报告速率建立 RMS 检波，给残余通路单独分配交流耦合状态。
    public DigitalResidualMeter(int sampleRate, double readingsPerSecond)
    {
        _sampleRate = sampleRate;
        _rms = new(sampleRate, readingsPerSecond);
        _residualAc = new(sampleRate);
    }

    // 输入是上游已处理的工作区，按顺序就地陷波、计权、条件交流耦合，最后送入持续运行的 RMS 检波器。
    public void Process(Span<double> workspace, double centerHz, AnalyzerWeighting weighting, bool dcCoupled)
    {
        double center = Math.Clamp(centerHz, 5, _sampleRate * 0.495);
        // 中心变化超过相对阈值才换陷波器，新实例从空历史开始。很小的波动可沿用上次滤波状态。
        if (Math.Abs(center - _center) > 0.0001 * _center)
        {
            _notch = DesignNotch(center, _sampleRate);
            _center = center;
        }
        // 计权类型改变才单独重建计权级，两个滤波器可以在不同包边界切换。
        if (weighting != _weighting)
        {
            _weight = ApxAnsiWeighting.Create(weighting, _sampleRate);
            _weighting = weighting;
        }
        // 设输入含 1 kHz 基波和 3 kHz 谐波。陷波中心对准 1 kHz 后，留下的 3 kHz 和带内噪声一起进入 RMS。
        // 调错中心，漏过的基波也会被计入残余。这解释了同一电路为何会随调谐设置显示不同的 THD+N。
        _notch.ProcessInPlace(workspace);
        _weight.ProcessInPlace(workspace);
        // 主路径开直流档时，直流参与主电平；在这条残余支路上再交流耦合，稳定的直流偏置会被压下。
        // 若把这个步骤挪到分路之前，主电平的定义也会随之改变。
        // 主通路处于直流耦合时在残余支路另行抑制直流，防止偏置进入失真加噪声幅值。
        if (dcCoupled) _residualAc.Process(workspace, workspace);
        _rms.Process(workspace);
    }

    // 读出从上次报告至今的检波结果，单位仍是线性样本幅度。
    public double Read() => _rms.Read();

    // 将这段工作区处理并立即出数。连续分块调用 Process 后统一 Read 可控制报告边界。
    public double Measure(Span<double> workspace, double centerHz, AnalyzerWeighting weighting, bool dcCoupled)
    {
        Process(workspace, centerHz, weighting, dcCoupled);
        return Read();
    }

    // 以目标 Hz 设计三节带阻。预畸变同时约束中心及其半频边缘，使数字频率轴上的带宽可控。
    private static StreamingIirCascade DesignNotch(double center, int fs)
    {
        // Third-order 0.01 dB Chebyshev prototype. Prewarp the notch and its
        // lower half-frequency edge before the analog bandstop substitution.
        double w = Math.Tan(Math.PI * center / fs);
        double edge = Math.Tan(Math.PI * center / (2 * fs));
        // 双线性变换使数字频率与原型频率呈 tan 关系，先把中心和下边缘换到同一原型坐标。
        // 带阻变换的两边缘满足 下边缘×上边缘=w²，所以这里用 w²/edge 求上边缘，再求两边缘之差。
        double bandwidth = w * w / edge - edge;
        double ripple = Math.Pow(10, 0.01 / 10);
        double mu = Math.Asinh(1 / Math.Sqrt(ripple - 1)) / 3;
        // 三阶切比雪夫原型的极点经过带阻变换，每个原型极点产生两个带阻极点。
        var complexPole = new Complex(-Math.Sinh(mu) / 2, Math.Cos(Math.PI / 6) * Math.Cosh(mu));
        Complex middle = bandwidth / (2 * complexPole);
        Complex offset = Complex.Sqrt(middle * middle - w * w);
        Complex realMiddle = -bandwidth / (2 * Math.Sinh(mu));
        Complex realOffset = Complex.Sqrt(realMiddle * realMiddle - w * w);
        // 将已预畸变的模拟根映到 z 平面，左半平面的极点映入单位圆。
        Complex Map(Complex analog) => (1 + analog) / (1 - analog);
        // 每节分子是 1−2cos(ω₀)z⁻¹+z⁻²。把中心频率对应的 z=e^(jω₀) 代入就得到零，
        // 中心正弦因此在稳态被抑制。极点控制邻近频率的衰减，也决定输入突变后要多久才能稳定。
        // 单位圆上的共轭零点位于目标角频率，分子在该正弦频率处为零。
        double b1 = -2 * Math.Cos(2 * Math.PI * center / fs);
        // 将一对已映射极点组合成实系数分母。按 z=1 处的分子分母之和定出该节直流增益。
        StreamingIirCascade.BiquadCoeffs Section(Complex first, Complex second)
        {
            Complex p = Map(first), q = Map(second);
            double a1 = -(p + q).Real, a2 = (p * q).Real;
            double gain = (1 + a1 + a2) / (2 + b1);
            return new(gain, gain * b1, gain, a1, a2);
        }
        Complex high = middle + offset, low = middle - offset;
        return new([
            Section(high, Complex.Conjugate(high)),
            Section(low, Complex.Conjugate(low)),
            Section(realMiddle + realOffset, realMiddle - realOffset)
        ], Math.Sqrt(Math.Sqrt(ripple)));
    }
}
