// 频率表要跟随交流变化，但录音还可能带直流偏置。StreamingBenchPipeline 为每个通道保留一个此滤波器，在主路径另有直流读数的同时，给过零检测提供交流副本。
namespace ApxRemix.Core.Dsp;

/// <summary>
/// APxRT zero-at-DC AC conditioning section. The 0.002 dB Chebyshev prototype
/// and 20 Hz lowpass-to-highpass transform are set in FUN_10007180 (10.0.0.17).
/// </summary>
public sealed class ApxAcCouplingFilter
{
    private readonly double _a1, _a2, _gain;
    private double _z1, _z2;
    // 设计跨音频块连续运行的交流耦合级。采样率决定一个采样周期内极点衰减和旋转多少。
    public ApxAcCouplingFilter(int sampleRate)
    {
        // 从 0.002 dB 纹波求出模拟原型的极点，再按采样周期取指数，得到离散时间的极点。
        double epsilon = Math.Sqrt(Math.Pow(10, 0.002 / 10) - 1);
        double v = Math.Asinh(1 / epsilon) / 2;
        var prototype = new System.Numerics.Complex(-Math.Sinh(v), Math.Cosh(v)) / Math.Sqrt(2);
        var pole = System.Numerics.Complex.Exp((2 * Math.PI * 20 / Math.Max(sampleRate, 1000)) / prototype);
        // 共轭极点配对后，分母系数分别为负的两倍实部和模平方，整个递推就只需实数乘加。
        _a1 = -2 * pole.Real;
        _a2 = pole.Real * pole.Real + pole.Imaginary * pole.Imaginary;
        // APxRT 10.0.0.17 takes the reciprocal of the biquad magnitude at Nyquist
        // (1000a960 → 10009440). Keep square, divide, square root, and reciprocal rounded separately.
        // 在奈奎斯特点 z⁻¹ = −1 代入分子和分母，定出高频端的单位幅度增益。
        double denominator = 1 - _a1 + _a2;
        _gain = 1 / Math.Sqrt(16 / (denominator * denominator));
    }

    // 输出沿用输入的幅度单位。每声道需要自己的实例，上一块留下的两个状态会作用于下一块的第一点。
    public void Process(ReadOnlySpan<double> input, Span<double> output)
    {
        if (output.Length < input.Length) throw new ArgumentException("Output buffer too short.");
        for (int i = 0; i < input.Length; i++)
        {
            // 展开下面的递推，输入部分是 gain·(x[n]−2x[n−1]+x[n−2])。恒定输入时三项相消；
            // 台阶刚到时三个样本还不同，所以交流支路也会短暂有输出，随后由反馈状态决定衰减。
            // 分子系数 1、−2、1 在直流处相加为零。恒定输入经过起始过渡后被滤掉。
            double scaled = input[i] * _gain;
            double value = scaled + _z1;
            _z1 = value * -_a1 + _z2 + scaled * -2;
            _z2 = value * -_a2 + scaled;
            output[i] = value;
        }
    }
}
