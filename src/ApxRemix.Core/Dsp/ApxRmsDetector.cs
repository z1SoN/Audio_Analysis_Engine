// 主电平和残余电平都要把一串采样变成随时间更新的幅度。本类只对交进来的波形做包络检波，各支路分别持有自己的实例，最后才由上层换算显示单位。
namespace ApxRemix.Core.Dsp;

/// <summary>
/// Scalar port of APxRT 10.0.0.17 RMS detector (10003670), including coefficient
/// generation (10010070/1000fbf0). Input is already conditioned PCM. This stage
/// returns linear RMS; AES17/dBFS conversion belongs to the following unit stage.
/// </summary>
public sealed class ApxRmsDetector
{
    private double _power, _z1, _z2, _post, _sum;
    private int _count;
    private readonly double _powerK, _powerA, _b0, _b1, _a1, _a2, _postK, _postA;

    // 这里建立数字检波器的时间常数。Fs 控制每个样本的步进量，报告速率选择快慢参数组。
    public ApxRmsDetector(int sampleRate, double readingsPerSecond = 20)
    {
        double fs = Math.Max(1000, sampleRate);
        bool slow = readingsPerSecond < 32;
        double c = slow ? 2e-6 : 1e-6;
        double r = slow ? 27000 : 4000;
        double f = slow ? 37 : 95;
        // 由两个时间常数的和与积写出二阶特征方程，再求共轭极点，决定开方后平滑级的衰减和振荡。
        double inverseTauSum = 1 / (r * 4.7e-6) + 54.55537370430988;
        double inverseTauProduct = 1 / (r * 4.7e-6 * 3900 * 1e-6);
        double realPole = -inverseTauSum / 2;
        double imaginaryPole = Math.Sqrt(4 * inverseTauProduct - inverseTauSum * inverseTauSum) / 2;
        double inverseNormalization = 1 / (2 * Math.PI * f);
        double gain = (realPole * realPole + imaginaryPole * imaginaryPole) * inverseNormalization * inverseNormalization;
        // 1/fs 是每一步的秒数。这里 K=Δt/(RC)，采样率提高一倍，每步吸收的新功率约减半，
        // 同一秒内却会更新两倍次数。要复现表头的响应时间，两者必须一起考虑。
        _powerK = (1 / (c * 25000)) * (1 / fs);
        _powerA = _powerK - 1;
        double real = realPole * (1 / fs), imaginary = imaginaryPole * (1 / fs);
        double magnitudeSquared = real * real + imaginary * imaginary;
        _a1 = (1 + real) * -2;
        _a2 = 2 * real + 1 + magnitudeSquared;
        _b1 = magnitudeSquared - gain * 2;
        _b0 = gain;
        _postK = (1 / fs) * 141.84397163120568;
        _postA = _postK - 1;
    }

    // 每点依次经历平方、平滑、开方和读数平滑。只要历史不同，同一包 PCM 也可能产生不同读数。
    /// <summary>Keep all filter states across blocks; only Read clears the report accumulator.</summary>
    public void Process(ReadOnlySpan<double> samples)
    {
        foreach (double x in samples)
        {
            // 把下两行合起来就是 P新=(1−K)P旧+Kx²。正负半周都贡献功率。
            // 若输入突然变成零，旧功率仍按 (1−K) 逐点衰减；开方后得到与输入幅值同单位的包络。
            double inputPower = x * x * _powerK;
            _power = _power * -_powerA + inputPower;
            double root = Math.Sqrt(_power);
            // 这段二阶滤波和随后的 _post 继续平滑包络。_sum 累计的是每个时刻已经平滑过的读数，
            // 因此仅拿当前帧求一次平方平均再开方，无法重建持续测量中的瞬态。
            double output = root * _b0 + _z1;
            _z1 = root * _b1 + _z2 - output * _a1;
            _z2 = output * -_a2 + root * _b0;
            _post = _post * -_postA + output * _postK;
            _sum += _post;
        }
        _count += samples.Length;
    }

    // 返回本次报告期间的平均检波输出。清空报告和与点数，滤波器状态继续传给下一报告。
    public double Read()
    {
        double rms = _count == 0 ? double.NaN : _sum / _count;
        // 这一帧已交给读者，清空计数和报告总和。_power、_z1、_z2、_post 继续工作，
        // 下一帧的第一个采样点会继承上一帧最后一个采样点留下的状态。
        _count = 0; _sum = 0;
        return rms;
    }
}
