// 把可设置的增益、滤波、延时等作用在生成的波形上，构成虚拟被测电路。测试预先知道设置的变化，就能检查后端能否从采样反推出对应读数。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// 虚拟被测件（Virtual DUT）信号链路处理器。
/// <para>
/// 在“内置虚拟环回仿真”模式下，模拟一台具有可调通带增益、高/低通频响滚降、
/// 可控二/三次谐波失真（采用 Chebyshev 正交多项式实现精准谐波注入）、
/// 通道间增益/延时偏差以及宽带高斯白噪声底噪的真实音频设备。
/// </para>
/// </summary>
public sealed class VirtualDutProcessor
{
    // 双通道一阶高通/低通滤波器的前一步输入与输出状态
    private double _hpX1L, _hpY1L, _hpX1R, _hpY1R;
    private double _lpX1L, _lpY1L, _lpX1R, _lpY1R;

    // 通道 2 的环形缓冲有 256 点。为相邻点插值预留位置后，ProcessBlock 把可设置延时限制到 254 点。
    private readonly double[] _ch2DelayLine = new double[256];
    private int _delayWriteIndex;

    // Box–Muller 产生标准正态变量，按指定标准差缩放后给出目标噪声 RMS；有限记录的实测 RMS 会随机波动。
    private readonly Random _rng = new(987654321);
    private bool _hasSpareGaussian;
    private double _spareGaussian;

    /// <summary>
    /// 将输入立体声激励信号通过虚拟 DUT 模型处理，生成仿真采集信号。
    /// </summary>
    /// <param name="config">虚拟 DUT 物理参数。</param>
    /// <param name="referenceInputPeak">
    /// 当前激励信号的标称峰值幅度（线性值），用于校准 Chebyshev 二/三次谐波注入百分比，
    /// 当非线性级接收到幅度匹配的纯正弦时，对应的二、三次谐波与基波的幅度比分别等于该级设定值。
    /// </param>
    /// <param name="sampleRate">采样率（Hz）。</param>
    /// <param name="leftIn">输入左声道（激励信号）。</param>
    /// <param name="rightIn">输入右声道（激励信号）。</param>
    /// <param name="leftOut">输出左声道（仿真采集信号）。</param>
    /// <param name="rightOut">输出右声道（仿真采集信号）。</param>
    public void ProcessBlock(
        VirtualDutConfig config,
        double referenceInputPeak,
        int sampleRate,
        ReadOnlySpan<double> leftIn,
        ReadOnlySpan<double> rightIn,
        Span<double> leftOut,
        Span<double> rightOut)
    {
        int count = Math.Min(Math.Min(leftIn.Length, rightIn.Length), Math.Min(leftOut.Length, rightOut.Length));

        // 1. 计算通带线性增益
        // 用 10^(dB/20) 把电压增益换成样本乘数；右路单独附加增益偏差，后面才有可控的通道不平衡。
        double gainL = Math.Pow(10.0, config.GainDb / 20.0);
        double gainR = Math.Pow(10.0, (config.GainDb + config.Channel2GainOffsetDb) / 20.0);

        // 2. 双线性变换计算一阶高通与一阶低通滤波器系数（精确保证截止频率处 -3.01 dB）
        double hpFc = Math.Clamp(config.HighPassCutoffHz, 0.5, sampleRate * 0.45);
        double lpFc = Math.Clamp(config.LowPassCutoffHz, 50.0, sampleRate * 0.48);

        // 一阶高通去低频、低通去高频。按当前 Fs 计算数字系数，使设置的截止频率以 Hz 表示，前一输入及输出保存在各路独立状态中。
        double kHp = Math.Tan(Math.PI * hpFc / sampleRate);
        double hpB0 = 1.0 / (1.0 + kHp);
        double hpB1 = -hpB0;
        double hpA1 = (kHp - 1.0) / (1.0 + kHp);

        double kLp = Math.Tan(Math.PI * lpFc / sampleRate);
        double lpB0 = kLp / (1.0 + kLp);
        double lpB1 = lpB0;
        double lpA1 = (kLp - 1.0) / (1.0 + kLp);

        // 3. 谐波注入系数（将百分比转换为线性比例）
        double h2 = Math.Clamp(config.SecondHarmonicPercent, 0.0, 20.0) / 100.0;
        double h3 = Math.Clamp(config.ThirdHarmonicPercent, 0.0, 20.0) / 100.0;

        // 4. 高斯白噪声标准差（AES17：0 dBFS 正弦 RMS = 1/sqrt(2)，因此线性噪声 RMS = 10^(dBFS/20) / sqrt(2)）
        // 噪声设置按满幅正弦 RMS 标尺换算为样本标准差。每路独立加噪，随后分析器看到的底噪取决于分析带宽。
        double noiseSigma = Math.Pow(10.0, config.NoiseFloorDbfs / 20.0) / Math.Sqrt(2.0);

        // 5. 通道 2 分数延时量
        double delaySamples = Math.Clamp(config.Channel2DelaySamples, 0.0, _ch2DelayLine.Length - 2);

        // 为每个通道建立有效参考幅值（防止静音时除零）
        double refPeakL = Math.Max(referenceInputPeak * gainL, 1e-6);
        double refPeakR = Math.Max(referenceInputPeak * gainR, 1e-6);
        bool applyHarmonics = referenceInputPeak > 1e-5 && (h2 > 0.0 || h3 > 0.0);

        for (int i = 0; i < count; i++)
        {
            // ---- Step A: 应用通带增益 ----
            double sL = leftIn[i] * gainL;
            double sR = rightIn[i] * gainR;

            // ---- Step B: 应用一阶高/低通频响滤波器 ----
            if (config.EnableBandwidthFilter)
            {
                // 高通级联（Ch1 & Ch2）
                double yHpL = hpB0 * sL + hpB1 * _hpX1L - hpA1 * _hpY1L;
                _hpX1L = sL;
                _hpY1L = yHpL;

                double yHpR = hpB0 * sR + hpB1 * _hpX1R - hpA1 * _hpY1R;
                _hpX1R = sR;
                _hpY1R = yHpR;

                // 低通级联（Ch1 & Ch2）
                double yLpL = lpB0 * yHpL + lpB1 * _lpX1L - lpA1 * _lpY1L;
                _lpX1L = yHpL;
                _lpY1L = yLpL;

                double yLpR = lpB0 * yHpR + lpB1 * _lpX1R - lpA1 * _lpY1R;
                _lpX1R = yHpR;
                _lpY1R = yLpR;

                sL = yLpL;
                sR = yLpR;
            }

            // ---- Step C: 通道 2 可选分数延时（用于仿真通道间相位差）----
            _ch2DelayLine[_delayWriteIndex] = sR;
            // 从环形延时线读出比写入位置更早的样本。非整数延迟在相邻两点间线性插值，得到近似的亚样本通道时差。
            if (delaySamples > 1e-6)
            {
                double readPos = _delayWriteIndex - delaySamples;
                if (readPos < 0)
                    readPos += _ch2DelayLine.Length;
                int idx0 = (int)Math.Floor(readPos) % _ch2DelayLine.Length;
                int idx1 = (idx0 + 1) % _ch2DelayLine.Length;
                double frac = readPos - Math.Floor(readPos);
                sR = _ch2DelayLine[idx0] * (1.0 - frac) + _ch2DelayLine[idx1] * frac;
            }
            _delayWriteIndex = (_delayWriteIndex + 1) % _ch2DelayLine.Length;

            // ---- Step D: Chebyshev 正交多项式注入精确二/三次谐波 ----
            // 利用恒等式：2*sin²(θ)-1 = -cos(2θ)，4*sin³(θ)-3*sin(θ) = -sin(3θ)
            // 当进入多项式的是无直流、峰值恰等于归一参考的一只正弦时，这两项只生成对应阶次。预先经过滤波衰减的音调或多音输入不满足这一条件。
            if (applyHarmonics)
            {
                // 若 u=cos θ，则 2u²−1=cos 2θ，4u³−3u=cos 3θ。以参考峰值归一后使用这些多项式，可给测试正弦注入可预期的二、三次分量。
                double uL = sL / refPeakL;
                double distL = h2 * (2.0 * uL * uL - 1.0) + h3 * (4.0 * uL * uL * uL - 3.0 * uL);
                sL += distL * refPeakL;

                double uR = sR / refPeakR;
                double distR = h2 * (2.0 * uR * uR - 1.0) + h3 * (4.0 * uR * uR * uR - 3.0 * uR);
                sR += distR * refPeakR;
            }

            // ---- Step E: 叠加宽带高斯白噪声并模拟 ADC 满量程削波 ----
            if (noiseSigma > 0.0)
            {
                sL += NextGaussian() * noiseSigma;
                sR += NextGaussian() * noiseSigma;
            }

            leftOut[i] = Math.Clamp(sL, -1.0, 1.0);
            rightOut[i] = Math.Clamp(sR, -1.0, 1.0);
        }
    }

    /// <summary>清空滤波器与延时线历史状态（通常在切换采样率或重新扫频时调用）。</summary>
    public void Reset()
    {
        // 清空两路滤波与分数延时历史。随机数发生器和已保留的高斯样本跨 Reset 延续，多次模拟仍可能得到不同的有限时长噪声。
        _hpX1L = _hpY1L = _hpX1R = _hpY1R = 0.0;
        _lpX1L = _lpY1L = _lpX1R = _lpY1R = 0.0;
        Array.Clear(_ch2DelayLine);
        _delayWriteIndex = 0;
    }

    /// <summary>使用 Box-Muller 变换生成标准正态分布 N(0,1) 随机数。</summary>
    private double NextGaussian()
    {
        if (_hasSpareGaussian)
        {
            _hasSpareGaussian = false;
            return _spareGaussian;
        }

        // Box–Muller 变换将两个均匀随机数换成两个高斯样本；保留一个给下次调用，避免重复计算同一对随机量。
        double u1 = Math.Max(_rng.NextDouble(), 1e-12);
        double u2 = _rng.NextDouble();
        double mag = Math.Sqrt(-2.0 * Math.Log(u1));
        _spareGaussian = mag * Math.Sin(2.0 * Math.PI * u2);
        _hasSpareGaussian = true;
        return mag * Math.Cos(2.0 * Math.PI * u2);
    }
}
