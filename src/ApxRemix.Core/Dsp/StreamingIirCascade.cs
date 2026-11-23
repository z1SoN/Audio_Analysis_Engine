// 把高阶滤波器拆成若干二阶节后，每节只需记住自己的两个状态。主带宽、带通与陷波都能共用这个逐采样执行器，设计系数与推动实时状态可以分别检查。
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// 连续实时 IIR 二阶节（Biquad）级联滤波器。
/// <para>
/// 采用标准双线性变换（Bilinear Transform）与 Direct Form II Transposed 状态机实现，
/// 在任意音频回调分块边界保持内部状态连续无跳变，涵盖：
/// - Butterworth HP4 / LP8 与通用 2 阶 AC 耦合高通；
/// - 6 阶 Butterworth 带通（1/3、1/6、1/12、1/24 倍频程）及传统二阶 Q=10 带通；
/// - IEC 61672-1 A/C 计权、ITU-R BS.468-4 CCIR-1k/2k 与 50µs/75µs 一阶去加重。
/// </para>
/// </summary>
public sealed class StreamingIirCascade
{
    public readonly record struct BiquadCoeffs(double B0, double B1, double B2, double A1, double A2);

    private struct SectionState
    {
        public double Z1;
        public double Z2;
    }

    private readonly BiquadCoeffs[] _sections;
    private readonly SectionState[] _states;
    private readonly double _gain;
    private readonly bool _useApxStateOrder;

    public bool IsBypass => _sections.Length == 0 && Math.Abs(_gain - 1.0) < 1e-12;
    public int SectionCount => _sections.Length;

    // 为一串二阶节建立独立延迟状态。普通入口支持各节独立的 B0 系数。
    public StreamingIirCascade(BiquadCoeffs[] sections, double gain = 1.0)
        : this(sections, gain, useApxStateOrder: false) { }

    // 同一套系数可以供多实例共享，但每节的两个延迟量属于当前实例，否则两声道会相互串入历史。
    internal StreamingIirCascade(BiquadCoeffs[] sections, double gain, bool useApxStateOrder)
    {
        _sections = sections;
        _states = new SectionState[sections.Length];
        _gain = gain;
        _useApxStateOrder = useApxStateOrder;
    }

    // 空级联加单位增益就是直通，不需要逐点修改输入。
    public static StreamingIirCascade Bypass() => new(Array.Empty<BiquadCoeffs>(), 1.0);

    // 清空延迟相当于假定开始前没有输入，系数和带宽仍由原设计决定。
    public void Reset() => Array.Clear(_states);

    // 两个级联的节数一致时逐节复制历史，供调用方在小幅调谐时选择保留暂态。
    internal void CopyHistoryFrom(StreamingIirCascade prior)
    {
        if (prior._states.Length == _states.Length) prior._states.CopyTo(_states, 0);
    }

    // 每个输入先乘总增益，再逐节通过。输出覆盖当前采样点，延迟状态保存到下一点和下一包。
    /// <summary>原地连续处理一个采样块，保留全部滤波器内部状态。</summary>
    public void ProcessInPlace(Span<double> samples)
    {
        if (IsBypass) return;
        // 专用分支接受每节 B0=1 的设计。普通递推则显式乘各节的 B0。
        if (_useApxStateOrder)
        {
            ProcessApxSections(samples);
            return;
        }
        for (int i = 0; i < samples.Length; i++)
        {
            double x = samples[i] * _gain;
            for (int s = 0; s < _sections.Length; s++)
            {
                var c = _sections[s];
                ref var st = ref _states[s];
                // 先看一节。即使这一刻 x=0，只要上一刻留下的 Z1 还在，输出仍可以非零。
                // 算完 y 后更新两个状态供下一采样点使用，再把本节 y 送入下一节。这是两个不同方向的传递。
                // 当前直接项加第一延迟得到输出，输出又进入两条反馈项。第二延迟保存更早样本对下一点的影响。
                double y = c.B0 * x + st.Z1;
                st.Z1 = c.B1 * x - c.A1 * y + st.Z2;
                st.Z2 = c.B2 * x - c.A2 * y;
                x = y;
            }
            samples[i] = x;
        }
    }

    // 专用递推按节扫描整包，每节以保存的延迟起步，结束后再写回。总增益只在第一级施加一次。
    private void ProcessApxSections(Span<double> samples)
    {
        // APxRT weights each block section by section, adding the old Z2 before B1*x.
        for (int s = 0; s < _sections.Length; s++)
        {
            var c = _sections[s];
            double gain = s == 0 ? _gain : 1;
            var state = _states[s];
            for (int i = 0; i < samples.Length; i++)
            {
                double x = samples[i] * gain;
                double y = x + state.Z1;
                state.Z1 = -c.A1 * y + state.Z2 + c.B1 * x;
                state.Z2 = -c.A2 * y + c.B2 * x;
                samples[i] = y;
            }
            _states[s] = state;
        }
    }

    // 先将输入复制到输出工作区，再处理相同长度。这样上游还可把原波形送给其他分支。
    /// <summary>从输入复制到输出并连续滤波（不修改输入缓冲区）。</summary>
    public void Process(ReadOnlySpan<double> input, Span<double> output)
    {
        if (output.Length < input.Length)
            throw new ArgumentException("Output buffer shorter than input.");
        input.CopyTo(output);
        ProcessInPlace(output[..input.Length]);
    }

    // 以单位延迟 z⁻¹=exp(−j2πf/Fs) 代入各节。各节复响应相乘，最终 |H|² 给出稳态功率增益。
    /// <summary>计算当前数字级联滤波器在指定频率处的精确功率响应 |H(e^jω)|²。</summary>
    public double EvaluatePowerGain(double frequencyHz, int sampleRate)
    {
        if (sampleRate <= 0 || frequencyHz < 0.0 || frequencyHz >= sampleRate * 0.5)
            return 0.0;
        double w = 2.0 * Math.PI * frequencyHz / sampleRate;
        var z1 = Complex.FromPolarCoordinates(1.0, -w);
        var z2 = z1 * z1;
        Complex h = new(_gain, 0.0);
        for (int s = 0; s < _sections.Length; s++)
        {
            var c = _sections[s];
            Complex num = c.B0 + c.B1 * z1 + c.B2 * z2;
            Complex den = 1.0 + c.A1 * z1 + c.A2 * z2;
            h *= num / den;
        }
        return h.Real * h.Real + h.Imaginary * h.Imaginary;
    }

    // 将偶数阶 Butterworth 高通拆成共轭极点二阶节。输入参数不适合该设计时走直通。
    /// <summary>设计偶数阶双线性 Butterworth 高通滤波器（如 4 阶 HP4）。</summary>
    public static StreamingIirCascade CreateButterworthHighPass(int order, double cutoffHz, int sampleRate)
    {
        if (cutoffHz <= 0.0 || sampleRate <= 0 || order < 2 || (order & 1) != 0)
            return Bypass();

        double fc = Math.Clamp(cutoffHz, 1.0, sampleRate * 0.495);
        // 先用 tan(πfc/Fs) 做频率预畸变，使双线性变换后的截止点仍在指定 Hz。
        double k = Math.Tan(Math.PI * fc / sampleRate);
        double k2 = k * k;
        int sectionCount = order / 2;
        var sections = new BiquadCoeffs[sectionCount];

        for (int m = 0; m < sectionCount; m++)
        {
            // 极点角度决定各节 Q。分子按 1、−2、1 排列，直流处的双零点使恒定输入逐渐消失。
            double invQ = 2.0 * Math.Cos((2 * m + 1) * Math.PI / (2 * order));
            double norm = 1.0 / (1.0 + invQ * k + k2);
            double b0 = norm;
            double b1 = -2.0 * norm;
            double b2 = norm;
            double a1 = 2.0 * (k2 - 1.0) * norm;
            double a2 = (1.0 - invQ * k + k2) * norm;
            sections[m] = new BiquadCoeffs(b0, b1, b2, a1, a2);
        }

        return new StreamingIirCascade(sections, 1.0);
    }

    // 将偶数阶低通拆成二阶节，低频通过，高频衰减。每个新实例从零历史开始。
    /// <summary>设计偶数阶双线性 Butterworth 低通滤波器（如 8 阶 LP8）。</summary>
    public static StreamingIirCascade CreateButterworthLowPass(int order, double cutoffHz, int sampleRate)
    {
        if (cutoffHz <= 0.0 || sampleRate <= 0 || order < 2 || (order & 1) != 0)
            return Bypass();

        double fc = Math.Clamp(cutoffHz, 5.0, sampleRate * 0.495);
        double k = Math.Tan(Math.PI * fc / sampleRate);
        double k2 = k * k;
        int sectionCount = order / 2;
        var sections = new BiquadCoeffs[sectionCount];

        for (int m = 0; m < sectionCount; m++)
        {
            // 分母沿用 Butterworth 极点分布，低通分子按 1、2、1 排列，在奈奎斯特点 z=−1 相消。
            double invQ = 2.0 * Math.Cos((2 * m + 1) * Math.PI / (2 * order));
            double norm = 1.0 / (1.0 + invQ * k + k2);
            double b0 = k2 * norm;
            double b1 = 2.0 * b0;
            double b2 = b0;
            double a1 = 2.0 * (k2 - 1.0) * norm;
            double a2 = (1.0 - invQ * k + k2) * norm;
            sections[m] = new BiquadCoeffs(b0, b1, b2, a1, a2);
        }

        return new StreamingIirCascade(sections, 1.0);
    }

    // 以中心 Hz 和倍频程分数指定通带。三分之一倍频程的低边缘是 f0·2^(−1/6)。
    /// <summary>
    /// 设计 6 阶双线性 Butterworth 带通滤波器（3 阶模拟低通原型 LP→BP + 双线性变换，3 个二阶节级联）。
    /// 支持 1/3、1/6、1/12、1/24 倍频程选择性，中心频率处通带增益精确为 1.0。
    /// </summary>
    public static StreamingIirCascade CreateButterworthBandpass(double centerHz, int octaveFraction, int sampleRate)
    {
        if (centerHz < 5.0 || sampleRate <= 0 || centerHz >= sampleRate * 0.5)
            return Bypass();

        int frac = Math.Clamp(octaveFraction, 1, 48);
        double f0 = Math.Clamp(centerHz, 5.0, sampleRate * 0.495);
        double f1ToF0 = Math.Pow(2.0, -1.0 / (2.0 * frac));

        // 预畸变轴上的两边缘乘积为 w0²，所以上边缘是 w0²/w1。减去下边缘即得带宽。
        double w0 = Math.Tan(Math.PI * f0 / sampleRate);
        double w1 = Math.Tan(Math.PI * (f0 * f1ToF0) / sampleRate);
        double bw = (w0 * w0 / w1) - w1;
        if (bw <= 1e-12) return Bypass();

        // 原型极点 p0 = -1 对应的实二阶节：H0(s) = (bw * s) / (s^2 + bw*s + w0^2)
        // 将分子正比于 s 的模拟二阶节映为数字系数，在直流和奈奎斯特各留下一个零点。
        static BiquadCoeffs FromNormalizedBandpassS(double bwNum, double c1, double c0)
        {
            double d = 1.0 + c1 + c0;
            return new BiquadCoeffs(
                B0: bwNum / d,
                B1: 0.0,
                B2: -bwNum / d,
                A1: 2.0 * (c0 - 1.0) / d,
                A2: (1.0 - c1 + c0) / d);
        }

        // 原型的实极点经带通变换贡献一节，复极点对产生其余两节，共组成六阶带通。
        var s0 = FromNormalizedBandpassS(bw, bw, w0 * w0);

        // 原型共轭极点 p1 = -0.5 + j*sqrt(3)/2 -> 解 s^2 - bw*p1*s + w0^2 = 0 得两对共轭极点
        Complex bp = new(-0.5 * bw, 0.5 * Math.Sqrt(3.0) * bw);
        Complex disc = Complex.Sqrt(bp * bp - 4.0 * w0 * w0);
        Complex rootA = 0.5 * (bp + disc);
        Complex rootB = 0.5 * (bp - disc);

        var s1 = FromNormalizedBandpassS(bw, -2.0 * rootA.Real, rootA.Real * rootA.Real + rootA.Imaginary * rootA.Imaginary);
        var s2 = FromNormalizedBandpassS(bw, -2.0 * rootB.Real, rootB.Real * rootB.Real + rootB.Imaginary * rootB.Imaginary);

        return new StreamingIirCascade([s0, s1, s2], 1.0);
    }

    // 单节带通的 Q 控制选择性。中心 Hz 给相位步长，同一中心下 Q 越大，α 越小，通带越窄。
    /// <summary>设计传统二阶常数 Q 带通滤波器（中心频率处增益为 1.0）。</summary>
    public static StreamingIirCascade CreateSecondOrderBandpassQ(double centerHz, double q, int sampleRate)
    {
        if (centerHz <= 5.0 || sampleRate <= 0 || centerHz >= sampleRate * 0.49)
            return Bypass();
        double w0 = 2.0 * Math.PI * Math.Clamp(centerHz, 5.0, sampleRate * 0.485) / sampleRate;
        double alpha = Math.Sin(w0) / (2.0 * Math.Max(0.5, q));
        double norm = 1.0 / (1.0 + alpha);
        return new StreamingIirCascade(
            [new BiquadCoeffs(alpha * norm, 0.0, -alpha * norm, -2.0 * Math.Cos(w0) * norm, (1.0 - alpha) * norm)],
            1.0);
    }

    // 将计权选项转成可处理连续音频的级联。频响描述稳态正弦的增益，有限记录还包含起始暂态。
    /// <summary>
    /// 设计连续时域标准计权与去加重 IIR 滤波器级联（双线性变换并在标称参考频率精确归一化为 0 dB）：
    /// - A/C 计权：IEC 61672-1 精确模拟极点，1 kHz 归一化；
    /// - CCIR-1k / CCIR-2k：ITU-R BS.468-4 精确 6 极点 1 零点模拟分解，分别在 1 kHz / 2 kHz 归一化；
    /// - De-emphasis 50 µs / 75 µs：一阶双线性低通。
    /// </summary>
    public static StreamingIirCascade CreateWeightingFilter(AnalyzerWeighting weighting, int sampleRate)
    {
        if (weighting == AnalyzerWeighting.Flat || sampleRate <= 0)
            return Bypass();

        double kScale = 2.0 * sampleRate;

        // 模拟实极点双节：(s^2 或 1) / ((s + w1)(s + w2))
        // 将两个实模拟极点配成二次分母。Hz 乘 2π 转成 rad/s，与双线性代换单位一致。
        BiquadCoeffs BilinearTwoRealPoles(double f1Hz, double f2Hz, int zerosAtOrigin)
        {
            double w1 = 2.0 * Math.PI * f1Hz;
            double w2 = 2.0 * Math.PI * f2Hz;
            double c2 = 1.0;
            double c1 = w1 + w2;
            double c0 = w1 * w2;
            return BilinearQuad(c2, c1, c0, zerosAtOrigin, kScale);
        }

        // 模拟共轭复极点对：1 / (s^2 - 2*Re*s + |p|^2)，p = 2π * (reHz ± j*imHz)
        // 共轭极点的和与积均为实数，可以只用实系数描述该节的幅值和相位响应。
        BiquadCoeffs BilinearComplexPair(double reHz, double imHz, int zerosAtOrigin)
        {
            double re = 2.0 * Math.PI * reHz;
            double im = 2.0 * Math.PI * imHz;
            double c1 = -2.0 * re;
            double c0 = re * re + im * im;
            return BilinearQuad(1.0, c1, c0, zerosAtOrigin, kScale);
        }

        // Nyquist 精确增益匹配的 1 阶实极点（重数 1 或 2），彻底消除 12.194 kHz 与去加重的高频双线性压缩误差
        // 用 exp(−2πfc/Fs) 匹配极点每采样的衰减，调整零点匹配奈奎斯特幅度，再将直流增益校正到 1。
        BiquadCoeffs MatchedRealPole(double fcHz, bool doubled)
        {
            double p = Math.Exp(-2.0 * Math.PI * fcHz / sampleRate);
            double wNyq = (0.5 * sampleRate) / fcHz;
            double gNyq = 1.0 / Math.Sqrt(1.0 + wNyq * wNyq);
            double rho = gNyq * (1.0 + p) / Math.Max(1.0 - p, 1e-12);
            double z0 = (rho - 1.0) / (rho + 1.0);
            double g = (1.0 - p) / Math.Max(1.0 - z0, 1e-12);
            // 若两个相同极点串联，则分子分母分别平方；单独的一阶去加重仅需一对极点、零点。
            return doubled
                ? new BiquadCoeffs(g * g, -2.0 * g * g * z0, g * g * z0 * z0, -2.0 * p, p * p)
                : new BiquadCoeffs(g, -g * z0, 0.0, -p, 0.0);
        }

        BiquadCoeffs[] sections;
        double refFreqHz;

        switch (weighting)
        {
            // A 计权低频有四个原点零点，各段合成后以 1 kHz 作为单位增益参考。
            case AnalyzerWeighting.AWeighting:
                sections =
                [
                    BilinearTwoRealPoles(20.598997, 20.598997, zerosAtOrigin: 2),
                    BilinearTwoRealPoles(107.65265, 737.86223, zerosAtOrigin: 2),
                    MatchedRealPole(12194.217, doubled: true)
                ];
                refFreqHz = 1000.0;
                break;

            // B 计权在低频双零点以外再加入一个转折点为 158.48932 Hz 的一阶高通。
            case AnalyzerWeighting.BWeighting:
            {
                double k=2*Math.PI*158.48932/kScale;
                sections=[BilinearTwoRealPoles(20.598997,20.598997,2),
                    new BiquadCoeffs(1/(1+k),-1/(1+k),0,(k-1)/(k+1),0),MatchedRealPole(12194.217,true)];
                refFreqHz=1000;
                break;
            }
            // 复合计权先设计 A 计权，再串入所选时间常数的去加重级，稳态增益为两者之积。
            case AnalyzerWeighting.Deemphasis50usA:
            case AnalyzerWeighting.Deemphasis75usA:
            {
                var a=CreateWeightingFilter(AnalyzerWeighting.AWeighting,sampleRate);
                double tau=weighting==AnalyzerWeighting.Deemphasis50usA ? 50e-6 : 75e-6;
                return new StreamingIirCascade([..a._sections,MatchedRealPole(1/(2*Math.PI*tau),false)],a._gain);
            }
            // C 计权保留低频双零点和高频滚降，其低频衰减较此处 A 计权平缓。
            case AnalyzerWeighting.CWeighting:
                sections =
                [
                    BilinearTwoRealPoles(20.598997, 20.598997, zerosAtOrigin: 2),
                    MatchedRealPole(12194.217, doubled: true)
                ];
                refFreqHz = 1000.0;
                break;

            // 两种 CCIR 选择的极点、零点相同，差别在于 1 kHz 或 2 kHz 处定为单位增益。
            case AnalyzerWeighting.Ccir1k:
            case AnalyzerWeighting.Ccir2k:
                // ITU-R BS.468-4 标准 6 阶模拟传递函数分解（1 个原点零点 + 2 个实极点 + 2 对预畸变共轭极点）
                sections =
                [
                    BilinearTwoRealPoles(4122.7020661347487, 9975.0631239303111, zerosAtOrigin: 1),
                    BilinearComplexPair(-3758.5291630107204, 5790.0423366089581, zerosAtOrigin: 0),
                    BilinearComplexPair(-2983.15993789067, 9940.8426457095611, zerosAtOrigin: 0)
                ];
                refFreqHz = weighting == AnalyzerWeighting.Ccir2k ? 2000.0 : 1000.0;
                break;

            // 单独去加重由 τ 换算 fc=1/(2πτ)，以直流为单位增益直接返回。
            case AnalyzerWeighting.Deemphasis50us:
            case AnalyzerWeighting.Deemphasis75us:
            {
                double tau = weighting == AnalyzerWeighting.Deemphasis50us ? 50e-6 : 75e-6;
                double fc = 1.0 / (2.0 * Math.PI * tau);
                return new StreamingIirCascade([MatchedRealPole(fc, doubled: false)], 1.0);
            }

            default:
                return Bypass();
        }

        // 在参考频率上计算整条链的功率增益。总幅度修正取其平方根的倒数。
        var unnorm = new StreamingIirCascade(sections, 1.0);
        double refPower = unnorm.EvaluatePowerGain(refFreqHz, sampleRate);
        double normGain = refPower > 1e-300 ? 1.0 / Math.Sqrt(refPower) : 1.0;
        return new StreamingIirCascade(sections, normGain);
    }

    // 令 s=k(1−z⁻¹)/(1+z⁻¹) 代入模拟二次分母，通分并按延迟次数收集同类项。
    private static BiquadCoeffs BilinearQuad(double c2, double c1, double c0, int zerosAtOrigin, double k)
    {
        double k2 = k * k;
        double d0 = c2 * k2 + c1 * k + c0;
        double d1 = 2.0 * (c0 - c2 * k2);
        double d2 = c2 * k2 - c1 * k + c0;

        double n0, n1, n2;
        // 模拟分子的 s²、s、常数给出三种数字零点组合。所有系数最后同除 d0，使当前输出项系数为 1。
        if (zerosAtOrigin == 2)
        {
            // s^2 -> k^2 * (1 - 2z^-1 + z^-2)
            n0 = k2; n1 = -2.0 * k2; n2 = k2;
        }
        else if (zerosAtOrigin == 1)
        {
            // sqrt(c0) * s -> sqrt(c0) * k * (1 - z^-2)，保持每节增益量级 O(1)
            double s = Math.Sqrt(Math.Max(c0, 1.0)) * k;
            n0 = s; n1 = 0.0; n2 = -s;
        }
        else
        {
            // c0 -> c0 * (1 + 2z^-1 + z^-2)，保持 DC 增益为 1.0
            n0 = c0; n1 = 2.0 * c0; n2 = c0;
        }

        return new BiquadCoeffs(n0 / d0, n1 / d0, n2 / d0, d1 / d0, d2 / d0);
    }
}
