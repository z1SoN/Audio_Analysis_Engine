using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APxRT 10.0.0.17 连续时域 IMD 仪表独立复刻（APxRT.c 1000e660 DfdMeter / 1000e720 DimMeter / 1000eaf0 SmpteMeter）。
/// 在主通路（带宽+计权）之后连续推进，跨回调与报告窗保留全部滤波与检波历史。
/// </summary>
public sealed class ApxRealtimeImdMeter
{
    private readonly int _sampleRate;
    private double _readingsPerSecond;
    private ImdMeasurementStandard _mode = ImdMeasurementStandard.Auto;
    private double _dfdLowPassHz = 2000.0;

    // 共享检波器（对齐 APxRT ImdMeter 基类：0x10 为交流耦合 RMS，0x1a0 为直通 RMS）
    private ApxRmsDetector _rmsA1, _rmsA2;
    private ApxRmsDetector _rmsB1, _rmsB2;
    private ApxAcCouplingFilter _acA1, _acA2;

    // SMPTE 滤波器（2 kHz 高通 + 平方 + 750 Hz 低通）
    private StreamingIirCascade _smpteHp1 = StreamingIirCascade.Bypass(), _smpteHp2 = StreamingIirCascade.Bypass();
    private StreamingIirCascade _smpteLp1 = StreamingIirCascade.Bypass(), _smpteLp2 = StreamingIirCascade.Bypass();

    // DFD / CCIF 状态（8 阶低通提取差频 + 交流耦合 + 过零测频 + 1/12 倍频程 3 阶 0.01 dB 切比雪夫带通）
    private StreamingIirCascade _dfdLp1 = StreamingIirCascade.Bypass(), _dfdLp2 = StreamingIirCascade.Bypass();
    private ApxAcCouplingFilter _dfdAc1, _dfdAc2;
    private ApxFrequencyPhaseDetector _dfdFreq1 = new(), _dfdFreq2 = new();
    private ApxRmsDetector _dfdFreqRms1, _dfdFreqRms2;
    private double _dfdPrevPeak1, _dfdPrevPeak2;
    private StreamingIirCascade _dfdBp1 = StreamingIirCascade.Bypass(), _dfdBp2 = StreamingIirCascade.Bypass();
    private double _dfdBpCenter1, _dfdBpCenter2;

    // DIM 滤波器（2400 Hz 16 阶低通 + 400 Hz 5 阶椭圆高通）
    private StreamingIirCascade _dimLp1 = StreamingIirCascade.Bypass(), _dimLp2 = StreamingIirCascade.Bypass();
    private StreamingIirCascade _dimHp1 = StreamingIirCascade.Bypass(), _dimHp2 = StreamingIirCascade.Bypass();

    private double[] _workL = [], _workR = [];
    private double[] _sqL = [], _sqR = [];

    // APxRT SmpteMeter (1000eaf0) 数字原型系数（fc = Fs/4）
    private static readonly (double B1, double B2, double A1, double A2)[] SmpteHpProto =
    [
        (-1.99331752659859, 1.0, 0.688850989946946, 0.190721121403311),
        (-1.95091729281665, 1.0, 0.257481200048254, 0.463191190851903),
        (-1.90985030819994, 1.0, -0.108697895554353, 0.807937157359801)
    ];

    private static readonly (double B1, double B2, double A1, double A2)[] SmpteLpProto =
    [
        (1.98098787393243, 1.0, -0.928554213855523, 0.270735129582513),
        (1.85328653368552, 1.0, -0.569752275054727, 0.449007668994776),
        (1.69252533887938, 1.0, -0.173814692029095, 0.676270132349913),
        (1.59085019399582, 1.0, 0.0599214125979227, 0.890721481143697)
    ];

    // APxRT DimMeter (1000e720) 16 阶低通数字原型系数（fc = Fs/4）
    private static readonly (double B1, double B2, double A1, double A2)[] DimLpProto =
    [
        (1.95258683076766, 1.0, -1.00945142551128, 0.280864576865032),
        (1.63595428429476, 1.0, -0.846122718024604, 0.381566926832856),
        (1.22118719923429, 1.0, -0.607098730430981, 0.52964219768173),
        (0.872003428490094, 1.0, -0.379704555541827, 0.672217082222038),
        (0.627917692319203, 1.0, -0.2038515670332, 0.785775155853133),
        (0.473297592954933, 1.0, -0.0840021918601778, 0.869065217328654),
        (0.38447090286115, 1.0, -0.0115106629628032, 0.929835605354508),
        (0.344273450395347, 1.0, 0.0227283170233415, 0.977784839913871)
    ];

    private const double SmpteProtoEdgeGain = 0.994260073952957;
    private const double DimProtoEdgeGain = 0.99942451938;
    private static readonly double DfdBandpassRatio = Math.Pow(2.0, -1.0 / 24.0); // 1/12 octave (APxRT 10016c38)

    public ApxRealtimeImdMeter(int sampleRate, double readingsPerSecond)
    {
        _sampleRate = Math.Max(8000, sampleRate);
        _readingsPerSecond = Math.Max(0.1, readingsPerSecond);
        _rmsA1 = new(_sampleRate, _readingsPerSecond);
        _rmsA2 = new(_sampleRate, _readingsPerSecond);
        _rmsB1 = new(_sampleRate, _readingsPerSecond);
        _rmsB2 = new(_sampleRate, _readingsPerSecond);
        _acA1 = new(_sampleRate);
        _acA2 = new(_sampleRate);
        _dfdAc1 = new(_sampleRate);
        _dfdAc2 = new(_sampleRate);
        // APxRT 1000f800: ImdMeter::SetSlowRms 仅作用于主 rmsA/rmsB，DFD 内部过零测频的 RMS 始终保持慢速档 (slow=1)
        _dfdFreqRms1 = new(_sampleRate, 20.0);
        _dfdFreqRms2 = new(_sampleRate, 20.0);
    }

    public ImdMeasurementStandard Mode => _mode;

    internal void SetReadingRate(double readingsPerSecond)
    {
        _readingsPerSecond = readingsPerSecond;
        _rmsA1.SetReadingRate(readingsPerSecond);
        _rmsA2.SetReadingRate(readingsPerSecond);
        _rmsB1.SetReadingRate(readingsPerSecond);
        _rmsB2.SetReadingRate(readingsPerSecond);
    }

    public void Configure(ImdMeasurementStandard mode, double dfdLowPassHz = 2000.0)
    {
        double clampedLp = Math.Clamp(dfdLowPassHz > 0 ? dfdLowPassHz : 2000.0, 50.0, _sampleRate * 0.45);

        if (mode == _mode)
        {
            if (mode is ImdMeasurementStandard.Dfd or ImdMeasurementStandard.Ccif
                && Math.Abs(clampedLp - _dfdLowPassHz) > _dfdLowPassHz * 1e-5)
            {
                var lp1 = StreamingIirCascade.CreateButterworthLowPass(8, clampedLp, _sampleRate);
                var lp2 = StreamingIirCascade.CreateButterworthLowPass(8, clampedLp, _sampleRate);
                lp1.CopyHistoryFrom(_dfdLp1);
                lp2.CopyHistoryFrom(_dfdLp2);
                _dfdLp1 = lp1;
                _dfdLp2 = lp2;
                _dfdLowPassHz = clampedLp;
            }
            return;
        }

        _dfdLowPassHz = clampedLp;
        _mode = mode;
        Reset();
        switch (_mode)
        {
            case ImdMeasurementStandard.Smpte:
                _smpteHp1 = CreateFromDigitalPrototype(SmpteHpProto, 2000.0, _sampleRate, SmpteProtoEdgeGain);
                _smpteHp2 = CreateFromDigitalPrototype(SmpteHpProto, 2000.0, _sampleRate, SmpteProtoEdgeGain);
                _smpteLp1 = CreateFromDigitalPrototype(SmpteLpProto, 750.0, _sampleRate, SmpteProtoEdgeGain);
                _smpteLp2 = CreateFromDigitalPrototype(SmpteLpProto, 750.0, _sampleRate, SmpteProtoEdgeGain);
                break;
            case ImdMeasurementStandard.Dfd:
            case ImdMeasurementStandard.Ccif:
                _dfdLp1 = StreamingIirCascade.CreateButterworthLowPass(8, _dfdLowPassHz, _sampleRate);
                _dfdLp2 = StreamingIirCascade.CreateButterworthLowPass(8, _dfdLowPassHz, _sampleRate);
                _dfdBpCenter1 = _dfdBpCenter2 = 0;
                break;
            case ImdMeasurementStandard.DimEmulation:
                _dimLp1 = CreateFromDigitalPrototype(DimLpProto, 2400.0, _sampleRate, DimProtoEdgeGain);
                _dimLp2 = CreateFromDigitalPrototype(DimLpProto, 2400.0, _sampleRate, DimProtoEdgeGain);
                (_dimHp1, _dimHp2) = ApxEllipticLowPass.CreateEllipticHighPassStereo(400.0, _sampleRate);
                break;
        }
    }

    public void Reset()
    {
        _rmsA1 = new(_sampleRate, _readingsPerSecond);
        _rmsA2 = new(_sampleRate, _readingsPerSecond);
        _rmsB1 = new(_sampleRate, _readingsPerSecond);
        _rmsB2 = new(_sampleRate, _readingsPerSecond);
        _acA1 = new(_sampleRate);
        _acA2 = new(_sampleRate);
        _smpteHp1.Reset();
        _smpteHp2.Reset();
        _smpteLp1.Reset();
        _smpteLp2.Reset();
        _dfdLp1.Reset();
        _dfdLp2.Reset();
        _dimLp1.Reset();
        _dimLp2.Reset();
        _dimHp1.Reset();
        _dimHp2.Reset();
        _dfdAc1 = new(_sampleRate);
        _dfdAc2 = new(_sampleRate);
        _dfdFreq1 = new();
        _dfdFreq2 = new();
        _dfdFreqRms1 = new(_sampleRate, 20.0);
        _dfdFreqRms2 = new(_sampleRate, 20.0);
        _dfdPrevPeak1 = _dfdPrevPeak2 = 0;
        _dfdBpCenter1 = _dfdBpCenter2 = 0;
        _dfdBp1 = StreamingIirCascade.Bypass();
        _dfdBp2 = StreamingIirCascade.Bypass();
    }

    public void Process(ReadOnlySpan<double> mainLeft, ReadOnlySpan<double> mainRight)
    {
        if (_mode is not (ImdMeasurementStandard.Smpte or ImdMeasurementStandard.Dfd
            or ImdMeasurementStandard.Ccif or ImdMeasurementStandard.DimEmulation))
            return;

        int n = Math.Min(mainLeft.Length, mainRight.Length);
        if (n <= 0) return;
        EnsureBuffers(n);
        var wL = _workL.AsSpan(0, n);
        var wR = _workR.AsSpan(0, n);

        switch (_mode)
        {
            case ImdMeasurementStandard.Smpte:
                ProcessSmpte(mainLeft[..n], mainRight[..n], wL, wR);
                break;
            case ImdMeasurementStandard.Dfd:
            case ImdMeasurementStandard.Ccif:
                ProcessDfdOrCcif(mainLeft[..n], mainRight[..n], wL, wR);
                break;
            case ImdMeasurementStandard.DimEmulation:
                ProcessDim(mainLeft[..n], mainRight[..n], wL, wR);
                break;
        }
    }

    // APxRT SmpteMeter::Process (1000f5b0): HP(2k) -> rmsB(carrier) -> x^2 -> LP(750) -> AC -> rmsA(mod)
    private void ProcessSmpte(ReadOnlySpan<double> inL, ReadOnlySpan<double> inR, Span<double> wL, Span<double> wR)
    {
        _smpteHp1.Process(inL, wL);
        _smpteHp2.Process(inR, wR);
        _rmsB1.Process(wL);
        _rmsB2.Process(wR);
        for (int i = 0; i < wL.Length; i++)
        {
            wL[i] *= wL[i];
            wR[i] *= wR[i];
        }
        _smpteLp1.ProcessInPlace(wL);
        _smpteLp2.ProcessInPlace(wR);
        _acA1.Process(wL, wL);
        _acA2.Process(wR, wR);
        _rmsA1.Process(wL);
        _rmsA2.Process(wR);
    }

    // APxRT DfdMeter::Process (1000f200): AC+rmsA(total) | x^2 -> LP8(dfdLp) -> freqDetect -> ChebyBP(1/12 oct, f_diff) -> rmsB(d2)
    private void ProcessDfdOrCcif(ReadOnlySpan<double> inL, ReadOnlySpan<double> inR, Span<double> wL, Span<double> wR)
    {
        var sqL = _sqL.AsSpan(0, wL.Length);
        var sqR = _sqR.AsSpan(0, wR.Length);

        _acA1.Process(inL, wL);
        _acA2.Process(inR, wR);
        _rmsA1.Process(wL);
        _rmsA2.Process(wR);

        for (int i = 0; i < wL.Length; i++)
        {
            sqL[i] = wL[i] * wL[i];
            sqR[i] = wR[i] * wR[i];
        }
        _dfdLp1.ProcessInPlace(sqL);
        _dfdLp2.ProcessInPlace(sqR);

        // APxRT 1000e660 (this + 0x480): 专用 AcCoupling 就地处理平方低通支路，保留 wL/wR 中交流耦合后的主信号送入切比雪夫带通 (APxRT.c:8873,8889)
        _dfdAc1.Process(sqL, sqL);
        _dfdAc2.Process(sqR, sqR);
        _dfdFreqRms1.Process(sqL);
        _dfdFreqRms2.Process(sqR);
        _dfdFreq1.ProcessBlock(sqL, _sampleRate, _dfdPrevPeak1);
        _dfdFreq2.ProcessBlock(sqR, _sampleRate, _dfdPrevPeak2);
        var f1 = _dfdFreq1.Read().FrequencyHz;
        var f2 = _dfdFreq2.Read().FrequencyHz;
        _dfdPrevPeak1 = _dfdFreqRms1.Read() * Math.Sqrt(2.0);
        _dfdPrevPeak2 = _dfdFreqRms2.Read() * Math.Sqrt(2.0);

        TuneChebyshevBandpass(ref _dfdBp1, ref _dfdBpCenter1, f1);
        TuneChebyshevBandpass(ref _dfdBp2, ref _dfdBpCenter2, f2);

        _dfdBp1.ProcessInPlace(wL);
        _dfdBp2.ProcessInPlace(wR);
        _rmsB1.Process(wL);
        _rmsB2.Process(wR);
    }

    // APxRT DimMeter::Process (1000f400): AC+rmsA(total) | LP16(2400) -> HP5_Elliptic(400) -> rmsB(dim)
    private void ProcessDim(ReadOnlySpan<double> inL, ReadOnlySpan<double> inR, Span<double> wL, Span<double> wR)
    {
        _acA1.Process(inL, wL);
        _acA2.Process(inR, wR);
        _rmsA1.Process(wL);
        _rmsA2.Process(wR);

        _dimLp1.ProcessInPlace(wL);
        _dimLp2.ProcessInPlace(wR);
        _dimHp1.ProcessInPlace(wL);
        _dimHp2.ProcessInPlace(wR);
        _rmsB1.Process(wL);
        _rmsB2.Process(wR);
    }

    public (double Ch1Ratio, double Ch2Ratio) Read()
    {
        if (_mode is not (ImdMeasurementStandard.Smpte or ImdMeasurementStandard.Dfd
            or ImdMeasurementStandard.Ccif or ImdMeasurementStandard.DimEmulation))
            return (double.NaN, double.NaN);

        double a1 = _rmsA1.Read(), a2 = _rmsA2.Read();
        double b1 = _rmsB1.Read(), b2 = _rmsB2.Read();
        return (_mode switch
        {
            // APxRT 1000f110 SmpteMeter: modRms / (carrierRms^2 * sqrt(2))
            ImdMeasurementStandard.Smpte => SafeRatio(a1, b1 * b1 * Math.Sqrt(2.0)),
            // APxRT 1000eec0 DfdMeter: (d2Rms / (totalRms * sqrt(2))) * (isCcif ? 2 : 1)
            ImdMeasurementStandard.Dfd => SafeRatio(b1, a1 * Math.Sqrt(2.0)),
            ImdMeasurementStandard.Ccif => SafeRatio(b1 * 2.0, a1 * Math.Sqrt(2.0)),
            // APxRT 1000efe0 DimMeter: dimRms / (totalRms / sqrt(33))
            ImdMeasurementStandard.DimEmulation => SafeRatio(b1, a1 / Math.Sqrt(33.0)),
            _ => double.NaN
        }, _mode switch
        {
            ImdMeasurementStandard.Smpte => SafeRatio(a2, b2 * b2 * Math.Sqrt(2.0)),
            ImdMeasurementStandard.Dfd => SafeRatio(b2, a2 * Math.Sqrt(2.0)),
            ImdMeasurementStandard.Ccif => SafeRatio(b2 * 2.0, a2 * Math.Sqrt(2.0)),
            ImdMeasurementStandard.DimEmulation => SafeRatio(b2, a2 / Math.Sqrt(33.0)),
            _ => double.NaN
        });

        static double SafeRatio(double num, double den)
        {
            if (!(den > 0) || !double.IsFinite(num)) return double.NaN;
            double r = num / den;
            return r < 1.0 ? r : double.NaN;
        }
    }

    private void TuneChebyshevBandpass(ref StreamingIirCascade filter, ref double activeCenter, double measuredHz)
    {
        double center = Math.Clamp(measuredHz, 5.0, _sampleRate * 0.495);
        if (Math.Abs(center - activeCenter) <= activeCenter * 1e-4 && !filter.IsBypass)
            return;
        activeCenter = center;
        filter = CreateChebyshevBandpass3(center, DfdBandpassRatio, _sampleRate);
    }

    // APxRT 10005060(param_1 = 1) + 10007380: 3 阶 0.01 dB 切比雪夫原型经 LP->BP 与双线性变换
    private static StreamingIirCascade CreateChebyshevBandpass3(double centerHz, double f1ToF0, int fs)
    {
        double w0 = Math.Tan(Math.PI * centerHz / fs);
        double w1 = Math.Tan(Math.PI * (centerHz * f1ToF0) / fs);
        double bw = w0 * w0 / w1 - w1;
        if (bw <= 1e-12) return StreamingIirCascade.Bypass();

        double ripple = Math.Pow(10.0, 0.01 / 10.0);
        double mu = Math.Asinh(1.0 / Math.Sqrt(ripple - 1.0)) / 3.0;
        double realProto = -Math.Sinh(mu);
        var complexProto = new Complex(-0.5 * Math.Sinh(mu), Math.Cos(Math.PI / 6.0) * Math.Cosh(mu));

        // 对实原型极点 s^2 - bw*p*s + w0^2 = 0
        Complex dReal = Complex.Sqrt(bw * bw * realProto * realProto - 4.0 * w0 * w0);
        Complex r0a = 0.5 * (bw * realProto + dReal);
        Complex r0b = 0.5 * (bw * realProto - dReal);

        Complex bp = bw * complexProto;
        Complex dC = Complex.Sqrt(bp * bp - 4.0 * w0 * w0);
        Complex r1 = 0.5 * (bp + dC);
        Complex r2 = 0.5 * (bp - dC);

        static StreamingIirCascade.BiquadCoeffs MapSection(Complex p1, Complex p2)
        {
            Complex z1 = (Complex.One + p1) / (Complex.One - p1);
            Complex z2 = (Complex.One + p2) / (Complex.One - p2);
            double a1 = -(z1 + z2).Real;
            double a2 = (z1 * z2).Real;
            return new(1.0, 0.0, -1.0, a1, a2);
        }

        var sections = new[]
        {
            MapSection(r0a, r0b),
            MapSection(r1, Complex.Conjugate(r1)),
            MapSection(r2, Complex.Conjugate(r2))
        };

        var unnorm = new StreamingIirCascade(sections, 1.0, useApxStateOrder: true);
        double centerPow = unnorm.EvaluatePowerGain(centerHz, fs);
        double gain = centerPow > 1e-300 ? 1.0 / Math.Sqrt(centerPow) : 1.0;
        return new(sections, gain, useApxStateOrder: true);
    }

    // APxRT IirFromDigProto (1000bad0 / 1000bdd0 / 1000a960): Constantinides 全通数字频率变换
    internal static StreamingIirCascade CreateFromDigitalPrototype(
        (double B1, double B2, double A1, double A2)[] proto,
        double cutoffHz, int sampleRate, double edgeGain)
    {
        double rel = Math.Clamp(2.0 * cutoffHz / sampleRate, 1e-6, 0.99);
        double alpha = Math.Sin((0.5 - rel) * (Math.PI * 0.5)) / Math.Sin((0.5 + rel) * (Math.PI * 0.5));
        double a2 = alpha * alpha;
        var sections = new StreamingIirCascade.BiquadCoeffs[proto.Length];

        for (int i = 0; i < proto.Length; i++)
        {
            var p = proto[i];
            double dNum = 1.0 / (1.0 - alpha * p.B1 + a2 * p.B2);
            double nb1 = (p.B1 * (1.0 + a2) - 2.0 * alpha * (1.0 + p.B2)) * dNum;
            double nb2 = (a2 - alpha * p.B1 + p.B2) * dNum;

            double dDen = 1.0 / (1.0 - alpha * p.A1 + a2 * p.A2);
            double na1 = (p.A1 * (1.0 + a2) - 2.0 * alpha * (1.0 + p.A2)) * dDen;
            double na2 = (a2 - alpha * p.A1 + p.A2) * dDen;

            sections[i] = new(1.0, nb1, nb2, na1, na2);
        }

        double evalHz = Math.Min(rel * sampleRate * 0.5, sampleRate * 0.499999);
        var unnorm = new StreamingIirCascade(sections, 1.0, useApxStateOrder: true);
        double edgePow = unnorm.EvaluatePowerGain(evalHz, sampleRate);
        double gain = edgePow > 1e-300 ? edgeGain / Math.Sqrt(edgePow) : edgeGain;
        return new(sections, gain, useApxStateOrder: true);
    }

    private void EnsureBuffers(int n)
    {
        if (_workL.Length >= n) return;
        _workL = new double[n];
        _workR = new double[n];
        _sqL = new double[n];
        _sqR = new double[n];
    }
}
