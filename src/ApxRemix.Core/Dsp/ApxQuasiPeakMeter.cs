using System.Numerics;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// APx500 10.0.0.17 ITU-R BS.468-4 / CCIR-1k 准峰值检波器（对齐 <c>AudioPrecision.Processing.Measurements.QPeak.CcirQpeak</c>、
/// <c>Ccir1KFilter</c> 与 <c>QPeakDetect</c>）。
/// 链路：ITU-R BS.468-4 6 极点冲激不变 CCIR-1k 滤波器（6300 Hz 处归一化为 10^0.61 = +12.2 dB）→
/// 全波整流非对称攻释检波（τ_attack = 1.5 ms, τ_decay = 300 ms）→ 一阶双线性低通（ω₀ = 9.718542606875246 rad/s）→ × 1.0366843917102098 / √2。
/// </summary>
public sealed class ApxQuasiPeakMeter
{
    private const double AttackTimeSeconds = 0.0015;
    private const double DecayTimeSeconds = 0.3;
    private const double PostLowpassOmega = 9.718542606875246;
    private const double CalibrationFactor = 1.0366843917102098;
    private static readonly double InvSqrt2 = 1.0 / Math.Sqrt(2.0);
    private static readonly double Target6k3Gain = Math.Pow(10.0, 0.61);

    // ITU-R BS.468-4 精确 6 极点部分分式展开（对齐 AudioPrecision.Om.dll Ccir1KFilter / DynamicImpulseInvariantIIR）。
    private static readonly Complex[] ComplexPoles =
    [
        new(-18743.746620223184, 62460.156713716038),
        new(-23615.535228758155, 36379.908985967159)
    ];
    private static readonly Complex[] ComplexResidues =
    [
        new(3338.4079226404187, 10134.05018818214),
        new(724.80102159237958, -30053.912966699088)
    ];
    private static readonly double[] RealPoles = [-62675.170140072834, -25903.701161964338];
    private static readonly double[] RealResidues = [26430.576910883181, -34556.994799348751];

    private int _sampleRate;
    private readonly int _channels;
    private readonly bool _applyCcir1kWeighting;

    // 冲激不变并联系数与每声道状态
    private readonly Complex[] _zp = new Complex[2];
    private readonly double[] _rp = new double[2];
    private double _ccirScale = 1.0;
    private readonly Complex[,] _cState;
    private readonly double[,] _rState;

    private readonly double[] _detectorState;
    private readonly double[] _lpX1;
    private readonly double[] _lpY1;
    private double _lpB0, _lpB1, _lpA1;
    private readonly double[] _peaks;

    public ApxQuasiPeakMeter(int sampleRate, int channels = 1, bool applyCcir1kWeighting = true)
    {
        if (sampleRate <= 0) throw new ArgumentOutOfRangeException(nameof(sampleRate));
        if (channels < 1) throw new ArgumentOutOfRangeException(nameof(channels));
        _sampleRate = sampleRate;
        _channels = channels;
        _applyCcir1kWeighting = applyCcir1kWeighting;
        _cState = new Complex[channels, 2];
        _rState = new double[channels, 2];
        _detectorState = new double[channels];
        _lpX1 = new double[channels];
        _lpY1 = new double[channels];
        _peaks = new double[channels];
        RebuildFilters(sampleRate);
    }

    public double[] Peaks => _peaks;

    public void Reset()
    {
        Array.Clear(_cState);
        Array.Clear(_rState);
        Array.Clear(_detectorState);
        Array.Clear(_lpX1);
        Array.Clear(_lpY1);
        Array.Clear(_peaks);
        RebuildFilters(_sampleRate);
    }

    /// <summary>推进一帧多声道采样，更新每声道准峰值读数（线性幅度，与 APx <c>CcirQpeak.Peaks</c> 同尺度）。</summary>
    public double[] ComputePeaks(double[][] inputSamples, int sampleRate)
    {
        if (inputSamples.Length == 0 || inputSamples[0].Length == 0) return _peaks;
        if (sampleRate != _sampleRate)
        {
            _sampleRate = sampleRate;
            Reset();
        }
        int chCount = Math.Min(_channels, inputSamples.Length);
        double alphaAttack = 1.0 / (_sampleRate * AttackTimeSeconds);
        double alphaDecay = 1.0 / (_sampleRate * DecayTimeSeconds);

        for (int ch = 0; ch < chCount; ch++)
        {
            var src = inputSamples[ch];
            if (src.Length == 0) continue;

            Complex c0 = _cState[ch, 0], c1 = _cState[ch, 1];
            double r0 = _rState[ch, 0], r1 = _rState[ch, 1];
            double env = _detectorState[ch];
            double x1 = _lpX1[ch];
            double y1 = _lpY1[ch];
            double last = 0.0;

            for (int i = 0; i < src.Length; i++)
            {
                double v = src[i];
                if (_applyCcir1kWeighting)
                {
                    c0 = c0 * _zp[0] + v;
                    c1 = c1 * _zp[1] + v;
                    r0 = r0 * _rp[0] + v;
                    r1 = r1 * _rp[1] + v;
                    double yw = 2.0 * (ComplexResidues[0].Real * c0.Real - ComplexResidues[0].Imaginary * c0.Imaginary)
                        + 2.0 * (ComplexResidues[1].Real * c1.Real - ComplexResidues[1].Imaginary * c1.Imaginary)
                        + RealResidues[0] * r0 + RealResidues[1] * r1;
                    v = yw * _ccirScale;
                }

                double mag = Math.Abs(v);
                if (mag > env)
                    env += (mag - env) * alphaAttack;
                else
                    env -= env * alphaDecay;

                double y = _lpB0 * env + _lpB1 * x1 - _lpA1 * y1;
                x1 = env;
                y1 = y;
                last = y;
            }

            _cState[ch, 0] = c0;
            _cState[ch, 1] = c1;
            _rState[ch, 0] = r0;
            _rState[ch, 1] = r1;
            _detectorState[ch] = env;
            _lpX1[ch] = x1;
            _lpY1[ch] = y1;
            _peaks[ch] = last * CalibrationFactor * InvSqrt2;
        }
        return _peaks;
    }

    public static double MeasureChannel(ReadOnlySpan<double> samples, int sampleRate, bool applyCcir1kWeighting = true)
    {
        var meter = new ApxQuasiPeakMeter(sampleRate, 1, applyCcir1kWeighting);
        return meter.ComputePeaks([samples.ToArray()], sampleRate)[0];
    }

    private void RebuildFilters(int sampleRate)
    {
        double twoFs = 2.0 * sampleRate;
        double denom = twoFs + PostLowpassOmega;
        _lpB0 = PostLowpassOmega / denom;
        _lpB1 = _lpB0;
        _lpA1 = (PostLowpassOmega - twoFs) / denom;

        double dt = 1.0 / sampleRate;
        for (int i = 0; i < 2; i++)
        {
            _zp[i] = Complex.Exp(ComplexPoles[i] * dt);
            _rp[i] = Math.Exp(RealPoles[i] * dt);
        }

        // 在 6300 Hz 处归一化为 10^0.61 (+12.2 dB)，对齐 Ccir1KFilter / DynamicImpulseInvariantIIR。
        var zInv = Complex.FromPolarCoordinates(1.0, -Math.Tau * 6300.0 * dt);
        Complex h = Complex.Zero;
        for (int i = 0; i < 2; i++)
        {
            h += ComplexResidues[i] / (Complex.One - _zp[i] * zInv);
            h += Complex.Conjugate(ComplexResidues[i]) / (Complex.One - Complex.Conjugate(_zp[i]) * zInv);
            h += RealResidues[i] / (Complex.One - _rp[i] * zInv);
        }
        double mag6k3 = h.Magnitude;
        _ccirScale = mag6k3 > 0 ? Target6k3Gain / mag6k3 : 1.0;
    }
}
