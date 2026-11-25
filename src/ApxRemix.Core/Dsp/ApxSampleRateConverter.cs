namespace ApxRemix.Core.Dsp;

/// <summary>
/// APx500 10.0.0.17 采样率转换引擎独立复刻（对齐 <c>AudioPrecision.Processing.SampleRateConversion</c> 的
/// <c>PolyphaseRateChanger</c>、<c>HalfBandRateChanger</c> 与 <c>BoxcarDecimator</c>）。
/// </summary>
public static class ApxSampleRateConverter
{
    /// <summary>对齐 <c>BoxcarDecimator.SimpleBoxcarDecimation</c>。</summary>
    public static double[] SimpleBoxcarDecimation(ReadOnlySpan<double> input, int decimationFactor, bool padToFullLength = false)
    {
        if (decimationFactor <= 0) throw new ArgumentOutOfRangeException(nameof(decimationFactor));
        int outCount = input.Length / decimationFactor;
        var output = new double[padToFullLength ? input.Length : outCount];
        double inv = 1.0 / decimationFactor;
        int cursor = 0;
        for (int j = 0; j < outCount; j++)
        {
            double sum = 0.0;
            for (int k = 0; k < decimationFactor; k++) sum += input[cursor++];
            output[j] = sum * inv;
        }
        return output;
    }

    /// <summary>
    /// 高阻带抑制（&gt;120 dB）半带抽取器（2:1 降采样，对齐 APx <c>HalfBandRateChanger</c> 121 dB 阻带规格）。
    /// </summary>
    public static double[] DecimateByTwo(ReadOnlySpan<double> input, double passbandNormalized = 0.45)
    {
        if (input.Length < 2) return [];
        var halfCoeffs = DesignHalfBandHalfFilter(passbandNormalized);
        var rc = new PolyphaseRateChanger(1, 1, 2, halfCoeffs.Length - 1, halfCoeffs);
        return rc.Process([input.ToArray()])[0];
    }

    private static readonly System.Collections.Concurrent.ConcurrentDictionary<(int HalfOddTaps, double Passband), double[]> HalfBandCache = new();

    /// <summary>
    /// 任意采样率抗混叠 Sinc 重采样（97 抽头 Kaiser 窗，halfLen = 48，beta = 13.5，&gt;120 dB 旁瓣抑制；
    /// 降采样时自动按比率缩放截止频率以防止混叠）。
    /// </summary>
    public static double[] Resample(ReadOnlySpan<double> input, double sourceSampleRate, double targetSampleRate)
    {
        if (input.IsEmpty) return [];
        if (!(sourceSampleRate > 0) || !(targetSampleRate > 0))
            throw new ArgumentOutOfRangeException(nameof(sourceSampleRate));
        if (Math.Abs(sourceSampleRate - targetSampleRate) < 1e-9)
            return input.ToArray();

        double ratio = sourceSampleRate / targetSampleRate; // input step per output sample
        int outLen = Math.Max(1, (int)Math.Round(input.Length / ratio));
        double cutoff = ratio > 1.0 ? 0.92 / ratio : 0.92;
        const int halfLen = 48;
        const double beta = 13.5; // >120 dB sidelobe rejection
        double i0Beta = BesselI0(beta);
        var output = new double[outLen];

        for (int i = 0; i < outLen; i++)
        {
            double center = i * ratio;
            int kStart = Math.Max(0, (int)Math.Ceiling(center - halfLen));
            int kEnd = Math.Min(input.Length - 1, (int)Math.Floor(center + halfLen));
            double acc = 0.0, wSum = 0.0;
            for (int k = kStart; k <= kEnd; k++)
            {
                double x = k - center;
                double sinc = Math.Abs(x) < 1e-12 ? cutoff : Math.Sin(Math.PI * cutoff * x) / (Math.PI * x);
                double u = x / (halfLen + 1.0);
                double win = Math.Abs(u) >= 1.0 ? 0.0 : BesselI0(beta * Math.Sqrt(1.0 - u * u)) / i0Beta;
                double tap = sinc * win;
                acc += input[k] * tap;
                wSum += tap;
            }
            output[i] = wSum != 0.0 ? acc / wSum : 0.0;
        }
        return output;
    }

    /// <summary>
    /// 设计等波纹（Parks-McClellan / Remez）线性相位对称半带 FIR 半边系数（中心抽头 0.5，非零奇对称抽头满足 &gt;121 dB 阻带衰减）。
    /// 对 0.45 归一化通带给出 38 个非零奇半抽头（全长 151 抽头，半边长度 76，群延时精确为 75 输入样本，
    /// 与 APx <c>HalfBandRateChanger.FilterPassband0P45Atten122Db</c> 一致）。
    /// </summary>
    public static double[] DesignHalfBandHalfFilter(double passbandFraction = 0.45)
    {
        double fpFrac = Math.Clamp(passbandFraction, 0.125, 0.475);
        double df = 0.5 - fpFrac;
        // Bellanger 等波纹半带阶数估计：122 dB 对应 df=0.05 -> M=38 (151 taps, Delay=75),
        // df=0.25 -> M=7 (27 taps, Delay=13), df=0.375 -> M=4 (15 taps, Delay=7)。
        int m = Math.Abs(fpFrac - 0.45) < 1e-6 ? 38
            : Math.Abs(fpFrac - 0.25) < 1e-6 ? 7
            : Math.Abs(fpFrac - 0.125) < 1e-6 ? 4
            : Math.Max(4, (int)Math.Ceiling(1.9 / df));

        var cached = HalfBandCache.GetOrAdd((m, Math.Round(fpFrac, 6)), key => SolveEquirippleHalfBand(key.HalfOddTaps, key.Passband));
        return (double[])cached.Clone();
    }

    private static double[] SolveEquirippleHalfBand(int m, double passbandFraction)
    {
        // Half-band frequency response: H(w) = 0.5 + sum_{k=1..m} a_k * cos((2k-1)*w),
        // where a_k = 2 * h[2k-1]. Passband edge is f_p = passbandFraction * (Fs/2) / 2 -> omegaP = pi * passbandFraction.
        double omegaP = Math.PI * passbandFraction;
        int extCount = m + 1;
        var ext = new double[extCount];
        // Chebyshev initial extremal grid on [0, omegaP]
        for (int j = 0; j < extCount; j++)
            ext[j] = 0.5 * omegaP * (1.0 - Math.Cos(Math.PI * j / m));

        int gridSize = Math.Max(512, m * 64);
        var gridW = new double[gridSize];
        for (int g = 0; g < gridSize; g++)
            gridW[g] = omegaP * g / (gridSize - 1);

        var a = new double[m];
        var mat = new double[extCount, extCount];
        var rhs = new double[extCount];
        var err = new double[gridSize];

        for (int iter = 0; iter < 40; iter++)
        {
            for (int row = 0; row < extCount; row++)
            {
                double w = ext[row];
                for (int k = 0; k < m; k++)
                    mat[row, k] = Math.Cos((2 * k + 1) * w);
                mat[row, m] = (row & 1) == 0 ? 1.0 : -1.0;
                rhs[row] = 0.5;
            }

            var sol = SolveLinearSystem(mat, rhs);
            for (int k = 0; k < m; k++) a[k] = sol[k];
            double delta = Math.Abs(sol[m]);

            for (int g = 0; g < gridSize; g++)
            {
                double w = gridW[g];
                double val = 0.0;
                for (int k = 0; k < m; k++) val += a[k] * Math.Cos((2 * k + 1) * w);
                err[g] = 0.5 - val;
            }

            // Collect local extrema (including endpoints 0 and gridSize-1)
            var candidates = new List<int>(extCount + 4) { 0 };
            for (int g = 1; g < gridSize - 1; g++)
            {
                if ((err[g] >= err[g - 1] && err[g] >= err[g + 1] && err[g] > 0.0) ||
                    (err[g] <= err[g - 1] && err[g] <= err[g + 1] && err[g] < 0.0))
                {
                    candidates.Add(g);
                }
            }
            candidates.Add(gridSize - 1);

            // Enforce alternating signs by keeping strongest extremum in each run of same sign
            var alt = new List<int>(extCount + 2);
            foreach (int idx in candidates)
            {
                if (alt.Count == 0) { alt.Add(idx); continue; }
                int last = alt[^1];
                if (Math.Sign(err[idx]) == Math.Sign(err[last]))
                {
                    if (Math.Abs(err[idx]) > Math.Abs(err[last])) alt[^1] = idx;
                }
                else
                {
                    alt.Add(idx);
                }
            }

            // Trim to extCount by dropping smallest endpoint if extra extrema appeared
            while (alt.Count > extCount)
            {
                if (Math.Abs(err[alt[0]]) < Math.Abs(err[alt[^1]])) alt.RemoveAt(0);
                else alt.RemoveAt(alt.Count - 1);
            }

            if (alt.Count != extCount) break;

            double maxErr = 0.0;
            for (int j = 0; j < extCount; j++)
            {
                double eAbs = Math.Abs(err[alt[j]]);
                if (eAbs > maxErr) maxErr = eAbs;
            }

            // Refine each interior extremum quadratically on grid
            bool changed = false;
            for (int j = 0; j < extCount; j++)
            {
                int gi = alt[j];
                double newW = gridW[gi];
                if (gi > 0 && gi < gridSize - 1)
                {
                    double e0 = err[gi - 1], e1 = err[gi], e2 = err[gi + 1];
                    double denom = e0 - 2.0 * e1 + e2;
                    if (Math.Abs(denom) > 1e-24)
                    {
                        double shift = Math.Clamp(0.5 * (e0 - e2) / denom, -0.5, 0.5);
                        newW = gridW[gi] + shift * (omegaP / (gridSize - 1));
                    }
                }
                if (Math.Abs(newW - ext[j]) > 1e-12) changed = true;
                ext[j] = newW;
            }

            if (!changed || (delta > 0.0 && (maxErr - delta) / delta < 1e-9))
                break;
        }

        // Convert odd cosine weights a_k = 2 * h[2k-1] into half-filter [h[-(2m-1)], ..., h[-1], h[0]=0.5]
        int halfOrder = 2 * m - 1;
        var half = new double[halfOrder + 1];
        half[halfOrder] = 0.5;
        for (int k = 1; k <= m; k++)
        {
            int dist = 2 * k - 1;
            half[halfOrder - dist] = 0.5 * a[k - 1];
        }
        return half;
    }

    private static double[] SolveLinearSystem(double[,] a, double[] b)
    {
        int n = b.Length;
        var m = (double[,])a.Clone();
        var rhs = (double[])b.Clone();

        for (int col = 0; col < n; col++)
        {
            int pivot = col;
            double maxVal = Math.Abs(m[col, col]);
            for (int r = col + 1; r < n; r++)
            {
                double v = Math.Abs(m[r, col]);
                if (v > maxVal) { maxVal = v; pivot = r; }
            }
            if (pivot != col)
            {
                for (int c = col; c < n; c++) (m[col, c], m[pivot, c]) = (m[pivot, c], m[col, c]);
                (rhs[col], rhs[pivot]) = (rhs[pivot], rhs[col]);
            }

            double diag = m[col, col];
            if (Math.Abs(diag) < 1e-30) continue;
            for (int r = col + 1; r < n; r++)
            {
                double factor = m[r, col] / diag;
                for (int c = col + 1; c < n; c++) m[r, c] -= factor * m[col, c];
                rhs[r] -= factor * rhs[col];
            }
        }

        var x = new double[n];
        for (int r = n - 1; r >= 0; r--)
        {
            double sum = rhs[r];
            for (int c = r + 1; c < n; c++) sum -= m[r, c] * x[c];
            x[r] = Math.Abs(m[r, r]) > 1e-30 ? sum / m[r, r] : 0.0;
        }
        return x;
    }

    private static double BesselI0(double x)
    {
        double sum = 1.0, term = 1.0, half = 0.5 * x;
        for (int k = 1; k <= 30; k++)
        {
            term *= (half / k) * (half / k);
            sum += term;
            if (term < 1e-16 * sum) break;
        }
        return sum;
    }
}

/// <summary>对齐 APx <c>PolyphaseRateChanger</c>（对称半边 FIR 多相有理重采样器，支持任意 up/dn 比率与流式多通道状态）。</summary>
public sealed class PolyphaseRateChanger
{
    private readonly int _channels;
    private readonly int _up;
    private readonly int _down;
    private readonly int _phaseTapLen;
    private readonly double[]? _mirroredFilter;
    private double[][] _delayLine = [];
    private int _writePos;
    private int _phase;
    private int _accum;

    public double RateChangeFactor => (double)_up / _down;
    public double Delay { get; }

    public PolyphaseRateChanger(int numChannels)
    {
        _channels = Math.Max(1, numChannels);
        _up = 1;
        _down = 1;
        Delay = 0.0;
    }

    public PolyphaseRateChanger(int numChannels, int up, int down, double delay, double[] halfFilter)
    {
        if (numChannels < 1 || up < 1 || down < 1 || halfFilter.Length == 0)
            throw new ArgumentOutOfRangeException(nameof(halfFilter));
        _channels = numChannels;
        _up = up;
        _down = down;
        Delay = delay;
        int n = halfFilter.Length;
        _mirroredFilter = new double[4 * n - 1];
        Array.Copy(halfFilter, 0, _mirroredFilter, 2 * n - 1, n);
        Array.Copy(halfFilter, 0, _mirroredFilter, 3 * n - 1, n - 1);
        Array.Reverse(_mirroredFilter, 3 * n - 1, n - 1);
        Array.Copy(_mirroredFilter, 2 * n, _mirroredFilter, 0, 2 * n - 1);
        _phaseTapLen = (2 * n) / _up;
        Reset();
    }

    public void Reset()
    {
        if (_mirroredFilter is null) return;
        _delayLine = new double[_channels][];
        for (int c = 0; c < _channels; c++) _delayLine[c] = new double[_phaseTapLen];
        _writePos = 0;
        _phase = 0;
        _accum = 0;
    }

    public double[][] Process(double[][] input)
    {
        if (_mirroredFilter is null) return input;
        int chCount = Math.Min(_channels, input.Length);
        int inLen = input[0].Length;
        int outLen = (_accum + inLen * _up) / _down;
        var output = new double[chCount][];
        for (int c = 0; c < chCount; c++) output[c] = new double[outLen];

        int outIdx = 0;
        for (int n = 0; n < inLen; n++)
        {
            for (int c = 0; c < chCount; c++)
                _delayLine[c][_writePos] = input[c][n];
            _writePos = (_writePos + 1) % _phaseTapLen;
            _accum += _up;
            while (_accum >= _down)
            {
                _phase = (_phase + _up - _down) % _up;
                if (_phase < 0) _phase += _up;
                int fBase = (_phaseTapLen - _writePos) * _up + _phase - 1;
                for (int c = 0; c < chCount; c++)
                {
                    double acc = 0.0;
                    var dl = _delayLine[c];
                    for (int t = 0; t < _phaseTapLen; t++)
                    {
                        int fi = fBase + t * _up;
                        if ((uint)fi < (uint)_mirroredFilter.Length)
                            acc += _mirroredFilter[fi] * dl[t];
                    }
                    output[c][outIdx] = acc;
                }
                outIdx++;
                _accum -= _down;
            }
        }
        return output;
    }
}
