// 多个分析器都要把时间序列变为复数频谱。此处集中定义变换方向、窗增益和单边幅值刻度；查某条谱线为何是这个 dBFS 值，可从 ComputeSingleSidedSpectrum 跟进去。
using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// 高精度音频专用 FFT 引擎（基-2 与混合基数 FFT + 窗函数相干增益/等效噪声带宽校准）。
/// <para>
/// 设计目标：零外部数值库依赖、64 位双精度浮点运算（保证 &gt; 140 dB 动态范围，避免 32 位浮点底噪抬升），
/// 并严格遵循音频测量仪器（如 APx500）的单边幅值谱标定规范。
/// </para>
/// </summary>
public static class FftEngine
{
    /// <summary>
    /// 窗函数元数据：包含窗系数数组、相干增益（Coherent Gain）以及等效噪声带宽系数（ENBW）。
    /// </summary>
    /// <param name="Coefficients">长度为 N 的时域窗系数。</param>
    /// <param name="CoherentGain">相干增益 CG = (1/N) * Σw[n]，用于校正单频正弦峰值的准确幅度。</param>
    /// <param name="NoisePowerGain">功率增益 S2 = (1/N) * Σw[n]^2，用于校正宽带噪声/能量积分。</param>
    /// <param name="EnbwBins">等效噪声带宽（以 FFT bin 为单位）= S2 / CG^2。</param>
    public sealed record WindowData(
        double[] Coefficients,
        double CoherentGain,
        double NoisePowerGain,
        double EnbwBins);

    // 为长度 N 的记录生成窗值和三种标定量，分别用于校正正弦幅度、宽带功率和等效噪声带宽。
    /// <summary>
    /// 生成指定长度与类型的时域窗函数，并预计算相干增益与功率增益。
    /// </summary>
    /// <param name="size">窗长度 N（通常为 2 的整数次幂）。</param>
    /// <param name="windowType">窗函数类型。</param>
    /// <returns>包含窗系数与标定系数的 <see cref="WindowData"/>。</returns>
    public static WindowData CreateWindow(int size, FftWindowType windowType)
    {
        if (size <= 0)
            throw new ArgumentOutOfRangeException(nameof(size), "窗长度必须大于 0。");

        bool dolph = windowType is FftWindowType.Dolph150 or FftWindowType.Dolph200 or FftWindowType.Dolph250;
        // The small-length Dolph fallback is also used by standalone callers.
        // 优先取当前计算路径支持的窗。返回系数副本，使用方修改窗值不会污染缓存。
        if (size > 2 && Enum.IsDefined(windowType))
        {
            lock (CachedWindows)
            {
                var cacheKey = (windowType, SequenceBinEstimator.DesktopWorksheet);
                if (CachedWindows.TryGetValue(cacheKey, out var data) && data.Coefficients.Length == size)
                    return data with { Coefficients = (double[])data.Coefficients.Clone() };
                var coefficients = SequenceBinEstimator.RawWindow(size, windowType);
                // 没有直接系数时，用余弦级数生成 Hann 等窗，选择 Dolph 窗则由目标旁瓣衰减计算。
                if (coefficients == null && windowType is FftWindowType.Hann or FftWindowType.BlackmanHarris or FftWindowType.BlackmanHarris4)
                    coefficients = SequenceCosineWindow.Generate(size, windowType);
                if (coefficients == null && dolph)
                {
                    int attenuation = windowType == FftWindowType.Dolph150 ? 150 : windowType == FftWindowType.Dolph200 ? 200 : 250;
                    coefficients = DolphWindow(size, attenuation);
                }
                if (coefficients != null)
                {
                    // 频格中心的正弦同相相加，幅度缩放等于窗均值。独立噪声按功率相加，缩放取窗平方的均值。
                    double mean = coefficients.Average();
                    double power = coefficients.Average(x => x * x);
                    CachedWindows[cacheKey] = data = new(coefficients, mean, power, power / (mean * mean));
                    return data with { Coefficients = (double[])coefficients.Clone() };
                }
            }
        }
        var coeffs = new double[size];
        double sum = 0.0;
        double sumSq = 0.0;

        // 原版从 −π 开始生成 N 个相位点，右端 π 不包含在内。
        double twoPiOverN = 2.0 * Math.PI / size;

        // 按 n/N 生成周期窗。各阶余弦的权重改变主瓣宽度和旁瓣高度，影响邻近强弱音的分辨。
        for (int n = 0; n < size; n++)
        {
            double x = n * twoPiOverN - Math.PI;
            double w = windowType switch
            {
                FftWindowType.Rectangular or FftWindowType.NoneMoveToBinCenter => 1.0,

                FftWindowType.Hann => 0.5 + Math.Cos(x) * 0.5,

                // APx has separate three-term and four-term Blackman-Harris windows.
                FftWindowType.BlackmanHarris =>
                    0.42 + Math.Cos(x) * 0.5 + Math.Cos(2.0 * x) * 0.08,

                FftWindowType.BlackmanHarris4 =>
                    0.35875
                    + 0.48829 * Math.Cos(x)
                    + 0.14128 * Math.Cos(2.0 * x)
                    + 0.01168 * Math.Cos(3.0 * x),

                FftWindowType.FlatTop => EvaluateApFlatTop(Math.Cos(x)),

                FftWindowType.Equiripple => EvaluateApEquiripple(Math.Cos(x)),

                _ => 1.0
            };

            coeffs[n] = w;
            sum += w;
            sumSq += w * w;
        }

        // 相干幅度校正后，每格白噪声的等效带宽为窗均方除以均值平方，单位为频格。
        double cg = sum / size;
        double s2 = sumSq / size;
        double enbw = s2 / (cg * cg);

        return new WindowData(coeffs, cg, s2, enbw);
    }

    private static readonly Dictionary<(FftWindowType, bool), WindowData> CachedWindows = new();

    // Evaluate the Chebyshev polynomial on a uniform frequency grid, centre the
    // time response and split its shared endpoint between the two ends.
    // 由指定旁瓣衰减（dB）构造切比雪夫多项式的频谱，再逆变换得到 Dolph 窗的时域权重。
    private static double[] DolphWindow(int size, int attenuation)
    {
        if (size <= 2) return Enumerable.Repeat(1.0, size).ToArray();
        const int gridLimit = 312000;
        // 特别长的窗先在有限网格上求解，再对内部权重线性插值，单独修正两端。
        if (size > gridLimit)
        {
            var grid = DolphWindow(gridLimit, attenuation);
            var expanded = new double[size];
            expanded[0] = expanded[^1] = grid[0] * size / gridLimit;
            for (int i = 1; i < size - 1; i++)
            {
                double at = ((gridLimit - 3.0) * i + size - gridLimit) / (size - 3.0);
                int low = (int)at;
                expanded[i] = at == low ? grid[low] : (at - low) * grid[low + 1] + (low + 1 - at) * grid[low];
            }
            return expanded;
        }
        int order = size - 1;
        // 反双曲余弦将衰减幅度换算成 β，x≥1 时这个实数对数表达式成立。
        static double InverseCosh(double x) => Math.Log(x + Math.Sqrt(x * x - 1.0));
        double beta = Math.Cosh(InverseCosh(Math.Pow(10, attenuation / 20.0)) / order);
        int lastHyperbolic = (int)Math.Floor(order / Math.PI * Math.Acos(1 / beta));
        double step = Math.PI / order;
        var spectrum = new Complex[order];
        // 切比雪夫多项式在单位区间内可由余弦计算，超过 1 时用双曲余弦；镜像填谱保证时域系数为实数。
        for (int k = 0; k <= order / 2; k++)
        {
            double x = beta * Math.Cos(step * k);
            double polynomial = k <= lastHyperbolic ? Math.Cosh(order * InverseCosh(x)) : Math.Cos(order * Math.Acos(x));
            spectrum[k] = (k % 2 == 0 ? 1 : -1) * polynomial;
            if (k > 0) spectrum[order - k] = spectrum[k];
        }
        // 将周期表示的第一项平分给两个端点，得到完整的对称窗。
        SequenceTransform.Inverse(spectrum, normalize: false, stableRoots: true);
        var result = new double[size];
        for (int i = 0; i < order; i++) result[i] = spectrum[i].Real;
        result[0] *= .5;
        result[^1] = result[0];
        return result;
    }

    // Verified against WindowDescriptor.GenerateWindow in APx500 10.0.0.17.
    // 用 cos(相位) 的嵌套多项式求平顶窗。频域主瓣顶部平缓，音调偏离格心时峰值估计受影响较小。
    private static double EvaluateApFlatTop(double c) =>
        -0.05828331947022 + c * (0.15550277475633 + c * (0.49819630174675 +
        c * (0.34465939386184 + c * 0.05990373024506)));

    // 同样以余弦为自变量，换用等波纹窗的系数以获得不同主瓣和旁瓣形状。
    internal static double EvaluateApEquiripple(double c) =>
        0.06372625603133489 + c * (0.23940452419651895 + c * (0.35000203221155923 +
        c * (0.24817008904762164 + c * (0.08582735035413477 +
        c * (0.01242535720360149 + c * 0.0004443909552289012)))));

    // 通用单边谱入口默认先去窗加权直流。返回显示幅值和用于相位、功率计算的复数谱。
    /// <summary>
    /// 对实数输入信号执行加窗与单边幅度谱计算。
    /// <para>
    /// 幅度标定约定：峰值振幅为 1.0（即 0 dBFS Peak）的纯正弦波，在单边谱中的对应峰值读数为 0.0 dBFS。
    /// 这样与信号发生器的 LevelDbfs（0 dBFS Peak）一一对应，读数直观。
    /// </para>
    /// </summary>
    /// <param name="samples">时域采样数组，长度须 &gt;= <paramref name="fftSize"/>（取尾部最新样本）。</param>
    /// <param name="fftSize">FFT 点数，支持 2 的整数次幂与 Bench 固定长度。</param>
    /// <param name="windowType">窗函数类型。</param>
    /// <param name="removeDcOffset">是否在加窗前扣除直流分量（默认 true，避免 DC 泄漏抬高超低频频谱）。</param>
    /// <returns>
    /// 返回三元组：(单边线性峰值幅度数组, 单边 dBFS 数组, 完整复数 FFT 频谱前 N/2+1 项, 窗元数据)。
    /// </returns>
    public static (double[] LinearPeak, double[] MagnitudeDbfs, Complex[] HalfSpectrum, WindowData Window)
        ComputeSingleSidedSpectrum(
            ReadOnlySpan<double> samples,
            int fftSize,
            FftWindowType windowType,
            bool removeDcOffset = true) => ComputeSpectrum(samples, fftSize, windowType, removeDcOffset, stableRoots: false);

    // Bench 谱采用稳定旋转因子计算路径，避免在较长蝶形组内反复相乘引入相位漂移。
    internal static (double[] LinearPeak, double[] MagnitudeDbfs, Complex[] HalfSpectrum, WindowData Window)
        ComputeBenchSpectrum(ReadOnlySpan<double> samples, int fftSize, FftWindowType windowType) =>
            ComputeSpectrum(samples, fftSize, windowType, removeDcOffset: true, stableRoots: true);

    // 只取记录末尾最近 N 点。不足时在末端补零，标定仍使用整个窗的系数。
    private static (double[] LinearPeak, double[] MagnitudeDbfs, Complex[] HalfSpectrum, WindowData Window)
        ComputeSpectrum(ReadOnlySpan<double> samples, int fftSize, FftWindowType windowType, bool removeDcOffset, bool stableRoots)
    {
        if (!IsPowerOfTwo(fftSize) && !IsBenchLength(fftSize))
            throw new ArgumentException("Unsupported FFT record length.", nameof(fftSize));

        int start = Math.Max(0, samples.Length - fftSize);
        int count = Math.Min(samples.Length, fftSize);

        var window = CreateWindow(fftSize, windowType);
        // Subtract the window-weighted DC component. An arithmetic mean of a
        // noncoherent sine creates a false DC lobe when the window is applied.
        // FFT 的第零格等于送进去的所有样本之和。若要求加窗后第零格为零，就令 Σw(x−dc)=0。
        // 这里得到 dc=Σwx/Σw，说明窗边缘贡献较小的样本为什么也应该在求均值时占较小的权重。
        double dc = 0.0;
        // 以和后续加窗相同的权重求均值，减掉它后各点加窗求和趋于零，因而直流格受到抑制。
        if (removeDcOffset && count > 0)
        {
            double weightSum = 0.0;
            for (int i = 0; i < count; i++)
            {
                dc += samples[start + i] * window.Coefficients[i];
                weightSum += window.Coefficients[i];
            }
            dc /= Math.Max(weightSum, 1e-18);
        }

        // 2. 应用时域窗函数并填充复数缓冲
        var buffer = new Complex[fftSize];
        for (int i = 0; i < count; i++)
        {
            double val = (samples[start + i] - dc) * window.Coefficients[i];
            buffer[i] = new Complex(val, 0.0);
        }

        // 3. 原位执行基-2 Cooley-Tukey FFT
        ForwardInPlace(buffer, stableRoots);

        // 4. 构造单边频谱（0 .. N/2）
        // 对于实信号，非 DC 与非 Nyquist 频点在负频率处有对称共轭能量，因此单边峰值需要乘以 2 / (N * CG)
        int halfLen = fftSize / 2 + 1;
        var linearPeak = new double[halfLen];
        var magnitudeDbfs = new double[halfLen];
        var halfSpectrum = new Complex[halfLen];

        // 实信号正负频率成共轭对。内部格幅度加倍合并两侧，直流和偶数长度的奈奎斯特格各自只出现一次。
        double scaleInterior = 2.0 / (fftSize * window.CoherentGain);
        double scaleEdge = 1.0 / (fftSize * window.CoherentGain);

        for (int k = 0; k < halfLen; k++)
        {
            // 原始复数系数另存，显示用的正弦峰值才除以 N 和窗相干增益，以 20log10 得 dBFS。
            halfSpectrum[k] = buffer[k];
            double scale = (k == 0 || k * 2 == fftSize) ? scaleEdge : scaleInterior;
            double amp = buffer[k].Magnitude * scale;
            linearPeak[k] = amp;
            // 防止 log10(0) 出现 -Infinity，将数值底限钳位在 -180 dBFS
            magnitudeDbfs[k] = amp > 1e-9 ? 20.0 * Math.Log10(amp) : -180.0;
        }

        return (linearPeak, magnitudeDbfs, halfSpectrum, window);
    }

    /// <summary>Fixed Bench FFT lengths in samples.</summary>
    public static readonly IReadOnlyList<int> BenchLengths = Array.AsReadOnly(new[]
    {
        256, 512, 1024, 2048, 4096, 8192, 12000, 16384, 24000, 32768, 48000,
        65536, 96000, 131072, 192000, 262144, 312000, 524288, 624000, 1048576, 1248000
    });

    // 原地对复数样本做未归一化的正向 DFT，调用后数组中存放频率分量。
    public static void ForwardInPlace(Complex[] data) => ForwardInPlace(data, stableRoots: false);

    // 2 的幂走基二算法，表内其余长度走混合基数，其他受支持长度交给通用变换。
    internal static void ForwardInPlace(Complex[] data, bool stableRoots)
    {
        if (IsPowerOfTwo(data.Length))
        {
            // 稳定模式预制单位根，普通模式递推旋转因子。精确算术下两种途径表示同一相位步进。
            if (stableRoots)
                RadixTwo(data, CreateRadixTwoRoots(data.Length), data.Length);
            else
                RadixTwo(data);
            return;
        }
        if (BenchLengths.Contains(data.Length)) MixedRadix(data, stableRoots);
        else if (data.Length > 0 && data.Length <= 1248001) SequenceTransform.Forward(data);
        else throw new ArgumentOutOfRangeException(nameof(data), "Unsupported FFT length.");
    }

    // AP's exact non-binary record lengths factor into 2, 3, 5 and 13. Gather each
    // strided subtransform into contiguous storage, then combine it in place.
    // 为指定的复合记录长度递归拆分短 DFT，各级共用全长网格上的旋转因子。
    private static void MixedRadix(Complex[] input, bool stableRoots)
    {
        int total = input.Length;
        var work = new Complex[total];
        var roots = new Complex[total];
        for (int i = 0; i < total; i++)
        {
            var (sine, cosine) = Math.SinCos(-2 * Math.PI * i / total);
            roots[i] = new Complex(cosine, sine);
        }
        Transform(0, 1, 0, total);
        work.CopyTo(input, 0);

        // source、stride 决定取哪组交错样本，dest 指定输出区。缩成 2 的幂后由基二 FFT 接手。
        void Transform(int source, int stride, int dest, int n)
        {
            if (IsPowerOfTwo(n))
            {
                for (int i = 0; i < n; i++) work[dest + i] = input[source + i * stride];
                if (stableRoots)
                    RadixTwo(work.AsSpan(dest, n), roots, total);
                else
                    RadixTwo(work.AsSpan(dest, n));
                return;
            }
            // 按下标除以 radix 的余数分组，各组长度 m。先分别变换，再按频率顺序合并。
            int radix = n % 3 == 0 ? 3 : n % 5 == 0 ? 5 : 13;
            int m = n / radix;
            for (int r = 0; r < radix; r++) Transform(source + r * stride, stride * radix, dest + r * m, m);
            Span<Complex> partial = stackalloc Complex[13];
            for (int k = 0; k < m; k++)
            {
                // 第一次复乘补回子序列起点的时间偏移，第二层小型 DFT 得到原长变换的各格。
                for (int r = 0; r < radix; r++)
                    partial[r] = work[dest + r * m + k] * roots[(long)r * k * (total / n) % total];
                for (int j = 0; j < radix; j++)
                {
                    Complex sum = partial[0];
                    for (int r = 1; r < radix; r++) sum += partial[r] * roots[(j * r % radix) * (total / radix)];
                    work[dest + k + j * m] = sum;
                }
            }
        }
    }

    // 负指数方向的单位根按对称性生成。轴上特殊点直接取精确值，避免本应为零的分量出现舍入误差。
    private static Complex[] CreateRadixTwoRoots(int n)
    {
        // The same root is reused at every applicable stage. Computing it from its
        // index prevents a large signal from leaking into weak bins due to drift.
        var roots = new Complex[n / 2];
        if (roots.Length == 0) return roots;
        roots[0] = Complex.One;
        if (n >= 4) roots[n / 4] = -Complex.ImaginaryOne;
        for (int k = 1; k < n / 4; k++)
        {
            var (sine, cosine) = Math.SinCos(-2.0 * Math.PI * k / n);
            roots[k] = new Complex(cosine, sine);
            roots[n / 2 - k] = new Complex(-cosine, sine);
        }
        return roots;
    }

    // 数据长度须为 2 的幂。位逆序重排之后，蝶形每一级把局部 DFT 的长度加倍。
    private static void RadixTwo(Span<Complex> data, ReadOnlySpan<Complex> roots = default, int rootOrder = 0)
    {
        int n = data.Length;
        // 第一步：位反转置换（Bit-Reversal Permutation）
        int j = 0;
        // 位逆序的计数从高位开始进位，i<j 才交换以免同一对位置重复操作。
        for (int i = 1; i < n; i++)
        {
            int bit = n >> 1;
            while ((j & bit) != 0)
            {
                j ^= bit;
                bit >>= 1;
            }
            j ^= bit;

            if (i < j)
            {
                (data[i], data[j]) = (data[j], data[i]);
            }
        }

        // 第二步：逐级蝶形运算（Butterfly Operations）
        for (int len = 2; len <= n; len <<= 1)
        {
            int half = len >> 1;
            // 有预计算表时，全表阶数除以本级长度给出所需查表步长。
            if (!roots.IsEmpty)
            {
                int rootStep = rootOrder / len;
                for (int i = 0; i < n; i += len)
                {
                    for (int k = 0; k < half; k++)
                        Butterfly(data, i + k, half, roots[k * rootStep]);
                }
                continue;
            }

            // 没有预计算表时，每组从 1 开始乘 e^(−j2π/len)，逐项走过本级所需相角。
            double angle = -2.0 * Math.PI / len;
            var wLen = new Complex(Math.Cos(angle), Math.Sin(angle));

            for (int i = 0; i < n; i += len)
            {
                Complex w = Complex.One;
                for (int k = 0; k < half; k++)
                {
                    Butterfly(data, i + k, half, w);
                    w *= wLen;
                }
            }
        }
    }

    // 偶序分量和旋转后的奇序分量相加、相减便得到相隔半段的两个输出，两者共享一次复乘。
    private static void Butterfly(Span<Complex> data, int index, int half, Complex root)
    {
        Complex u = data[index];
        Complex v = data[index + half] * root;
        data[index] = u + v;
        data[index + half] = u - v;
    }

    // 用主峰及两侧的对数幅度估计格间峰顶位置，要求附近主要由一个平滑主瓣占据。
    /// <summary>
    /// 使用三点抛物线插值（对数幅度域）精化频谱峰值位置，实现亚 bin 级别的频率测量精度。
    /// </summary>
    /// <param name="linearPeak">单边线性幅度谱。</param>
    /// <param name="peakBin">粗搜索到的离散峰值索引。</param>
    /// <param name="sampleRate">采样率（Hz）。</param>
    /// <param name="fftSize">FFT 总点数。</param>
    /// <returns>插值精化后的频率（Hz）与精化后的线性峰值幅度。</returns>
    public static (double RefinedFreqHz, double RefinedPeak) RefinePeakParabolic(
        double[] linearPeak,
        int peakBin,
        int sampleRate,
        int fftSize)
    {
        double binWidth = (double)sampleRate / fftSize;

        // 靠边处直接报告原频格，对被谱边缘截断的峰形不作外推。
        if (peakBin <= 1 || peakBin >= linearPeak.Length - 2)
        {
            return (peakBin * binWidth, linearPeak[peakBin]);
        }

        // 在对数域进行二次曲线拟合，对高斯型/Hann 主瓣的频率插值误差极小
        double alpha = Math.Log(Math.Max(linearPeak[peakBin - 1], 1e-12));
        double beta = Math.Log(Math.Max(linearPeak[peakBin], 1e-12));
        double gamma = Math.Log(Math.Max(linearPeak[peakBin + 1], 1e-12));

        // 三个对数幅度确定一条抛物线。顶点最多偏移半格，横坐标乘 Fs/N 得 Hz，纵坐标取指数恢复幅度。
        double denom = alpha - 2.0 * beta + gamma;
        double delta = Math.Abs(denom) > 1e-12 ? 0.5 * (alpha - gamma) / denom : 0.0;
        delta = Math.Clamp(delta, -0.5, 0.5);

        double refinedFreq = (peakBin + delta) * binWidth;
        double refinedLogPeak = beta - 0.25 * (alpha - gamma) * delta;
        double refinedPeak = Math.Exp(refinedLogPeak);

        return (refinedFreq, refinedPeak);
    }

    // 逆变换的缩放与本类未归一化的正向变换配对，使正变换后再逆变换能恢复原尺度。
    /// <summary>Normalized inverse transform, using the same sign convention as ForwardInPlace.</summary>
    public static void InverseInPlace(Complex[] data) => InverseInPlace(data, stableRoots: false);

    // 共轭翻转指数的符号。先共轭再正变换，结果共轭并除以 N，即可复用同一 FFT 完成逆变换。
    internal static void InverseInPlace(Complex[] data, bool stableRoots)
    {
        for (int i = 0; i < data.Length; i++) data[i] = Complex.Conjugate(data[i]);
        ForwardInPlace(data, stableRoots);
        for (int i = 0; i < data.Length; i++) data[i] = Complex.Conjugate(data[i]) / data.Length;
    }

    // 只检查可接受的长度范围。是否使用专门的混合基数路径还取决于长度是否在列表中。
    public static bool IsBenchLength(int n) => n is >= 256 and <= 1248000;

    // 2 的幂在二进制中恰有一个置位，与它减一后的值按位 AND 必为零。
    public static bool IsPowerOfTwo(int n) => n > 0 && (n & (n - 1)) == 0;
}
