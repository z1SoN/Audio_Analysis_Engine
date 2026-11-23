// 学习工具的发生端和采集端要同意每个频点何时开始。Generate 返回激励和生成段，AcquisitionSlices 依同一份时间计划确定录音中的测量窗口，两种边界各有用途。
namespace ApxRemix.Core.Dsp;

public sealed record FastSweepSettings(int SampleRate, double[] FrequenciesHz, double[] Amplitudes,
    double DurationSeconds, double TransitionCycles, double DiscardCycles, double MeasurementCycles,
    double TransitionSeconds, double DiscardSeconds, double MeasurementSeconds);

/// <summary>A constant-frequency interval starts at the discard section and ends after measurement.</summary>
public sealed record FastSweepSlice(double FrequencyHz, int Start, int Count);
public sealed record FastSweepExcitation(double[] Samples, FastSweepSlice[] Slices);

/// <summary>Independent implementation of the APx500 managed FastSweep.Generate section scheduler.</summary>
public static class FastSweepGenerator
{
    private static (double[] transition, double[] discard, double[] measure, double duration, double endingCycle)
        Schedule(FastSweepSettings settings)
    {
        var frequencies = settings.FrequenciesHz;
        var amplitudes = settings.Amplitudes;
        int fs = settings.SampleRate;
        // 每个频点既要有有效频率又要有对应幅度，三段时间必须非负。先验证计划，才可可靠预分配整段刺激。
        if (fs <= 0 || frequencies is not { Length: >= 2 } || amplitudes?.Length != frequencies.Length ||
            frequencies.Any(f => !double.IsFinite(f) || f <= 0 || f >= fs * 0.5) ||
            amplitudes.Any(a => !double.IsFinite(a) || a < 0))
            throw new ArgumentException("Each frequency requires a nonnegative amplitude and must be below Nyquist.");
        double[] times = [settings.DurationSeconds, settings.TransitionCycles, settings.DiscardCycles,
            settings.MeasurementCycles, settings.TransitionSeconds, settings.DiscardSeconds, settings.MeasurementSeconds];
        if (times.Any(t => !double.IsFinite(t) || t < 0))
            throw new ArgumentException("Durations and cycle counts must be finite and nonnegative.");

        var transition = new double[frequencies.Length];
        var discard = new double[frequencies.Length];
        var measure = new double[frequencies.Length];
        // 每个音依次经历切换、丢弃过渡、正式测量。周期要求除以该点 Hz 得到秒，同样的周期数会让低频占用更多时间。
        double minimum = 0, periodSum = 0;
        for (int i = 0; i < frequencies.Length; i++)
        {
            double f = frequencies[i];
            transition[i] = Math.Max(settings.TransitionSeconds, settings.TransitionCycles / f);
            discard[i] = Math.Max(settings.DiscardSeconds, settings.DiscardCycles / f);
            // 若要求至少 10 个周期且至少 10 ms，250 Hz 需要 40 ms，2 kHz 虽然 5 ms 就有 10 周期，
            // 仍要留 10 ms。这里得到每段的最低时长，计划若还有余量，后面才把它分出去。
            measure[i] = Math.Max(settings.MeasurementSeconds, settings.MeasurementCycles / f);
            minimum += measure[i] + transition[i] + discard[i];
            periodSum += 1 / f;
        }
        double endingCycle = frequencies[0] > frequencies[^1] && transition[0] > 0 ? 1 / frequencies[^1] : 0;
        minimum += endingCycle;
        double duration = minimum;
        // 指定总时长大于最低时长时，把余量按频点周期分配到各段。只扩充整段缓冲而不改时间表会使采集窗错位。
        if (settings.DurationSeconds > minimum)
        {
            if (settings.MeasurementCycles == 0)
                throw new ArgumentException("Distributing extra time requires positive measurement cycles.");
            double cycles = (settings.DurationSeconds - minimum) / periodSum;
            double d = settings.DiscardCycles / settings.MeasurementCycles;
            double t = settings.TransitionCycles / settings.MeasurementCycles;
            duration = 0;
            for (int i = 0; i < frequencies.Length; i++)
            {
                double extra = cycles / frequencies[i];
                measure[i] += extra * (1 - d - t);
                transition[i] += extra * t;
                discard[i] += extra * d;
                duration += measure[i] + transition[i] + discard[i];
            }
            duration += endingCycle;
        }
        if (measure.Any(t => t * fs < 1) || duration * fs >= Array.MaxLength)
            throw new ArgumentException("The sweep must have a measurable interval at each frequency and fit in memory.");

        return (transition, discard, measure, duration, endingCycle);
    }

    // 仅返回共享时间表的总秒数。这样容量校验所用时长与随后实际生成的各段一致。
    public static double Duration(FastSweepSettings settings) => Schedule(settings).duration;

    public static FastSweepExcitation Generate(FastSweepSettings settings)
    {
        var (transition, discard, measure, duration, endingCycle) = Schedule(settings);
        var frequencies = settings.FrequenciesHz;
        var amplitudes = settings.Amplitudes;
        int fs = settings.SampleRate;
        var signal = new double[(int)(duration * fs + 0.5)];
        var slices = new FastSweepSlice[frequencies.Length];
        double startSeconds = 0, phase = 0, samplePeriod = 1.0 / fs;
        int cursor = 0;
        void Emit(double seconds, double fromHz, double toHz, double fromAmp, double toAmp)
        {
            if (seconds == 0) return;
            int count = Math.Clamp((int)((seconds - (cursor * samplePeriod - startSeconds)) * fs), 0, signal.Length - cursor);
            // 切换段的瞬时频率按指数变化，由频率积分求相位；两端频率相同时使用固定频率，绕开 ln(1) 分母。
            double ratio = toHz / fromHz;
            double coefficient = ratio == 1 ? 0 : 2 * Math.PI * fromHz * seconds / Math.Log(ratio);
            // 正弦函数需要相位。瞬时频率 f(t) 表示每秒走几圈，所以相位要积累 2π∫f(t)dt。
            // 频率按比例连续变化时，下面的幂函数是这个积分；每段结束把总相位交给下一段，接缝才不会重起一个周期。
            double SectionPhase(double t) => ratio == 1 ? 2 * Math.PI * t * fromHz :
                coefficient * (Math.Pow(ratio, t / seconds) - 1);
            for (int j = 0; j < count; j++)
            {
                double t = (cursor + j) * samplePeriod - startSeconds;
                signal[cursor + j] = Math.Sin(phase + SectionPhase(t)) * Amplitude(fromAmp, toAmp, j, count);
            }
            cursor += count;
            startSeconds += seconds;
            phase += SectionPhase(seconds);
        }

        // Generate 返回的起点在转场结束处，因此生成切片同时包含丢弃段和测量段。采集端另用 AcquisitionSlices 选实际读数窗口。
        for (int i = 0; i < frequencies.Length; i++)
        {
            double f = frequencies[i], amp = amplitudes[i];
            if (i == 0) Emit(transition[0] / 2, f, f, 0, amp);
            else Emit(transition[i], frequencies[i - 1], f, amplitudes[i - 1], amp);
            int start = cursor;
            Emit(discard[i], f, f, amp, amp);
            Emit(measure[i], f, f, amp, amp);
            slices[i] = new(f, start, cursor - start);
        }
        Emit(transition[0] / 2 + endingCycle, frequencies[^1], frequencies[^1], amplitudes[^1], 0);
        return new(signal, slices);
    }

    /// <summary>Acquisition-clock windows. Point origins, when supplied, are absolute discard-start
    /// sample indices including the measured channel delay. A fixed-delay plan assumes synchronous clocks.</summary>
    public static FastSweepSlice[] AcquisitionSlices(FastSweepSettings settings, int acquisitionRate,
        int recordingLength, int channelDelay, bool distortion, int[]? pointOrigins = null)
    {
        var (transition, discard, measure, duration, _) = Schedule(settings);
        var frequencies = settings.FrequenciesHz;
        // 用采集率换算计数。记录边界只钳制返回的起点，调用方还要核对从该起点起是否有足够的完整样本。
        if (acquisitionRate <= 0 || recordingLength < 7 || duration * acquisitionRate >= Array.MaxLength ||
            frequencies.Any(f => f >= acquisitionRate * 0.5) ||
            (pointOrigins != null && pointOrigins.Length != frequencies.Length))
            throw new ArgumentException("Invalid acquisition rate, recording length or point alignment.");
        var result = new FastSweepSlice[frequencies.Length];
        double elapsedFraction = 0, lengthFraction = 0;
        int cursor = 0;
        for (int i = 0; i < result.Length; i++)
        {
            double ramp = i == 0 ? transition[i] / 2 : transition[i];
            double hz = frequencies[i];
            // 稳态窗从过渡和丢弃都结束后才起算。时间用秒描述，数组用录音采样点索引，乘采集率才能相加。
            // channelDelay 同样以录音采样点计，录音晚到 137 点，未触边的窗口就向右移 137 点。
            long start = (long)cursor + channelDelay + (int)Math.Ceiling((ramp + discard[i]) * acquisitionRate);
            cursor += TakeSamples(ramp * acquisitionRate, ref elapsedFraction);
            int count;
            // 失真对截断和基波泄漏较敏感。这里按可用样本、周期数及与奈奎斯特界限的距离另选测量窗口。
            if (distortion)
            {
                double cycles = acquisitionRate / hz;
                // Hann width and the allowable notch Q require ten bins on either side.
                // 若取 N 点，频谱间距为 fs/N。要让基频距两端各至少 10 格，N 必须同时满足
                // N≥10fs/f 和 N≥10fs/(fs/2−f)。靠近任一端都需要更长的窗，实际长度还受下面的时间预算限制。
                double resolution = Math.Max(10 * cycles, 10 / (0.5 - hz / acquisitionRate));
                int available = (int)(measure[i] * acquisitionRate + 0.5);
                double baseCount = Math.Min(available, Math.Floor(resolution + 0.5));
                double initialSeconds = Math.Max(settings.MeasurementSeconds, settings.MeasurementCycles / hz);
                count = checked((int)(baseCount * Math.Max(1, measure[i] / initialSeconds)));
                // 传入实测 pointOrigins 时以各点的丢弃段起点为基准；无此信息才以计划位置加整通道延时近似。
                long origin = pointOrigins == null ? (long)cursor + channelDelay : pointOrigins[i];
                // 已知窗口长度，先在恒频段的 95% 处选定右边界，再减去长度得到起点。
                // 95% 是复现此切片方法的时间选择，Hann 窗的分辨率约束只参与上面的长度计算。
                start = origin + (int)(acquisitionRate * (discard[i] + measure[i]) * 0.95) - count;
            }
            else
            {
                count = TakeSamples(measure[i] * acquisitionRate, ref lengthFraction);
            }
            cursor += TakeSamples((discard[i] + measure[i]) * acquisitionRate, ref elapsedFraction);
            result[i] = new(hz, (int)Math.Clamp(start, 0, recordingLength - 7), count);
        }
        return result;
    }

    // 一段持续时间未必对应整数个样本，将本段的小数余量带到下一段，就不会每段四舍五入后累计出多点偏移。
    private static int TakeSamples(double exact, ref double fraction)
    {
        int whole = (int)exact;
        // 例如连续三段各占 2.4 点，向下取整会得到 2、2、2。把每段余下的小数带给下一段，
        // 就得到 2、2、3 点，余下 0.2 点继续累计，后续点位不会因逐段舍入而持续提前。
        fraction += exact - whole;
        int carry = (int)fraction;
        fraction -= carry;
        return whole + carry;
    }

    private static double Amplitude(double from, double to, int index, int count)
    {
        // 幅度切换使用余弦形的权重，段首和段尾更平滑，可减轻幅度突跳产生的宽带成分。
        if (from == to) return from;
        double window = (1 + Math.Cos(Math.PI / count * index)) / 2;
        // 升幅时平方的是余弦权重；降幅时在两个平方根幅度间过渡后再平方。两条曲线的端值都是给定的起止幅度。
        if (from < to) return to - (to - from) * window * window;
        double floor = Math.Sqrt(to / from);
        double envelope = floor + (1 - floor) * window;
        return from * envelope * envelope;
    }
}
