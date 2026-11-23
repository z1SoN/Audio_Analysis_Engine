namespace ApxRemix.Core.Dsp;

public sealed record SequenceFileThdFrame(SequenceToneReading Left, SequenceToneReading? Right, int NextLength);

public sealed record SequenceFileThdResult(SequenceToneReading Left, SequenceToneReading? Right,
    int LastStart, int LastLength, int FramesEvaluated, bool TimedOut);

/// <summary>数字文件的 THD+N 批次。实时采集的 RMS 与桌面工作表的残余在同一采集边界汇合。</summary>
public static class SequenceFileThdBatch
{
    // 时间按采样率换算为点数；判稳要求至少 150 ms 或十个激励周期，允许电平相对变化 3%。
    private const int DelayMilliseconds = 20;
    private const double MinimumFrameSeconds = 0.25;
    private const double MinimumSettlingSeconds = 0.15;
    private const double SettlingCycles = 10;
    private const double Tolerance = 0.03;

    // 文件至少能覆盖起始延时和首帧；checked 让超出可表示点数的输入在这里暴露。
    public static int MinimumSamples(int sampleRate)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(sampleRate);
        return checked(FirstStart(sampleRate) + RequiredFrameLength(sampleRate, 2048));
    }

    // 从双声道或单声道文件逐帧采集，把每帧交给工作表回调，累计本次测量各个待报指标。
    public static SequenceFileThdResult Run(double[] left, double[]? right, int rate,
        Func<double[], double[]?, int, SequenceFileThdFrame> analyze, CancellationToken token,
        bool settlingEnabled = true, double generatorFrequency = 1000, bool ieeeRatio = false)
    {
        // 左右通道必须共用时间轴；正且有限的激励频率才能用周期数换算判稳时长。
        ArgumentNullException.ThrowIfNull(left);
        ArgumentNullException.ThrowIfNull(analyze);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(rate);
        if (right is not null && left.Length != right.Length)
            throw new ArgumentException("文件内的两个声道必须等长。", nameof(right));
        if (!(double.IsFinite(generatorFrequency) && generatorFrequency > 0))
            throw new ArgumentOutOfRangeException(nameof(generatorFrequency));

        // 采集检波状态随文件连续推进。残差电平和比值各自建历史，双声道还需要独立的左右历史。
        var capture = new SequenceCaptureRms(rate);
        var leftLevel = new MetricHistory();
        var leftRatio = new MetricHistory();
        var rightLevel = new MetricHistory();
        var rightRatio = new MetricHistory();
        MetricHistory[] histories = right is null ? [leftLevel, leftRatio] : [leftLevel, leftRatio, rightLevel, rightRatio];
        double requiredTime = Math.Max(MinimumSettlingSeconds, SettlingCycles / generatorFrequency);
        int start = FirstStart(rate), length = RequiredFrameLength(rate, 2048), evaluated = 0;
        while (true)
        {
            // 先确认能从文件中完整截取本帧；判稳尚未完成就到文件末尾属于采集失败。
            token.ThrowIfCancellationRequested();
            if (start > left.Length - length)
                throw new EndOfStreamException("WAV 在 THD+N 判稳完成前已结束。");

            // 采集检波处理完整数据包；工作表只收到 [start, end) 里的样本，两者的终点可能相差不到一包。
            int end = start + length;
            var captured = capture.CompleteRecord(left, right ?? left, end);
            var frame = analyze(left.AsSpan(start, length).ToArray(), right?.AsSpan(start, length).ToArray(), rate);
            token.ThrowIfCancellationRequested();
            evaluated++;
            // 回调必须返回与文件相同的声道数，否则后续等待各指标判稳会遗漏一个声道。
            if ((right is null) != (frame.Right is null))
                throw new InvalidOperationException("THD+N 工作表的声道数与采集帧不符。");

            // 工作表可请求更长的采集。过短的一帧不送入判稳，也不补零。
            // 需加长采集时先丢开这一帧读数；当前长度已经够用才把幅值和比值交给各自历史。
            if (frame.NextLength <= length)
            {
                AddReading(leftLevel, leftRatio, frame.Left, captured.Left);
                if (frame.Right is not null)
                    AddReading(rightLevel, rightRatio, frame.Right, captured.Right);
                // 所有启用的指标均就绪才结束，这一帧仍提供频率等其他字段，并汇总是否有超时。
                if (histories.All(history => history.IsReady))
                {
                    var combinedLeft = Combine(frame.Left, leftLevel, leftRatio);
                    var combinedRight = frame.Right is null ? null : Combine(frame.Right, rightLevel, rightRatio);
                    return new(combinedLeft, combinedRight, start, length, evaluated,
                        histories.Any(history => history.TimedOut));
                }
            }

            // 44.1 kHz 包的整数样本数按实际采集时钟结转。使用包末的样本索引继续采集。
            // 下一帧从采集包边界起步，帧长由刚算出的频率需求和测量最短时长共同决定。
            start = captured.CompletedPacketEnd;
            length = RequiredFrameLength(rate, frame.NextLength);

            // 同一残差进两条历史。IEEE 直接取相对基波的比值，常规模式用包时钟发布的 RMS 作分母。
            void AddReading(MetricHistory level, MetricHistory ratio, SequenceToneReading reading, double captureRms)
            {
                level.Add(reading.ThdNLevel, start, end, rate, requiredTime, settlingEnabled);
                double value = ieeeRatio ? reading.ThdNRatioIeee
                    : reading.IsNoise ? double.NaN : reading.ThdNLevel / captureRms;
                ratio.Add(value, start, end, rate, requiredTime, settlingEnabled);
            }
        }

        // 回填已判稳的残差和被选中的那种比值，其余字段仍来自最后一帧工作表。
        SequenceToneReading Combine(SequenceToneReading reading, MetricHistory level, MetricHistory ratio) =>
            ieeeRatio ? reading with { ThdNLevel = level.Result, ReportedThdNRatioIeee = ratio.Result }
                : reading with { ThdNLevel = level.Result, ReportedThdNRatio = ratio.Result };
    }

    // 起始延迟按整数采样点截断；先转 long 避免采样率与毫秒数相乘时过早溢出。
    private static int FirstStart(int rate) => (int)((long)rate * DelayMilliseconds / 1000);

    // 工作表要求的点数和 250 ms 中选较大者，再向上选支持的变换长度。
    private static int RequiredFrameLength(int rate, int requestedLength) =>
        SequenceTransform.NextLength(Math.Max(requestedLength, (int)Math.Ceiling(MinimumFrameSeconds * rate)));

    // 每个指标按各自的历史倒查。AC 的结果取已判稳读数的均方根，其余工作表值保留最后一帧。
    // 一个指标一份历史，acceptedStart 指向本次已接受区间的第一条读数。
    private sealed class MetricHistory
    {
        private readonly List<(int Start, double Value)> readings = [];
        private int acceptedStart = -1;

        internal bool IsReady => acceptedStart >= 0;
        internal bool TimedOut { get; private set; }

        // 一条直接返回，多条按读数的平方平均再开方；相邻帧的时长不作为权重。
        internal double Result
        {
            get
            {
                if (acceptedStart == readings.Count - 1)
                    return readings[acceptedStart].Value;
                double sum = 0;
                for (int index = acceptedStart; index < readings.Count; index++)
                    sum += readings[index].Value * readings[index].Value;
                return Math.Sqrt(sum / (readings.Count - acceptedStart));
            }
        }

        // 新值到来后重新考察以它为结尾的区间，先清除上次接受范围和超时标记。
        internal void Add(double value, int start, int end, int rate, double requiredTime, bool enabled)
        {
            readings.Add((start, value));
            acceptedStart = -1;
            TimedOut = false;
            // 关闭判稳就接受最新一条；启用时至少攒齐三条才开始检查。
            if (!enabled)
            {
                acceptedStart = readings.Count - 1;
                return;
            }
            if (readings.Count < 3)
                return;

            // 从最近向前检查每条值是否在最新读数的容差带内，最新值为 NaN 时只与 NaN 视作可比。
            double lower = value * (1 - Tolerance), upper = value * (1 + Tolerance);
            for (int index = readings.Count - 1; index >= 0; index--)
            {
                double candidate = readings[index].Value;
                bool comparable = double.IsNaN(value) ? double.IsNaN(candidate) : candidate >= lower && candidate <= upper;
                if (comparable)
                {
                    // 区间的条数及持续时间都满足才接受，采样间隔由起点与本帧终点换算。
                    if (readings.Count - index >= 3 && SpanFrom(index) >= requiredTime)
                    {
                        acceptedStart = index;
                        break;
                    }
                    continue;
                }
                // 遇到不可比的读数且总时长已过十倍限值时，取最近约五倍时长的区间并标记超时。
                if (SpanFrom(0) >= 10 * requiredTime)
                {
                    acceptedStart = readings.Count - 1;
                    while (acceptedStart > 0 && SpanFrom(acceptedStart) < 5 * requiredTime)
                        acceptedStart--;
                    TimedOut = true;
                }
                break;
            }

            // 采样索引之差除以采样率得到秒，供正常判稳和超时窗口共用。
            double SpanFrom(int index) => (end - readings[index].Start) / (double)rate;
        }
    }
}
