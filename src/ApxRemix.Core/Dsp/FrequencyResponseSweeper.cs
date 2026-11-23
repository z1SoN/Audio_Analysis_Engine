// 步进扫频是一串需要协调的动作。RunLiveSweepAsync 请求切换激励、等待新数据和稳定条件，再保留该频点；用回调接入采集，可让同一流程服务不同音频引擎。
using ApxRemix.Core.Audio;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>步进扫频采集、校验或调节中止异常。</summary>
public sealed class SweepMeasurementException : InvalidOperationException
{
    public SweepMeasurementException(string message) : base(message) { }
}

/// <summary>
/// 增强型步进扫频控制器（Stepped Sweep）。
/// <para>
/// - 统一物理电压增益标定：Gain = dBFS_in − dBFS_out_actual + 20·log10(InputRef / OutputRef)，与 Level &amp; Gain 表完全一致；
/// - 逐点调节明确区分 <see cref="RegulationOutcome.Converged"/>、<see cref="RegulationOutcome.LimitReached"/> 与 <see cref="RegulationOutcome.NotConverged"/>，
///   保留实际发出的发生器电平、实测电平与目标偏差，并按策略决定标记未达标或中止。
/// </para>
/// </summary>
public sealed class FrequencyResponseSweeper
{
    private static readonly string[] NestedPalette = ["#0284C7", "#D97706", "#7C3AED", "#059669", "#E11D48"];


    // 对数横轴上，100→200 Hz 与 1→2 kHz 占相同距离。n 个频点夹着 n−1 段，每一步乘 (fStop/fStart)^(1/(n−1))，最后落到终点。
    public static double[] GenerateLogFrequencies(double startHz, double stopHz, int pointCount)
    {
        int n = Math.Max(2, pointCount);
        double fStart = Math.Max(1.0, startHz);
        double fStop = Math.Max(1.0, stopHz);

        var freqs = new double[n];
        double ratio = Math.Pow(fStop / fStart, 1.0 / (n - 1));
        for (int i = 0; i < n; i++) freqs[i] = fStart * Math.Pow(ratio, i);
        return freqs;
    }

    // 线性轴每步加相同的差值。若横轴以 dB 表示，相等的 dB 步长对应电压逐次乘相同倍数。
    public static double[] GenerateLinearPoints(double startVal, double stopVal, int pointCount)
    {
        int n = Math.Max(2, pointCount);
        var pts = new double[n];
        double step = (stopVal - startVal) / (n - 1);
        for (int i = 0; i < n; i++) pts[i] = startVal + step * i;
        return pts;
    }

    // 线性与等比网格按所需点数生成；首选数系列、自定义表的点数由所选区间和实际表项决定。
    public static bool UsesPointCount(SweepSpacingMode spacing) => spacing is SweepSpacingMode.Logarithmic or SweepSpacingMode.Linear;

    // Nominal preferred frequencies in one decade. Each coarser series selects every 2nd, 4th or 8th entry.
    private static readonly int[] PreferredMantissas =
    [
        100,103,106,109,112,115,118,122,125,128,132,136,140,145,150,155,
        160,165,170,175,180,185,190,195,200,206,212,218,224,230,236,243,
        250,258,265,272,280,290,300,307,315,325,335,345,355,365,375,387,
        400,412,425,437,450,462,475,487,500,515,530,545,560,580,600,615,
        630,650,670,690,710,730,750,775,800,825,850,875,900,925,950,975
    ];
    public static double[] PreferredFrequencies(double start, double stop, SweepSpacingMode spacing)
    {
        int step = spacing switch { SweepSpacingMode.R10 => 8, SweepSpacingMode.R20 => 4, SweepSpacingMode.R40 => 2,
            SweepSpacingMode.R80 => 1, _ => throw new ArgumentException("Select an ISO R series.") };
        // R80 的标称点在每个十倍频程重复。R40 每隔两项取一点，以此类推，最后恢复用户指定的扫描方向。
        double low = Math.Min(start, stop), high = Math.Max(start, stop);
        if (!double.IsFinite(high) || !double.IsFinite(low) || low < 1 || high > 1e8)
            throw new ArgumentException("Invalid preferred frequency range.");
        List<double> frequencies = [];
        for (int decade = (int)Math.Floor(Math.Log10(low)); decade <= (int)Math.Floor(Math.Log10(high)); decade++)
        {
            double scale = Math.Pow(10, decade) / 100;
            for (int i=0; i<PreferredMantissas.Length; i+=step)
            {
                double frequency = PreferredMantissas[i] * scale;
                if (frequency >= low && frequency <= high) frequencies.Add(frequency);
            }
        }
        if (frequencies.Count < 2) throw new ArgumentException("The interval must contain at least two preferred frequencies.");
        if (start > stop) frequencies.Reverse();
        return frequencies.ToArray();
    }

    public static double[] GenerateSweepAxisPoints(FrequencySweepConfig cfg, int sampleRate)
    {
        if (cfg.SpacingMode == SweepSpacingMode.CustomTable && cfg.CustomPoints is { Length: >= 2 })
        {
            // 自定义表的数值可以是 Hz、dBFS、度或满量程偏置，须按横轴所表示的量限幅。
            return cfg.AxisType == SweepAxisType.Frequency
                ? cfg.CustomPoints.Select(f => Math.Clamp(f, 5.0, sampleRate * 0.45)).ToArray()
                : cfg.AxisType == SweepAxisType.GeneratorOffset ? cfg.CustomPoints.Select(x => Math.Clamp(x,-0.9,0.9)).ToArray()
                : cfg.AxisType == SweepAxisType.Phase
                    ? cfg.CustomPoints.Select(p => Math.Clamp(p, -180, 180)).ToArray()
                    : cfg.CustomPoints.Select(l => Math.Clamp(l, -100.0, 0.0)).ToArray();
        }

        if (cfg.AxisType == SweepAxisType.GeneratorOffset)
            return GenerateLinearPoints(cfg.StartOffsetFs,cfg.StopOffsetFs,cfg.PointCount);
        if (cfg.AxisType == SweepAxisType.Level)
            return GenerateLinearPoints(cfg.StartLevelDbfs, cfg.StopLevelDbfs, cfg.PointCount);
        if (cfg.AxisType == SweepAxisType.Phase)
            return GenerateLinearPoints(cfg.StartPhaseDeg, cfg.StopPhaseDeg, cfg.PointCount);

        // 采样率给出可表示频带上界，这里限制到 0.45 Fs，在奈奎斯特频率之前留出余量。
        double start = Math.Clamp(cfg.StartFrequencyHz, 5, sampleRate * 0.45);
        double stop = Math.Clamp(cfg.StopFrequencyHz, 5, sampleRate * 0.45);
        if (cfg.SpacingMode >= SweepSpacingMode.R10) return PreferredFrequencies(start, stop, cfg.SpacingMode);
        return cfg.SpacingMode == SweepSpacingMode.Linear
            ? GenerateLinearPoints(start, stop, cfg.PointCount)
            : GenerateLogFrequencies(start, stop, cfg.PointCount);
    }

    public static async Task<FrequencyResponseSummary> RunLiveSweepAsync(
        FrequencySweepConfig sweepConfig,
        int sampleRate,
        Action<GeneratorConfig> setGenerator,
        Func<int, AudioCaptureSnapshot> captureSnapshot,
        GeneratorConfig originalGenerator,
        Action<FrequencyResponsePoint, int, int>? onPointMeasured = null,
        CancellationToken cancellationToken = default)
    {
        if (sampleRate <= 0)
            throw new SweepMeasurementException("无效的采样率，无法执行步进扫频。");


        // 先确认本机激励和采集可用。外部信号源独立运行，本机不能通过自己的发生器电平闭环调节它。
        if (!sweepConfig.UseExternalFrequency && !originalGenerator.IsEnabled && !originalGenerator.AutoOn)
            throw new SweepMeasurementException("The generator is off. Enable it or select Auto On to run a generated sweep.");
        if (!captureSnapshot(1).IsEngineRunning)
            throw new SweepMeasurementException("音频采集已停止，无法启动步进扫频。");

        if (sweepConfig.UseExternalFrequency && (sweepConfig.AxisType != SweepAxisType.Frequency ||
            sweepConfig.EnableNestedSweep || sweepConfig.EnableTargetRegulation))
            throw new SweepMeasurementException("外部频率跟踪支持频率扫，不能接管外部信号的电平或逐点调节。");

        if (sweepConfig.EnableNestedSweep)
        {
            var nestedSeries = new List<SweepSeriesDataset>();
            FrequencyResponseSummary? lastSummary = null;
            var secondary = sweepConfig.SecondarySource;
            // 每条嵌套曲线应能解释成固定副轴、只改变主轴，因此要排除主轴与副轴控制同一个量的组合。
            var values = secondary == BenchSweepSource.GeneratorLevel ? sweepConfig.NestedLevelsDbfs : sweepConfig.SecondaryValues;
            if (values is not {Length: >= 2}) throw new SweepMeasurementException("Enter at least two secondary sweep points.");
            var primary = sweepConfig.AxisType switch
            {
                SweepAxisType.Level => BenchSweepSource.GeneratorLevel,
                SweepAxisType.Phase => BenchSweepSource.GeneratorPhase,
                SweepAxisType.GeneratorOffset => BenchSweepSource.GeneratorOffset,
                _ => Enum.TryParse<BenchSweepSource>(sweepConfig.ToneSource.ToString(),out var tone) ? tone : BenchSweepSource.GeneratorFrequency
            };
            if (!SweepNesting.Options(primary).Contains(secondary)) throw new SweepMeasurementException("The secondary source must be independent of the primary source.");
            (double min,double max) = secondary switch
            {
                BenchSweepSource.GeneratorOffset => (-.9,.9),
                BenchSweepSource.GeneratorPhase => (-180,180),
                BenchSweepSource.GeneratorLevel => (-100,0),
                _ => (5,sampleRate*.45)
            };
            if (values.Any(v=>!double.IsFinite(v) || v<min || v>max))
                throw new SweepMeasurementException($"Secondary values must be between {min:G} and {max:G} {SweepNesting.Unit(secondary)}.");
            int totalOuter = values.Length;

            try
            {
                for (int s = 0; s < totalOuter; s++)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    double value = values[s];
                    // 为每个副轴值复制一份发生器配置。内层完整扫一次主轴，回调将内层进度换算为整组曲线的进度。
                    var subCfg = sweepConfig.CopyForNestedPass();
                    subCfg.EnableNestedSweep = false;
                    var drive = originalGenerator.Clone();
                    switch (secondary)
                    {
                        case BenchSweepSource.GeneratorLevel: subCfg.SweepLevelDbfs = value; break;
                        case BenchSweepSource.GeneratorPhase: drive.Channel2PhaseOffsetDeg = value; break;
                        case BenchSweepSource.GeneratorOffset: drive.DcOffsetFs = value; break;
                        case BenchSweepSource.GeneratorFrequency: subCfg.FixedFrequencyForLevelSweepHz = value; break;
                        default: subCfg.CompanionToneHz = value; break;
                    }
                    var subSummary = await RunSingleLiveSweepPassAsync(
                        subCfg, sampleRate, setGenerator, captureSnapshot, drive,
                        (pt, done, total) => onPointMeasured?.Invoke(pt, s * total + done, totalOuter * total),
                        restoreGeneratorInFinally: false,
                        cancellationToken).ConfigureAwait(false);

                    nestedSeries.Add(new SweepSeriesDataset($"{value:0.0###} {SweepNesting.Unit(secondary)}", subSummary.Points, NestedPalette[s % NestedPalette.Length]));
                    lastSummary = subSummary;
                }
            }
            finally
            {
                // 所有副轴曲线共享发生器；由外层 finally 统一恢复实验开始前的设置，异常或取消也会执行。
                setGenerator(originalGenerator.Clone());
            }

            return lastSummary! with { NestedSeries = nestedSeries };
        }

        return await RunSingleLiveSweepPassAsync(
            sweepConfig, sampleRate, setGenerator, captureSnapshot, originalGenerator,
            onPointMeasured, restoreGeneratorInFinally: true, cancellationToken).ConfigureAwait(false);
    }

    private static async Task<FrequencyResponseSummary> RunSingleLiveSweepPassAsync(
        FrequencySweepConfig sweepConfig,
        int sampleRate,
        Action<GeneratorConfig> setGenerator,
        Func<int, AudioCaptureSnapshot> captureSnapshot,
        GeneratorConfig originalGenerator,
        Action<FrequencyResponsePoint, int, int>? onPointMeasured,
        bool restoreGeneratorInFinally,
        CancellationToken cancellationToken)
    {
        double[] axisPoints = GenerateSweepAxisPoints(sweepConfig, sampleRate);
        double toneOffsetDb = ValidateToneSource(sweepConfig, axisPoints, sampleRate);
        bool dualTone = sweepConfig.AxisType == SweepAxisType.Frequency && !sweepConfig.UseExternalFrequency &&
            sweepConfig.ToneSource != SweepToneSource.Sine;
        var rawPoints = new List<FrequencyResponsePoint>(axisPoints.Length);
        SweepPhaseReferenceMode resolvedRefMode = SweepPhaseReferenceMode.None;
        double calOffsetDb = AudioMetricsAnalyzer.ComputeCalibrationGainOffsetDb(
            sweepConfig.InputFullScaleVoltageVrms, sweepConfig.OutputFullScaleVoltageVrms);


        try
        {
            if (sweepConfig.UseExternalFrequency)
            {
                var muted = originalGenerator.Clone();
                muted.IsEnabled = false;
                setGenerator(muted);
            }

            // 预扫延时（对齐 APx500 PreSweepDelay）：在首个扫频点前留出外设稳定时间
            if (sweepConfig.PreSweepDelayMs > 0)
                await Task.Delay(Math.Clamp(sweepConfig.PreSweepDelayMs, 0, 10000), cancellationToken).ConfigureAwait(false);

            for (int i = 0; i < axisPoints.Length; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                long pointStart = System.Diagnostics.Stopwatch.GetTimestamp();

                // 把横轴值换为本轮的实际驱动量。双音模式另存要跟踪的那一音，其频率和幅度可以不同于发生器 A 音。
                double xVal = axisPoints[i];
                double f = dualTone ? ResolveTonePair(sweepConfig,xVal).Tracking : sweepConfig.AxisType == SweepAxisType.Frequency ? xVal : Math.Clamp(sweepConfig.FixedFrequencyForLevelSweepHz, 5, sampleRate * 0.45);
                bool regulate = sweepConfig.EnableTargetRegulation && sweepConfig.AxisType == SweepAxisType.Frequency;
                double genLevel = ClampDriveLevel(sweepConfig, sweepConfig.AxisType == SweepAxisType.Level ? xVal : sweepConfig.SweepLevelDbfs, regulate);
                int maxRegIters = regulate ? 5 : 1;
                double tol = Math.Max(0.02, sweepConfig.RegulationToleranceDb);

                AudioCaptureSnapshot snap = default!;
                BandpassLevelResult? bandpass = null;
                double ch1Db = -180, ch1RawPhase = 0, ch2Db = -180, ch2RawPhase = 0;
                double ch1ToneDb = -180, ch2ToneDb = -180;
                bool trackCh2 = sweepConfig.TrackingChannel == InputChannelSelection.Channel2;
                double trackedDb = double.NaN;
                double actualIssuedLevel = genLevel;
                RegulationOutcome regOutcome = regulate ? RegulationOutcome.NotConverged : RegulationOutcome.NotRequested;
                double regErrorDb = 0.0;
                bool hitClampLimit = false;

                // 每次调节都重新发出激励并等待新数据。沿用上一轮样本会让反馈重复校正同一个过期误差。
                for (int iter = 0; iter < maxRegIters; iter++)
                {
                    actualIssuedLevel = genLevel;
                    if (!sweepConfig.UseExternalFrequency)
                    {
                        setGenerator(CreateDrive(sweepConfig, f, xVal, actualIssuedLevel, dualTone, originalGenerator));
                    }

                    // 切换激励是一次命令，电路响应稳定则要等设备送来新的采样。在这一点等待的结果同时携带读数与采集快照，供本次频点计算使用。
                    double generatorHz = dualTone ? ResolveTonePair(sweepConfig, xVal).A : f;
                    var acquired = await WaitForFreshSteadySnapshotAsync(f, generatorHz, sampleRate, sweepConfig, captureSnapshot, cancellationToken).ConfigureAwait(false);
                    snap = acquired.Snapshot;
                    bandpass = acquired.Bandpass;
                    (_, ch1Db, ch1RawPhase) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(snap.Left, sampleRate, f);
                    (_, ch2Db, ch2RawPhase) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(snap.Right, sampleRate, f);
                    // AP Level is a software RMS meter, not a target FFT/lock-in bin.
                    // Preserve the tone readings for acquisition validation only.
                    ch1ToneDb = ch1Db; ch2ToneDb = ch2Db;
                    ch1Db = SignalGeneratorDsp.LinearToDbfs(acquired.Ch1Linear);
                    ch2Db = SignalGeneratorDsp.LinearToDbfs(acquired.Ch2Linear);
                    if (trackCh2 && !snap.HasDualChannel)
                        throw new SweepMeasurementException("当前采集只有一个独立输入，不能跟踪 Ch2。");
                    if (sweepConfig.AxisType == SweepAxisType.Phase && !snap.HasDualChannel)
                        throw new SweepMeasurementException("相位扫需要两个独立输入。");
                    trackedDb = trackCh2 ? ch2Db : ch1Db;
                    // 先验证被跟踪的音确实存在，才能用这个频率的测量值来驱动闭环。
                    var invalidReason = GetToneError(trackCh2 ? snap.Right : snap.Left, f,
                        trackCh2 ? ch2ToneDb : ch1ToneDb, sweepConfig.MinValidSignalDbfs, 20 - toneOffsetDb);
                    if (invalidReason is not null)
                        throw new SweepMeasurementException(invalidReason);

                    if (!regulate)
                        break;

                    // 误差统一定义为 (实测 - 目标)：负值代表实测低于目标，正值代表高于目标
                    regErrorDb = trackedDb - sweepConfig.TargetRegulatedLevelDbfs;
                    // 误差以 dB 表示。若通路在本次调节附近近似线性，输入比目标高 2 dB，就把激励降低 2 dB，再采一轮验证。
                    if (Math.Abs(regErrorDb) <= tol)
                    {
                        regOutcome = RegulationOutcome.Converged;
                        break;
                    }

                    double desiredNext = actualIssuedLevel - regErrorDb;
                    if (desiredNext > sweepConfig.RegulationMaxGeneratorDbfs || desiredNext < sweepConfig.RegulationMinGeneratorDbfs)
                    {
                        hitClampLimit = true;
                        double clamped = Math.Clamp(desiredNext, sweepConfig.RegulationMinGeneratorDbfs, sweepConfig.RegulationMaxGeneratorDbfs);
                        if (Math.Abs(clamped - actualIssuedLevel) < 1e-4)
                        {
                            // 已在发生器限幅边界测得稳态结果，明确判定 LimitReached 并结束迭代
                            regOutcome = RegulationOutcome.LimitReached;
                            break;
                        }
                        genLevel = clamped;
                    }
                    else
                    {
                        genLevel = desiredNext;
                    }
                }

                // 到达限幅和用完迭代次数要分别报告，两者都可能尚未达标。若允许继续，本点保存最后实际发出的电平。
                if (regulate && regOutcome != RegulationOutcome.Converged)
                {
                    regOutcome = hitClampLimit ? RegulationOutcome.LimitReached : RegulationOutcome.NotConverged;
                    if (sweepConfig.AbortOnRegulationFailure)
                    {
                        throw new SweepMeasurementException(
                            $"频点 {f:F1} Hz 目标调节未达标（{regOutcome}）：目标 {sweepConfig.TargetRegulatedLevelDbfs:F2} dBFS，实测 {trackedDb:F2} dBFS，发生器 {actualIssuedLevel:F2} dBFS。");
                    }
                }

                // 相位必须指定参考。同步参考给出各输入相对激励的相位，双通道相减则给出两路输入之间的相位差。
                var requestedPhase = sweepConfig.AxisType == SweepAxisType.Phase
                    ? SweepPhaseReferenceMode.Ch2RelativeToCh1 : sweepConfig.PhaseReferenceMode;
                resolvedRefMode = ResolvePhaseReferenceMode(requestedPhase, snap);
                double? ch1RelSyncPhase = null;
                double? ch2RelSyncPhase = null;
                double? measuredPhase = null;

                if (resolvedRefMode == SweepPhaseReferenceMode.SynchronousExcitation &&
                    snap.SyncReference is not null && snap.SyncReference.Length == snap.Left.Length)
                {
                    var (_, refDb, refRawPhase) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(snap.SyncReference, sampleRate, f);
                    if (refDb > -90.0)
                    {
                        ch1RelSyncPhase = NormalizeAngleDeg(ch1RawPhase - refRawPhase);
                        ch2RelSyncPhase = NormalizeAngleDeg(ch2RawPhase - refRawPhase);
                        measuredPhase = ch1RelSyncPhase;
                    }
                }
                else if (resolvedRefMode == SweepPhaseReferenceMode.Ch2RelativeToCh1 && snap.HasDualChannel && ch2Db > sweepConfig.MinValidSignalDbfs)
                {
                    double rel = sweepConfig.UseApxRealtimeReadings
                        ? snap.RealtimeReading?.Values.Ch2RelativeToCh1Deg ?? double.NaN
                        : NormalizeAngleDeg(ch2RawPhase - ch1RawPhase);
                    ch1RelSyncPhase = 0.0;
                    ch2RelSyncPhase = double.IsFinite(rel) ? rel : null;
                    measuredPhase = ch2RelSyncPhase;
                }

                // 以 NaN 表示还没有失真读数。仅在需要时计算残余量，避免把未测量的频点解释为零失真。
                double thdNPct = double.NaN, thdNDb = double.NaN;
                double ch2ThdNPct = double.NaN, ch2ThdNDb = double.NaN;
                BenchMeterSnapshot? meters = null;
                if (sweepConfig.CaptureBenchMeters)
                {
                    var drive = CreateDrive(sweepConfig, f, xVal, actualIssuedLevel, dualTone, originalGenerator);
                    drive.IsEnabled = !sweepConfig.UseExternalFrequency;
                    meters = AudioMetricsAnalyzer.AnalyzeBenchSnapshot(snap.Left, snap.Right, sampleRate, drive,
                        sweepConfig.AnalyzerFilter ?? new(), sweepConfig.InputFullScaleVoltageVrms, sweepConfig.OutputFullScaleVoltageVrms);
                    if (bandpass is not null) meters = meters with { BandpassLevel = bandpass };
                }
                if (!dualTone && (meters != null || sweepConfig.AxisType == SweepAxisType.Level ||
                    sweepConfig.YMetric is SweepYMetric.ThdPlusNPercent or SweepYMetric.ThdPlusNDb ||
                    sweepConfig.SecondaryMetric is SweepYMetric.ThdPlusNPercent or SweepYMetric.ThdPlusNDb))
                {
                    var af = sweepConfig.AnalyzerFilter;
                    var thd = meters?.ThdPlusN ?? AudioMetricsAnalyzer.AnalyzeThdPlusN(
                        snap.Left, snap.Right, sampleRate, f,
                        bwLowHz: af?.HighPassHz ?? 20.0,
                        bwHighHz: af?.LowPassHz ?? 20000.0,
                        residualWeighting: af?.ResidualWeighting ?? AnalyzerWeighting.Flat,
                        signalWeighting: af?.Weighting ?? AnalyzerWeighting.Flat,
                        lowPassType: af?.LowPassType ?? LowPassFilterType.Butterworth8,
                        thdReference: af?.ThdReference ?? ThdReferenceMode.TotalRmsIec);
                    if (thd.Ch1.HasValidSignal)
                    {
                        thdNPct = thd.Ch1.ThdNRatioPercent;
                        thdNDb = thd.Ch1.ThdNRatioDb;
                    }
                    if (snap.HasDualChannel && thd.Ch2.HasValidSignal)
                    {
                        ch2ThdNPct = thd.Ch2.ThdNRatioPercent;
                        ch2ThdNDb = thd.Ch2.ThdNRatioDb;
                    }
                }

                // 物理电压增益严格使用最后一次实际发出的发生器电平 + 统一 I/O 满量程标定偏置；Ch2 无有效信号时单独标 NaN
                // 输入与输出可能有不同的电压满量程。两个 dBFS 相减给出数字增益，还须加上两端参考电压之比的 dB 值才能得到电压增益。
                bool ch1HasSignal = !trackCh2 || GetToneError(snap.Left, f, ch1ToneDb, sweepConfig.MinValidSignalDbfs, 20 - toneOffsetDb) is null;
                bool ch2HasSignal = snap.HasDualChannel && (trackCh2 ||
                    GetToneError(snap.Right, f, ch2ToneDb, sweepConfig.MinValidSignalDbfs, 20 - toneOffsetDb) is null);
                double toneLevel = sweepConfig.UseExternalFrequency ? double.NaN : actualIssuedLevel + toneOffsetDb;
                double ch1Gain = ch1HasSignal ? ch1Db - toneLevel + calOffsetDb : double.NaN;
                double ch2ToneLevel = sweepConfig.UseExternalFrequency ? double.NaN :
                    (originalGenerator.LinkChannelLevels ? actualIssuedLevel : originalGenerator.EffectiveChannel2LevelDbfs) + toneOffsetDb;
                double ch2Gain = ch2HasSignal ? ch2Db - ch2ToneLevel + calOffsetDb : double.NaN;
                if (!ch1HasSignal) ch1RelSyncPhase = null;
                if (!ch2HasSignal) ch2RelSyncPhase = null;
                if (resolvedRefMode == SweepPhaseReferenceMode.Ch2RelativeToCh1 && (!ch1HasSignal || !ch2HasSignal))
                    measuredPhase = null;
                else if (resolvedRefMode == SweepPhaseReferenceMode.SynchronousExcitation)
                    measuredPhase = trackCh2 ? ch2RelSyncPhase : ch1RelSyncPhase;

                // 让同一点的读数、参考方式及调节状态一起流向绘图和导出，后续仍可查出该点的激励条件。
                var pt = new FrequencyResponsePoint(
                    FrequencyHz: f,
                    Ch1LevelDbfs: ch1HasSignal ? ch1Db : double.NaN,
                    Ch2LevelDbfs: ch2HasSignal ? ch2Db : double.NaN,
                    Ch1GainDb: ch1Gain,
                    Ch2GainDb: ch2Gain,
                    Ch1RelativeDb: ch1Gain,
                    Ch2RelativeDb: ch2Gain,
                    Ch1PhaseDeg: ch1RelSyncPhase,
                    Ch2PhaseDeg: ch2RelSyncPhase,
                    MeasuredPhaseDeg: measuredPhase,
                    XValue: xVal,
                    Ch1ThdNPercent: ch1HasSignal ? thdNPct : double.NaN,
                    Ch1ThdNDb: ch1HasSignal ? thdNDb : double.NaN,
                    Coherence: null,
                    IsValid: true,
                    ActualGeneratorLevelDbfs: sweepConfig.UseExternalFrequency ? double.NaN : actualIssuedLevel,
                    RegulationStatus: regOutcome,
                    RegulationErrorDb: regulate ? regErrorDb : 0.0,
                    Ch2ThdNPercent: ch2HasSignal ? ch2ThdNPct : double.NaN,
                    Ch2ThdNDb: ch2HasSignal ? ch2ThdNDb : double.NaN,
                    ExcitationToneDbfs: toneLevel, MeterReading: meters, InputRateHz:snap.SampleRate,
                    Ch1ErrorRate:snap.DigitalReading?.Left.ErrorRate, Ch2ErrorRate:snap.DigitalReading?.Right.ErrorRate);

                rawPoints.Add(pt);
                onPointMeasured?.Invoke(pt, i + 1, axisPoints.Length);
                int remainingMs = sweepConfig.PointIntervalMs -
                    (int)System.Diagnostics.Stopwatch.GetElapsedTime(pointStart).TotalMilliseconds;
                if (i + 1 < axisPoints.Length && remainingMs > 0)
                    await Task.Delay(remainingMs, cancellationToken).ConfigureAwait(false);
            }
        }
        finally
        {
            if (restoreGeneratorInFinally)
                setGenerator(originalGenerator.Clone());
        }

        return BuildSummary(rawPoints, sweepConfig.ReferenceFrequencyHz, resolvedRefMode,
            sweepConfig.TrackingChannel, sweepConfig.UseExternalFrequency);
    }

    private sealed record SettledAcquisition(AudioCaptureSnapshot Snapshot, double Ch1Linear, double Ch2Linear,
        BandpassLevelResult? Bandpass);

    // APx500 10.0 BenchModeKind and BandpassFrequencySweepContext, GetMeasurementSettlingDelay.
    public static double BandpassFrequencyDelayCycles(double frequencyHz, BandpassSelectivity width, double tolerancePercent)
    {
        // 根据带宽和允许误差估计等待的周期数。向上取整后，周期数除以发生频率才得到秒数。
        double fraction = width == BandpassSelectivity.OneOctave ? 0.0416666679084301 : (int)width / 24.0;
        double intercept = -33.5011803655725 * (width == BandpassSelectivity.OneOctave ? 2 : 1);
        return Math.Max(1, Math.Ceiling(Math.Log(tolerancePercent / 100.0)
            * fraction * (intercept + 0.00146730269562191 * frequencyHz)));
    }

    private static async Task<SettledAcquisition> WaitForFreshSteadySnapshotAsync(
        double freqHz, double generatorHz, int sampleRate, FrequencySweepConfig sweepConfig,
        Func<int, AudioCaptureSnapshot> captureSnapshot, CancellationToken cancellationToken)
    {
        // 记录切换激励时的末端样本序号。窗口长度足够并不保证内容来自新频点，还要等旧样本移出。
        var baselineSnap = captureSnapshot(1);
        if (!baselineSnap.IsEngineRunning)
            throw new SweepMeasurementException($"采集在频点 {freqHz:F1} Hz 切换时已停止。");

        long switchSampleIndex = baselineSnap.EndSampleIndex;
        int minWindowSamples = GetReadingSamples(sweepConfig, sampleRate, freqHz);
        int settleSamples = (int)Math.Ceiling(sampleRate * (Math.Max(0, sweepConfig.SettlingTimeMs) / 1000.0));
        var parameters = sweepConfig.Settling ?? new SweepSettlingConfig();
        var filters = sweepConfig.AnalyzerFilter ?? new AnalyzerFilterConfig { HighPassHz = 0, LowPassHz = 0 };
        bool useBandpass = sweepConfig.CaptureBenchMeters && sweepConfig.UseClockedBandpass
            && !(filters.LowPassHz > 0 && filters.LowPassType == LowPassFilterType.Aes17)
            && (filters.BandpassSelectivity != BandpassSelectivity.SecondOrderQ10 || filters.BandpassQ == 10);
        bool settleBandpass = sweepConfig.SettleBandpassLevel && useBandpass;
        if (settleBandpass && sweepConfig.AxisType == SweepAxisType.Frequency && !sweepConfig.UseExternalFrequency
            && filters.BandpassTuneMode == NotchTuningMode.Generator && filters.BandpassSelectivity != BandpassSelectivity.SecondOrderQ10)
        {
            double tolerance = parameters.Algorithm == SettlingAlgorithm.Flat && parameters.TolerancePercent > 0
                ? parameters.TolerancePercent : 1;
            double cycles = BandpassFrequencyDelayCycles(generatorHz, filters.BandpassSelectivity, tolerance);
            settleSamples = Math.Max(settleSamples, (int)Math.Ceiling(cycles * sampleRate / generatorHz));
        }
        long targetSampleIndex = switchSampleIndex + settleSamples + minWindowSamples;

        int expectedWaitMs = (int)Math.Ceiling((settleSamples + minWindowSamples) * 1000.0 / sampleRate);
        int timeoutMs = Math.Max(sweepConfig.StepTimeoutMs, expectedWaitMs + 100);
        var elapsed = System.Diagnostics.Stopwatch.StartNew();
        long lastExamined = -1;
        long lastReadingIndex = switchSampleIndex;
        long lastRealtimeIndex = -1;
        // Readings are paced by independent acquisitions. AP's original settler
        // operates in reading counts; never count several polls of the same data.
        // 稳定判断需要读数的时间间隔。时钟驱动的分支按每秒 20 次处理，两路总电平及带通电平分别保留历史。
        bool clocked = sweepConfig.UseApxRealtimeReadings || settleBandpass;
        double readingRate = clocked ? 20 : sampleRate / (double)minWindowSamples;
        var settler = new ApxReadingSettler(parameters, readingRate, timeoutMs / 1000.0);
        var otherSettler = new ApxReadingSettler(parameters, readingRate, timeoutMs / 1000.0);
        var bp1 = new ApxReadingSettler(parameters, readingRate, timeoutMs / 1000.0);
        var bp2 = new ApxReadingSettler(parameters, readingRate, timeoutMs / 1000.0);
        string? lastError = null;

        while (true)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var snap = captureSnapshot(minWindowSamples);

            if (!snap.IsEngineRunning)
                throw new SweepMeasurementException($"音频采集在频点 {freqHz:F1} Hz 中途停止，扫频已中止。");

            // 窗口长度、末端序号都符合条件才分析它；重复读取同一窗口不能增加一条独立的稳定证据。
            bool hasFresh = snap.EndSampleIndex >= targetSampleIndex;
            bool hasLen = snap.Left.Length >= minWindowSamples && snap.Right.Length >= minWindowSamples;
            if (hasFresh && hasLen)
            {
                long? realtimeIndex = snap.BenchReading?.EndSampleIndex ?? snap.RealtimeReading?.EndSampleIndex;
                if (snap.EndSampleIndex != lastExamined && (clocked
                    ? realtimeIndex != lastRealtimeIndex
                    : snap.EndSampleIndex - lastReadingIndex >= minWindowSamples))
                {
                    lastExamined = snap.EndSampleIndex;
                    lastReadingIndex = snap.EndSampleIndex;
                    bool trackCh2 = sweepConfig.TrackingChannel == InputChannelSelection.Channel2;
                    if (trackCh2 && !snap.HasDualChannel)
                        throw new SweepMeasurementException("当前采集只有一个独立输入，不能跟踪 Ch2。");
                    var input = trackCh2 ? snap.Right : snap.Left;
                    var (_, toneLevel, _) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(input, sampleRate, freqHz);
                    bool isDualTone = sweepConfig.AxisType == SweepAxisType.Frequency && sweepConfig.ToneSource != SweepToneSource.Sine && !sweepConfig.UseExternalFrequency;
                    // PCM 和实时仪表各有更新时刻。仪表样本位置仍在上一频点时，再等一次，避免将旧仪表值配给新窗口。
                    if (clocked)
                    {
                        if (realtimeIndex is not long index || index < targetSampleIndex || index <= lastRealtimeIndex)
                        {
                            lastError = "等待本频点的新 AP 实时读数。";
                            if (elapsed.ElapsedMilliseconds >= timeoutMs)
                                throw new SweepMeasurementException($"频点 {freqHz:F1} Hz 没有新 AP 实时读数，请检查采样率及采集状态。");
                            await Task.Delay(15, cancellationToken).ConfigureAwait(false);
                            continue;
                        }
                        lastRealtimeIndex = index;
                    }
                    // The phase detector always AC-couples its input. Use the main Bench RMS path,
                    // with the selected coupling/weighting and input trim, for level and gain.
                    // AES17 uses the frequency-domain window just as it does in the Bench meter.
                    bool useWindowForAes17 = filters.LowPassHz > 0 && filters.LowPassType == LowPassFilterType.Aes17;
                    var bench = sweepConfig.UseApxRealtimeReadings && !useWindowForAes17 ? snap.BenchReading : null;
                    // 读双音时单独提取被跟踪的音；单音模式按配置复用实时读数或对整段 PCM 算加权 RMS。两者计入的频率成分不同。
                    double ReadLevel(double[] pcm, bool second, double toneDb)
                    {
                        if (isDualTone) return toneDb;
                        if (bench is null)
                            return AudioMetricsAnalyzer.ComputeFullSignalWeightedRmsDbfs(pcm, sampleRate, filters);
                        double rms = second ? bench.WeightedCh2RmsLinear : bench.WeightedCh1RmsLinear;
                        return SignalGeneratorDsp.LinearToDbfs(rms * AudioMetricsAnalyzer.Sqrt2);
                    }
                    double level = ReadLevel(input, trackCh2, toneLevel);
                    // 比较目标音和总交流电平。默认允许总量高出 20 dB，即目标音幅度约为总交流正弦等效幅度的十分之一仍可通过。
                    var error = GetToneError(input, freqHz, toneLevel, sweepConfig.MinValidSignalDbfs);
                    int state = settler.Add(error is null ? Math.Pow(10, level / 20) : double.NaN, freqHz);
                    var other = trackCh2 ? snap.Left : snap.Right;
                    var (_, otherToneDb, _) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(other, sampleRate, freqHz);
                    double otherDb = ReadLevel(other, !trackCh2, otherToneDb);
                    otherSettler.Add(otherDb > sweepConfig.MinValidSignalDbfs ? Math.Pow(10, otherDb / 20) : double.NaN, freqHz);
                    BandpassLevelResult? bp = null;
                    bool bandReady = !settleBandpass;
                    // 带通滤波有记忆。先核对实时中心频率，再观察读数是否稳定，避免把上个频点的响应记在当前频点。
                    if (useBandpass && snap.BenchReading is { } reading && snap.SampleRate == sampleRate && reading.SampleRate == sampleRate
                        && reading.EndSampleIndex >= targetSampleIndex && reading.EndSampleIndex <= snap.EndSampleIndex
                        && reading.BandpassInput == new BandpassInputSettings(filters.HighPassHz, filters.LowPassHz,
                            filters.LowPassType, filters.BandpassSelectivity, filters.BandpassTuneMode, filters.Weighting))
                    {
                        double center = filters.BandpassCenterHz >= 10 ? filters.BandpassCenterHz : generatorHz;
                        center = Math.Clamp(center, 10, sampleRate * .48);
                        // 实测跟踪与外部频率的发生器跟踪接受任意有效正中心；其余模式还要落在预期中心的相对容差内。
                        bool centerMatches(double actual) => double.IsFinite(actual) && actual > 0 &&
                            (filters.BandpassTuneMode == NotchTuningMode.Measured ||
                            sweepConfig.UseExternalFrequency && filters.BandpassTuneMode == NotchTuningMode.Generator ||
                            Math.Abs(actual - center) <= center * .00011);
                        if (centerMatches(reading.BandpassCenterHz) && (!snap.HasDualChannel || centerMatches(reading.BandpassCh2CenterHz)))
                        {
                            double first = reading.BandpassCh1RmsLinear * AudioMetricsAnalyzer.Sqrt2;
                            double second = reading.BandpassCh2RmsLinear * AudioMetricsAnalyzer.Sqrt2;
                            if (settleBandpass)
                            {
                                int firstState = bp1.Add(first, freqHz);
                                int secondState = bp2.Add(second, freqHz);
                                bandReady = firstState == 1 && (!snap.HasDualChannel || secondState == 1);
                                first = bp1.Value;
                                second = bp2.Value;
                            }
                            bp = new(reading.BandpassCenterHz, SignalGeneratorDsp.LinearToDbfs(first),
                                SignalGeneratorDsp.LinearToDbfs(second)) { Ch2CenterFrequencyHz = reading.BandpassCh2CenterHz };
                        }
                    }
                    if (error is null)
                    {
                        // 先等跟踪声道判稳。若要求带通也判稳，额外检查每个已启用通道的带通窗口，再返回属于该频点的读数。
                        if (state == 1 && bandReady) return new(snap, trackCh2 ? otherSettler.Value : settler.Value,
                            trackCh2 ? settler.Value : otherSettler.Value, bp);
                        lastError = "目标频点尚未满足所设 AP Settling 参数。";
                    }
                    else
                    {
                        lastError = error;
                    }
                }
            }

            // 超时原因分成没有样本、采集尚未更新和数据中找不到有效目标音，各自指向不同的排查位置。
            if (elapsed.ElapsedMilliseconds >= timeoutMs)
            {
                if (snap.Left.Length == 0)
                    throw new SweepMeasurementException($"频点 {freqHz:F1} Hz 采集超时：输入缓冲为空（无音频数据流入）。");
                if (!hasFresh)
                    throw new SweepMeasurementException($"频点 {freqHz:F1} Hz 采集超时：采样序号未推进，可能为缓冲停滞或残留旧数据。");
                if (hasLen && sweepConfig.UseExternalFrequency)
                    throw new SweepMeasurementException($"等待外部信号 {freqHz:F1} Hz 超时，请检查跟踪通道和信号源。");
                if (hasLen && lastError is not null)
                    throw new SweepMeasurementException($"等待频点 {freqHz:F1} Hz 稳定超时：{lastError} 请检查回环路由、系统混音和其他音频程序。");
                throw new SweepMeasurementException($"频点 {freqHz:F1} Hz 采样长度不足（实得 {snap.Left.Length}，需要 {minWindowSamples}）。");
            }

            await Task.Delay(15, cancellationToken).ConfigureAwait(false);
        }
    }

    /// <summary>离线高速仿真扫频（与实时路径共用完全相同的物理电压标定与逐点调节状态判定）。</summary>
    public static FrequencyResponseSummary RunOfflineVirtualSweep(
        FrequencySweepConfig sweepConfig,
        VirtualDutConfig dutConfig,
        int sampleRate)
    {
        double[] axisPoints = GenerateSweepAxisPoints(sweepConfig, sampleRate);
        // 离线实验由软件生成激励和 DUT 响应，频率与幅度可重现。真实设备的延时和漂移要在采集路径上测量。
        if (sweepConfig.UseExternalFrequency)
            throw new SweepMeasurementException("离线虚拟扫描不能跟踪外部频率。");
        bool dualTone = sweepConfig.AxisType == SweepAxisType.Frequency && sweepConfig.ToneSource != SweepToneSource.Sine;
        double toneOffsetDb = ValidateToneSource(sweepConfig, axisPoints, sampleRate);
        var gen = new SignalGeneratorDsp();
        var dut = new VirtualDutProcessor();
        var rawPoints = new List<FrequencyResponsePoint>(axisPoints.Length);
        double calOffsetDb = AudioMetricsAnalyzer.ComputeCalibrationGainOffsetDb(
            sweepConfig.InputFullScaleVoltageVrms, sweepConfig.OutputFullScaleVoltageVrms);

        foreach (double xVal in axisPoints)
        {
            double f = dualTone ? ResolveTonePair(sweepConfig,xVal).Tracking : sweepConfig.AxisType == SweepAxisType.Frequency ? xVal : Math.Clamp(sweepConfig.FixedFrequencyForLevelSweepHz, 5, sampleRate * 0.45);
            bool regulate = sweepConfig.EnableTargetRegulation && sweepConfig.AxisType == SweepAxisType.Frequency;
            double genLevel = ClampDriveLevel(sweepConfig, sweepConfig.AxisType == SweepAxisType.Level ? xVal : sweepConfig.SweepLevelDbfs, regulate);
            int maxIters = regulate ? 5 : 1;
            double tol = Math.Max(0.02, sweepConfig.RegulationToleranceDb);

            double ch1Db = -180, ch2Db = -180, ch1Phase = 0, ch2Phase = 0, refPhase = 0;
            double actualIssuedLevel = genLevel;
            RegulationOutcome regOutcome = regulate ? RegulationOutcome.NotConverged : RegulationOutcome.NotRequested;
            double regErrorDb = 0.0;
            bool hitClampLimit = false;

            double[] dstL = Array.Empty<double>();
            double[] dstR = Array.Empty<double>();
            int settleSamples = Math.Clamp((int)(sampleRate * Math.Max(0.15, 10.0 / f)), 4096, 32768);
            int measureSamples = sweepConfig.ReadingWindowMs > 0
                ? GetReadingSamples(sweepConfig, sampleRate, f)
                : Math.Clamp((int)(sampleRate * Math.Max(0.10, 8.0 / f)), 4096, 16384);

            // 每一轮都重新生成激励并让模型响应，保留分析窗口前的过渡段，避免把启动过程当作稳态幅相。
            for (int iter = 0; iter < maxIters; iter++)
            {
                actualIssuedLevel = genLevel;
                gen.Reset();
                dut.Reset();
                var genCfg = CreateDrive(sweepConfig, f, xVal, actualIssuedLevel, dualTone);

                int total = settleSamples + measureSamples;
                var srcL = new double[total];
                var srcR = new double[total];
                dstL = new double[total];
                dstR = new double[total];

                gen.FillBlock(genCfg, sampleRate, srcL, srcR);
                dut.ProcessBlock(dutConfig, SignalGeneratorDsp.DbfsToLinearPeak(actualIssuedLevel), sampleRate, srcL, srcR, dstL, dstR);

                (_, ch1Db, ch1Phase) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(dstL.AsSpan(settleSamples), sampleRate, f);
                (_, ch2Db, ch2Phase) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(dstR.AsSpan(settleSamples), sampleRate, f);
                (_, _, refPhase) = AudioMetricsAnalyzer.ExtractSingleToneLockIn(srcL.AsSpan(settleSamples), sampleRate, f);

                if (!regulate) break;

                // 误差统一定义为 (实测 - 目标)：负值代表实测低于目标，正值代表高于目标
                regErrorDb = (sweepConfig.TrackingChannel == InputChannelSelection.Channel2 ? ch2Db : ch1Db)
                    - sweepConfig.TargetRegulatedLevelDbfs;
                if (Math.Abs(regErrorDb) <= tol)
                {
                    regOutcome = RegulationOutcome.Converged;
                    break;
                }

                double desiredNext = actualIssuedLevel - regErrorDb;
                if (desiredNext > sweepConfig.RegulationMaxGeneratorDbfs || desiredNext < sweepConfig.RegulationMinGeneratorDbfs)
                {
                    hitClampLimit = true;
                    double clamped = Math.Clamp(desiredNext, sweepConfig.RegulationMinGeneratorDbfs, sweepConfig.RegulationMaxGeneratorDbfs);
                    if (Math.Abs(clamped - actualIssuedLevel) < 1e-4)
                    {
                        regOutcome = RegulationOutcome.LimitReached;
                        break;
                    }
                    genLevel = clamped;
                }
                else
                {
                    genLevel = desiredNext;
                }
            }

            if (regulate && regOutcome != RegulationOutcome.Converged)
            {
                regOutcome = hitClampLimit ? RegulationOutcome.LimitReached : RegulationOutcome.NotConverged;
                if (sweepConfig.AbortOnRegulationFailure)
                {
                    throw new SweepMeasurementException(
                        $"频点 {f:F1} Hz 目标调节未达标（{regOutcome}）：目标 {sweepConfig.TargetRegulatedLevelDbfs:F2} dBFS，实测 {ch1Db:F2} dBFS。");
                }
            }

            // 用同一轮的响应计算失真和增益，并由激励的参考相位求两路相对相位，确保分子与分母属于同一组实验条件。
            var af = sweepConfig.AnalyzerFilter;
            var thd = AudioMetricsAnalyzer.AnalyzeThdPlusN(
                dstL.AsSpan(settleSamples), dstR.AsSpan(settleSamples), sampleRate, f,
                bwLowHz: af?.HighPassHz ?? 20.0,
                bwHighHz: af?.LowPassHz ?? 20000.0,
                residualWeighting: af?.ResidualWeighting ?? AnalyzerWeighting.Flat,
                signalWeighting: af?.Weighting ?? AnalyzerWeighting.Flat,
                lowPassType: af?.LowPassType ?? LowPassFilterType.Butterworth8,
                thdReference: af?.ThdReference ?? ThdReferenceMode.TotalRmsIec);
            double toneLevel = actualIssuedLevel + toneOffsetDb;
            double ch1Gain = ch1Db - toneLevel + calOffsetDb;
            double ch2Gain = ch2Db - toneLevel + calOffsetDb;
            double rel1 = NormalizeAngleDeg(ch1Phase - refPhase);
            double rel2 = NormalizeAngleDeg(ch2Phase - refPhase);
            double measuredPhase = sweepConfig.AxisType == SweepAxisType.Phase ? NormalizeAngleDeg(rel2 - rel1) : rel1;

            rawPoints.Add(new FrequencyResponsePoint(
                f, ch1Db, ch2Db, ch1Gain, ch2Gain, ch1Gain, ch2Gain, rel1, rel2, measuredPhase,
                XValue: xVal,
                Ch1ThdNPercent: !dualTone && thd.Ch1.HasValidSignal ? thd.Ch1.ThdNRatioPercent : double.NaN,
                Ch1ThdNDb: !dualTone && thd.Ch1.HasValidSignal ? thd.Ch1.ThdNRatioDb : double.NaN,
                Coherence: null,
                IsValid: true,
                ActualGeneratorLevelDbfs: actualIssuedLevel,
                RegulationStatus: regOutcome,
                RegulationErrorDb: regulate ? regErrorDb : 0.0,
                Ch2ThdNPercent: !dualTone && thd.Ch2.HasValidSignal ? thd.Ch2.ThdNRatioPercent : double.NaN,
                Ch2ThdNDb: !dualTone && thd.Ch2.HasValidSignal ? thd.Ch2.ThdNRatioDb : double.NaN,
                ExcitationToneDbfs: toneLevel));
        }

        var reference = sweepConfig.AxisType == SweepAxisType.Phase
            ? SweepPhaseReferenceMode.Ch2RelativeToCh1 : SweepPhaseReferenceMode.SynchronousExcitation;
        return BuildSummary(rawPoints, sweepConfig.ReferenceFrequencyHz, reference, sweepConfig.TrackingChannel);
    }

    // 闭环可用的上下限来自调节配置；普通扫描按输出范围限幅，实际下发值会影响该点增益的分母。
    private static double ClampDriveLevel(FrequencySweepConfig cfg, double level, bool regulate) =>
        regulate ? Math.Clamp(level, cfg.RegulationMinGeneratorDbfs, cfg.RegulationMaxGeneratorDbfs)
                 : Math.Clamp(level, -100.0, 0.0);

    private static double ValidateToneSource(FrequencySweepConfig cfg, double[] frequencies, int sampleRate)
    {
        if (cfg.AxisType == SweepAxisType.GeneratorOffset && frequencies.Any(x => !double.IsFinite(x) || Math.Abs(x) > .9))
            throw new SweepMeasurementException("Generator offset must be between −0.9 and +0.9 FS.");
        if (cfg.AxisType != SweepAxisType.Frequency || cfg.ToneSource == SweepToneSource.Sine || cfg.UseExternalFrequency)
            return 0;
        if (!double.IsFinite(cfg.CompanionToneHz) || cfg.CompanionToneHz < 5 || cfg.CompanionToneHz > sampleRate * 0.45)
            throw new SweepMeasurementException("双音的固定频率超出当前采样率允许范围。");
        // 有限记录的频率间隔约为 Fs/N。这里要求两个音至少隔开四个间隔，以便分析时区分两者。
        var pairs = frequencies.Select(x => ResolveTonePair(cfg,x)).ToArray();
        if (pairs.Any(p => !double.IsFinite(p.A) || !double.IsFinite(p.B) || Math.Min(p.A,p.B) < 5 || Math.Max(p.A,p.B) > sampleRate * .45))
            throw new SweepMeasurementException("The selected mean and difference place a tone outside the generator bandwidth.");
        if (pairs.Any(p => Math.Abs(p.A-p.B) < 4.0 * sampleRate / GetReadingSamples(cfg, sampleRate, p.Tracking)))
            throw new SweepMeasurementException("扫描点与双音的固定频率间隔过小，请修改点表或加长采样窗。");

        // 总峰值由两音共享。等幅双音中每个音仅占一半，相对总预算低约 6.02 dB，计算单音增益时要扣除这个差值。
        var pair = pairs[0];
        double ratio = SignalGeneratorDsp.DbfsToLinearPeak(pair.RatioDb);
        double fraction = (pair.Tracking == pair.A ? 1 : ratio) / (1 + ratio);
        return 20 * Math.Log10(fraction);
    }

    // DFD uses two equal-amplitude tones separated by the specified difference.
    // Track the lower carrier while retaining the user-selected mean/difference on X.
    private static (double A,double B,double Tracking,double RatioDb) ResolveTonePair(FrequencySweepConfig cfg,double x)
    {
        double fixedHz = cfg.CompanionToneHz;
        // 各双音模式对横轴有不同定义。由横轴先还原 A、B 两个实际频率，再选用于读数的那一个。
        return cfg.ToneSource switch
        {
            SweepToneSource.DfdMeanFrequency => (x-fixedHz/2,x+fixedHz/2,x-fixedHz/2,0),
            SweepToneSource.DfdDifferenceFrequency => (fixedHz-x/2,fixedHz+x/2,fixedHz-x/2,0),
            SweepToneSource.ImdFrequency1 => (x,fixedHz,x,-20*Math.Log10(4)),
            SweepToneSource.ImdFrequency2 => (fixedHz,x,x,-20*Math.Log10(4)),
            SweepToneSource.DualToneB => (fixedHz,x,x,cfg.DualToneRatioDb),
            _ => (x,fixedHz,x,cfg.DualToneRatioDb)
        };
    }

    private static GeneratorConfig CreateDrive(FrequencySweepConfig cfg, double frequency, double x, double level, bool dualTone, GeneratorConfig? original = null)
    {
        // 按频点复制驱动配置，只覆盖本次扫描要改变的调节量，让其他设置随原发生器继续生效。
        var drive = original?.Clone() ?? new GeneratorConfig();
        drive.IsEnabled = true;
        drive.Waveform = dualTone ? GeneratorWaveform.DualTone : GeneratorWaveform.Sine;
        var pair = ResolveTonePair(cfg,x);
        drive.FrequencyHz = dualTone ? pair.A : frequency;
        drive.SecondaryFrequencyHz = dualTone ? pair.B : frequency;
        drive.DualToneSecondaryRatioDb = pair.RatioDb;
        drive.DualToneMode = DualToneRoutingMode.Sum;
        if (cfg.AxisType == SweepAxisType.Phase) drive.Channel2PhaseOffsetDeg = x;
        if (cfg.AxisType == SweepAxisType.GeneratorOffset) drive.DcOffsetFs = x;
        drive.LevelDbfs = level;
        return drive;
    }

    public static int GetReadingSamples(FrequencySweepConfig cfg, int sampleRate, double frequencyHz)
    {
        // 显式读窗取指定时长、60 ms、六个周期中最大者，最后按缓冲区容量和最小样本数限制；切换后的等待时间另算。
        if (cfg.ReadingWindowMs > 0)
        {
            double seconds = Math.Max(Math.Max(0.06, 6.0 / frequencyHz), cfg.ReadingWindowMs / 1000.0);
            return Math.Clamp((int)Math.Ceiling(sampleRate * seconds), 2048, AudioEngineManager.RingCapacity);
        }

        // A fractional period leaves a phase-dependent error in the squared-sample
        // average, especially when the tone also carries a DC offset.
        // 自动窗先决定整数周期数，再换成最接近的整数样本数，以减轻截断小数周期造成的 RMS 偏差。
        double cycles = Math.Ceiling(Math.Max(0.06, 2048.0 / sampleRate) * frequencyHz);
        int samples = (int)Math.Round(Math.Max(6, cycles) * sampleRate / frequencyHz);
        return Math.Clamp(samples, 2048, AudioEngineManager.RingCapacity);
    }

    private static string? GetToneError(
        ReadOnlySpan<double> samples, double targetFreqHz, double toneLevelDbfs,
        double minValidSignalDbfs, double maxOtherEnergyDb = 20.0)
    {
        if (!double.IsFinite(toneLevelDbfs) || toneLevelDbfs < minValidSignalDbfs)
            return $"频点 {targetFreqHz:F1} Hz 未检测到有效信号（实测 {toneLevelDbfs:F1} dBFS，低于阈值 {minValidSignalDbfs:F1} dBFS）。";

        // 先减均值去掉直流，再比较总交流能量与目标音。否则一个很大的静态偏置会被误认为其他强交流分量。
        double mean = 0.0;
        for (int i = 0; i < samples.Length; i++) mean += samples[i];
        mean /= samples.Length;

        double sumSq = 0.0;
        for (int i = 0; i < samples.Length; i++) { double ac = samples[i] - mean; sumSq += ac * ac; }
        double totalAes17Linear = Math.Sqrt(sumSq / samples.Length) * AudioMetricsAnalyzer.Sqrt2;
        double totalDbfs = SignalGeneratorDsp.LinearToDbfs(totalAes17Linear);

        if (totalDbfs - toneLevelDbfs > maxOtherEnergyDb)
            return $"频点 {targetFreqHz:F1} Hz 读数无效：窗口能量由非目标频点主导（总电平 {totalDbfs:F1} dBFS vs 目标频点 {toneLevelDbfs:F1} dBFS）。";
        return null;
    }

    private static SweepPhaseReferenceMode ResolvePhaseReferenceMode(SweepPhaseReferenceMode requested, AudioCaptureSnapshot snap)
    {
        // 先判断有没有等长的同步参考、有没有第二路输入，再决定当前点能给出哪种相位。
        bool hasSync = snap.SyncReference is not null && snap.SyncReference.Length == snap.Left.Length;
        return requested switch
        {
            SweepPhaseReferenceMode.None => SweepPhaseReferenceMode.None,
            SweepPhaseReferenceMode.SynchronousExcitation => hasSync ? SweepPhaseReferenceMode.SynchronousExcitation : SweepPhaseReferenceMode.None,
            SweepPhaseReferenceMode.Ch2RelativeToCh1 => snap.HasDualChannel ? SweepPhaseReferenceMode.Ch2RelativeToCh1 : SweepPhaseReferenceMode.None,
            SweepPhaseReferenceMode.Auto => hasSync ? SweepPhaseReferenceMode.SynchronousExcitation : (snap.HasDualChannel ? SweepPhaseReferenceMode.Ch2RelativeToCh1 : SweepPhaseReferenceMode.None),
            _ => SweepPhaseReferenceMode.None
        };
    }

    // 相位标签标明哪个输入相对哪个参考，落后的正弦按负角度报告。
    public static string DescribePhaseReference(SweepPhaseReferenceMode mode) => mode switch
    {
        SweepPhaseReferenceMode.SynchronousExcitation => "Ch1 rel SyncRef (lag < 0°)",
        SweepPhaseReferenceMode.Ch2RelativeToCh1 => "Ch2 rel Ch1 (lag < 0°)",
        _ => "Phase N/A (No Sync Reference)"
    };

    // CSV 列名携带相同的参考关系，使导出的角度可追溯到同步激励或通道 1。
    public static string GetCsvPhaseColumnName(SweepPhaseReferenceMode mode) => mode switch
    {
        SweepPhaseReferenceMode.SynchronousExcitation => "Phase_Deg_Ch1_Rel_SyncRef",
        SweepPhaseReferenceMode.Ch2RelativeToCh1 => "Phase_Deg_Ch2_Rel_Ch1",
        _ => "Phase_Deg_Unavailable"
    };

    public static FrequencyResponseSummary BuildSummary(
        IReadOnlyList<FrequencyResponsePoint> rawPoints,
        double referenceFrequencyHz,
        SweepPhaseReferenceMode effectivePhaseReference = SweepPhaseReferenceMode.None,
        InputChannelSelection trackingChannel = InputChannelSelection.Channel1,
        bool useMeasuredLevel = false)
    {
        string phaseDesc = DescribePhaseReference(effectivePhaseReference);
        if (rawPoints.Count == 0)
            return new FrequencyResponseSummary(Array.Empty<FrequencyResponsePoint>(), 0, 0, 0, null, null, effectivePhaseReference, phaseDesc, null, 0);

        // 外部频率跟踪可用输入电平，其余情况下取电压增益。各路单独从有限值中选最接近参考频率的一点。
        double InputValue(FrequencyResponsePoint p, bool ch2) => useMeasuredLevel
            ? (ch2 ? p.Ch2LevelDbfs : p.Ch1LevelDbfs)
            : (ch2 ? p.Ch2GainDb : p.Ch1GainDb);
        // 按频率比的对数度量接近程度，例如参考上方一倍频程与下方一倍频程等距。
        double ReferenceDistance(FrequencyResponsePoint p) =>
            Math.Abs(Math.Log(Math.Max(p.FrequencyHz, 1) / Math.Max(referenceFrequencyHz, 1)));
        double Reference(bool ch2)
        {
            var p = rawPoints.Where(x => double.IsFinite(InputValue(x, ch2))).MinBy(ReferenceDistance);
            return p is null ? double.NaN : InputValue(p, ch2);
        }

        double refCh1 = Reference(false), refCh2 = Reference(true);
        var normalized = rawPoints.Select(p => p with
        {
            Ch1RelativeDb = InputValue(p, false) - refCh1,
            Ch2RelativeDb = InputValue(p, true) - refCh2
        }).ToArray();

        // 按频率排序后才能从参考点向低频及高频分别搜索阈值。原始扫描顺序仍保存在返回的数据点中。
        bool trackCh2 = trackingChannel == InputChannelSelection.Channel2;
        var sorted = normalized
            .Select(p => trackCh2 ? p with { Ch1RelativeDb = p.Ch2RelativeDb } : p)
            .Where(p => double.IsFinite(p.Ch1RelativeDb))
            .OrderBy(p => p.FrequencyHz).ToArray();
        double minDev = sorted.Length == 0 ? double.NaN : sorted.Min(p => p.Ch1RelativeDb);
        double maxDev = sorted.Length == 0 ? double.NaN : sorted.Max(p => p.Ch1RelativeDb);
        int refIdx = sorted.Length == 0 ? -1 : Array.IndexOf(sorted, sorted.MinBy(ReferenceDistance));
        double? lowMinus3 = refIdx < 0 ? null : FindCutoffFrequency(sorted, refIdx, searchHighSide: false, thresholdDb: -3.0);
        double? highMinus3 = refIdx < 0 ? null : FindCutoffFrequency(sorted, refIdx, searchHighSide: true, thresholdDb: -3.0);
        int unmetCount = normalized.Count(p => p.RegulationStatus is RegulationOutcome.LimitReached or RegulationOutcome.NotConverged);

        return new FrequencyResponseSummary(normalized, minDev, maxDev, maxDev - minDev, lowMinus3, highMinus3, effectivePhaseReference, phaseDesc, null, unmetCount);
    }

    private static double? FindCutoffFrequency(IReadOnlyList<FrequencyResponsePoint> points, int refIdx, bool searchHighSide, double thresholdDb)
    {
        int step = searchHighSide ? 1 : -1;
        for (int i = refIdx; i + step >= 0 && i + step < points.Count; i += step)
        {
            double y1 = points[i].Ch1RelativeDb;
            double y2 = points[i + step].Ch1RelativeDb;
            // 在相邻两点之间按 dB 对 log₁₀(f) 插值，因此 −3 dB 截止频率是由采样点估计的交点。
            if (y1 >= thresholdDb && y2 < thresholdDb)
            {
                double t = (thresholdDb - y1) / (y2 - y1);
                double logF1 = Math.Log10(Math.Max(points[i].FrequencyHz, 1.0));
                double logF2 = Math.Log10(Math.Max(points[i + step].FrequencyHz, 1.0));
                return Math.Pow(10.0, logF1 + t * (logF2 - logF1));
            }
        }
        return null;
    }

    public static double NormalizeAngleDeg(double deg)
    {
        // 相差整数圈表示同一相位，统一到 (−180°, 180°] 后，350° 就写作 −10°。
        double wrapped = ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return wrapped <= -180.0 ? 180.0 : wrapped;
    }
}
