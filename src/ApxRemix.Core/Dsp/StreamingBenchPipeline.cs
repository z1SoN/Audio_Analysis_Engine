// 这是 Local 实时 Bench 的汇合点。从 AudioEngineManager 或 MeasurementLab 接收 PCM，按样本数分帧，并让各通道的滤波器和检波器跨帧延续。
// 沿 Process → ProcessPacket → PublishReading 阅读。音频包推进滤波和检波，读取才清空报告累积并更新跟踪反馈。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// 连续实时 Bench 测量状态机链路。
/// <para>
/// 在每个音频回调块上连续无间断运行（跨任意回调分块边界保持全部 IIR/检波器状态）：
/// 1. 输入通道增益微调（InputCh1GainDb / InputCh2GainDb）；
/// 2. 滤波前 DC 均值与 Raw Peak / Clipping 统计；
/// 3. 上游信号路径连续状态滤波（交流耦合选项，Butterworth LP8 → 软件 HP4 → 时域计权 IIR）；
/// 4. 并行连续检测器：加权 RMS、非加权 AC RMS、6 阶 Butterworth 带通 RMS、AC Peak、Crest Factor、频率与通道间相位。
/// </para>
/// </summary>
public sealed class StreamingBenchPipeline
{
    public const double DefaultReadingsPerSecond = 20.0;
    public const int PanelReadingsPerSecond = 4;

    private readonly int _basePacketSize;
    private int _packetsPerReading;
    private readonly bool _transportClock;
    private double _readingRateHz;
    private double? _pendingReadingRateHz;
    private readonly double _fractionalPacketSize;
    private double _packetRemainder;
    private int _packetSize;
    private int _packetCount;
    private readonly double[] _pendingL;
    private readonly double[] _pendingR;
    private int _pendingCount;
    private long _processedSamples;
    private int _readingSampleCount;
    private double _rawDcSum1, _rawDcSum2, _rawPeak1, _rawPeak2, _acPeak1, _acPeak2;
    private PanelReportParameters? _panelReportParameters;
    private (AnalyzerFilterConfig Filter, double Gain1Db, double Gain2Db, double GeneratorHz)? _pendingPanelConfiguration;

    // 记录会影响面板报告的设置。与上一组比较时才能决定是否重新计数本轮已经送入的毫秒包（对齐 APxRT AU1wHikrya）。
    private readonly record struct PanelReportParameters(
        double GeneratorHz, double HighPassHz, HighPassFilterType HighPassType,
        bool Ch1HighPassEnabled, bool Ch2HighPassEnabled,
        double LowPassHz, LowPassFilterType LowPassType,
        bool Ch1LowPassEnabled, bool Ch2LowPassEnabled,
        AnalyzerWeighting MainWeighting, bool Ch1WeightingEnabled, bool Ch2WeightingEnabled,
        AnalyzerWeighting ResidualWeighting,
        NotchTuningMode NotchMode, double FixedNotchHz, NotchTuningMode BandpassMode,
        double FixedBandpassHz, BandpassSelectivity Selectivity, bool PhaseReferenceIsCh2,
        bool EnableRealtimeImd, ImdMeasurementStandard ImdStandard);

    // 配置快照（仅在参数真正变化时重建对应 IIR，避免无故清空滤波器状态）
    private double _ch1GainDb;
    private double _ch2GainDb;
    private double _highPassHz = -1;
    private HighPassFilterType _highPassType = HighPassFilterType.Butterworth4;
    private double _hpRelativeCutoff;
    private double _lowPassHz = -1;
    private LowPassFilterType _lowPassType = LowPassFilterType.Butterworth8;
    private double _ellipticRelativeCutoff;
    private AnalyzerWeighting _weighting = AnalyzerWeighting.Flat;
    private bool _hpOn1 = true, _hpOn2 = true;
    private bool _lpOn1 = true, _lpOn2 = true;
    private bool _wtOn1 = true, _wtOn2 = true;
    private bool _enableRealtimeImd = true;
    private BandpassSelectivity _bpSelectivity = BandpassSelectivity.ThirdOctave;
    private NotchTuningMode _bpTuneMode = NotchTuningMode.Generator;
    private double _bpFixedCenterHz = 1000.0;
    private double _generatorFreqHz = 1000.0;
    private double _activeBpCenter1, _activeBpCenter2;

    // 连续状态滤波器（Ch1 / Ch2 各自独立状态）
    private readonly ApxInputEqFilter _inputEq = new();
    private ApxAcCouplingFilter _acOnly1, _acOnly2;
    private StreamingIirCascade _hp1, _hp2;
    private StreamingIirCascade _lp1, _lp2;
    private StreamingIirCascade _wt1, _wt2;
    private StreamingIirCascade _bp1, _bp2;

    // 连续检波器，读数时清空报告累积。
    private readonly ApxRmsDetector _weightedRms1, _weightedRms2;
    private readonly ApxRmsDetector _acRms1, _acRms2;
    private readonly ApxRmsDetector _bpRms1, _bpRms2;
    private readonly ApxFrequencyPhaseDetector _phase1 = new(), _phase2 = new();
    private double _prevPeak1, _prevPeak2;
    private readonly DigitalDcMeter _dc1, _dc2;
    private readonly DigitalResidualMeter _residual1, _residual2;
    private readonly ApxRealtimeImdMeter _imd;
    private AnalyzerWeighting _residualWeighting;
    private NotchTuningMode _notchMode;
    private double _fixedNotchHz;

    // 工作缓冲区（复用，避免热路径频繁 GC）
    private readonly double[] _workL1, _workR1;
    private readonly double[] _acL, _acR;
    private readonly double[] _bpL, _bpR;
    private readonly FftEngine.WindowData? _bandpassRecordWindow;
    private readonly FftEngine.WindowData? _bandpassRecordWindowPlusOne;

    public int SampleRate { get; }
    public int ReadingsPerSecond { get; }
    public StreamingBenchReading? Latest { get; private set; }
    public ApxRealtimeReading? LatestPhaseReading { get; private set; }

    // 默认路径按每秒 20 次建立读数。面板的 4 Hz 节拍由另一构造入口显式选择。
    public StreamingBenchPipeline(int sampleRate, AnalyzerFilterConfig? initialFilter = null)
        : this(sampleRate, initialFilter, (int)DefaultReadingsPerSecond) { }

    // 采样率决定一秒有多少音频点，读数率决定多久报告一次。这两种时间尺度在面板路径中由毫秒包连接。
    public StreamingBenchPipeline(int sampleRate, AnalyzerFilterConfig? initialFilter, int readingsPerSecond)
        : this(sampleRate, initialFilter, readingsPerSecond, readingsPerSecond == PanelReadingsPerSecond) { }

    internal static StreamingBenchPipeline ForRecorder(int sampleRate, AnalyzerFilterConfig filter, double readingRateHz) =>
        new(sampleRate, filter, readingRateHz, transportClock: true);

    private StreamingBenchPipeline(int sampleRate, AnalyzerFilterConfig? initialFilter, double readingsPerSecond, bool transportClock)
    {
        if (!double.IsFinite(readingsPerSecond) || readingsPerSecond <= 0 || readingsPerSecond > 1000)
            throw new ArgumentOutOfRangeException(nameof(readingsPerSecond), "ReadingsPerSecond must be in (0, 1000].");
        SampleRate = Math.Max(8000, sampleRate);
        ReadingsPerSecond = (int)Math.Round(readingsPerSecond);
        _readingRateHz = readingsPerSecond;
        _transportClock = transportClock;
        if (_transportClock)
        {
            // APx500 file input emits one transport report per millisecond. The
            // fractional samples carry into later reports (44/45 at 44.1 kHz).
            double samplesPerMillisecond = SampleRate * .001;
            _basePacketSize = (int)Math.Floor(samplesPerMillisecond);
            _fractionalPacketSize = samplesPerMillisecond - _basePacketSize;
            _packetsPerReading = (int)Math.Ceiling(1000 / _readingRateHz);
        }
        else
        {
            // 支持任意读速（如 16/20/50/64 Hz）；小数样本跨读数结转以保持精确平均读速（如 44.1 kHz / 64 Hz = 689.0625）。
            double exactSamples = SampleRate / _readingRateHz;
            if (exactSamples >= 64.0)
            {
                _basePacketSize = (int)Math.Floor(exactSamples);
                _fractionalPacketSize = exactSamples - _basePacketSize;
                if (_fractionalPacketSize < 1e-12) _fractionalPacketSize = 0.0;
            }
            else
            {
                _basePacketSize = 64;
                _fractionalPacketSize = 0.0;
            }
            _packetsPerReading = 1;
            // The original WAV RealtimeProcessor begins at zero. Keep the
            // standalone non-panel aperture's established sub-PCM initial envelope.
            _prevPeak1 = _prevPeak2 = double.Epsilon;
            _bandpassRecordWindow = FftEngine.CreateWindow(_basePacketSize, FftWindowType.BlackmanHarris);
            if (_fractionalPacketSize > 0.0)
                _bandpassRecordWindowPlusOne = FftEngine.CreateWindow(_basePacketSize + 1, FftWindowType.BlackmanHarris);
        }
        // 预留可能多出一个点的最大包长。主路、交流路和带通路有各自缓冲，避免就地处理提前改写其他分支的输入。
        _packetSize = NextPacketSize();
        int capacity = _basePacketSize + (_fractionalPacketSize > 0 ? 1 : 0);
        _pendingL = new double[capacity];
        _pendingR = new double[capacity];
        _workL1 = new double[capacity];
        _workR1 = new double[capacity];
        _acL = new double[capacity];
        _acR = new double[capacity];
        _bpL = new double[capacity];
        _bpR = new double[capacity];

        // 两个声道及各仪表的检波器都需独立记忆。即使带宽相同，同一时刻听到的波形也可能不同。
        _dc1 = new(SampleRate);
        _dc2 = new(SampleRate);
        _residual1 = new(SampleRate, _readingRateHz);
        _residual2 = new(SampleRate, _readingRateHz);
        _imd = new(SampleRate, _readingRateHz);
        _acOnly1 = new ApxAcCouplingFilter(SampleRate);
        _acOnly2 = new ApxAcCouplingFilter(SampleRate);
        _weightedRms1 = new ApxRmsDetector(SampleRate, _readingRateHz);
        _weightedRms2 = new ApxRmsDetector(SampleRate, _readingRateHz);
        _acRms1 = new ApxRmsDetector(SampleRate, _readingRateHz);
        _acRms2 = new ApxRmsDetector(SampleRate, _readingRateHz);
        _bpRms1 = new ApxRmsDetector(SampleRate, _readingRateHz);
        _bpRms2 = new ApxRmsDetector(SampleRate, _readingRateHz);

        // 从直通开始，再用实际设置装入滤波器。这样未开启的一段也能与其余级采用同一调用顺序。
        _hp1 = StreamingIirCascade.Bypass();
        _hp2 = StreamingIirCascade.Bypass();
        _lp1 = StreamingIirCascade.Bypass();
        _lp2 = StreamingIirCascade.Bypass();
        _wt1 = StreamingIirCascade.Bypass();
        _wt2 = StreamingIirCascade.Bypass();
        _bp1 = StreamingIirCascade.Bypass();
        _bp2 = StreamingIirCascade.Bypass();

        var cfg = initialFilter ?? new AnalyzerFilterConfig();
        Configure(cfg, 0.0, 0.0, 1000.0);
    }

    /// <summary>配置 APxRT 原生兼容的双槽位 IIR Biquad 输入 EQ（对齐 TwoChannelMeasurer.SetEqFilter）。</summary>
    public void SetEqFilter(int order, double[] b1, double[] b2, double[] a1, double[] a2,
        double overallGain, int index, bool on1, bool on2) =>
        _inputEq.SetEqFilter(order, b1, b2, a1, a2, overallGain, index, on1, on2);

    // 面板设置先做快照，下一包到来才应用，从而避免在一包中间混用两套状态。常规路径立即生效。
    /// <summary>热更新信号路径滤波与带通配置（仅在参数实际改变时重建受影响的滤波器段）。</summary>
    public void Configure(AnalyzerFilterConfig filter, double ch1GainDb, double ch2GainDb, double generatorFreqHz)
    {
        if (_transportClock)
        {
            _pendingPanelConfiguration = (filter.Clone(), ch1GainDb, ch2GainDb, generatorFreqHz);
            return;
        }
        ApplyConfiguration(filter, ch1GainDb, ch2GainDb, generatorFreqHz);
    }

    internal void SetRecorderReadingRate(double readingRateHz) => _pendingReadingRateHz = readingRateHz;

    // 按设置变化分别更新需要重设计的级。中心频率用 Hz，通道增益用 dB，延迟状态是否保留取决于对应分支。
    private void ApplyConfiguration(AnalyzerFilterConfig filter, double ch1GainDb, double ch2GainDb, double generatorFreqHz)
    {
        double generator = Math.Clamp(generatorFreqHz, 5.0, SampleRate * 0.48);
        bool bpModeChanged = _bpSelectivity != filter.BandpassSelectivity
            || _bpTuneMode != filter.BandpassTuneMode
            || ((!_transportClock || filter.BandpassTuneMode == NotchTuningMode.Fixed)
                && Math.Abs(_bpFixedCenterHz - filter.BandpassCenterHz) > 1e-3);
        // 面板报告参数改变时重新计算本轮包数。这里的计数复位与后面各检波器的状态更新是不同操作。
        if (_transportClock)
        {
            // RealtimeProcessor compares ProcessingParameters before consuming the
            // next packet, including stored frequencies for inactive filter modes.
            // A report restart retains detector accumulations and filter delays.
            var parameters = new PanelReportParameters(
                generatorFreqHz, filter.HighPassHz, filter.HighPassType,
                filter.Ch1HighPassEnabled, filter.Ch2HighPassEnabled,
                filter.LowPassHz, filter.LowPassType,
                filter.Ch1LowPassEnabled, filter.Ch2LowPassEnabled,
                filter.Weighting, filter.Ch1WeightingEnabled, filter.Ch2WeightingEnabled,
                filter.ResidualWeighting, filter.NotchTuningMode,
                filter.FixedNotchFrequencyHz, filter.BandpassTuneMode, filter.BandpassCenterHz,
                filter.BandpassSelectivity, filter.ReferenceChannel == AnalyzerReferenceChannel.Channel2,
                filter.EnableRealtimeImd, filter.ImdStandard);
            if (_panelReportParameters != parameters)
            {
                _panelReportParameters = parameters;
                _packetCount = 0;
            }
        }

        _notchMode = filter.NotchTuningMode;
        _fixedNotchHz = filter.FixedNotchFrequencyHz;
        _residualWeighting = filter.ResidualWeighting;
        _ch1GainDb = ch1GainDb;
        _ch2GainDb = ch2GainDb;
        _generatorFreqHz = generator;
        _hpOn1 = filter.Ch1HighPassEnabled;
        _hpOn2 = filter.Ch2HighPassEnabled;
        _lpOn1 = filter.Ch1LowPassEnabled;
        _lpOn2 = filter.Ch2LowPassEnabled;
        _wtOn1 = filter.Ch1WeightingEnabled;
        _wtOn2 = filter.Ch2WeightingEnabled;
        _enableRealtimeImd = filter.EnableRealtimeImd;
        _imd.Configure(filter.EnableRealtimeImd ? filter.ImdStandard : ImdMeasurementStandard.Auto, filter.DfdMeterLowPassHz);

        // 10 Hz 附近的选择由专用交流耦合级实现，其他高通按 Butterworth4 或 Elliptic5 拓扑建立。
        // 同拓扑下 HighPassHz 微调采用 APxRT FUN_10006120 的 1e-5 相对门限并保留滤波延迟历史；仅拓扑切换时重置。
        if (_highPassHz != filter.HighPassHz || _highPassType != filter.HighPassType)
        {
            bool wasActiveHp = _highPassHz > 0.0 && !(UseAcCoupling && _highPassType == HighPassFilterType.Butterworth4);
            bool wantAcCoupling = Math.Abs(filter.HighPassHz - 10.0) < 0.05 && filter.HighPassType == HighPassFilterType.Butterworth4;
            bool wantActiveHp = filter.HighPassHz > 0.0 && !wantAcCoupling;
            bool sameTopology = wasActiveHp && wantActiveHp && _highPassType == filter.HighPassType;

            _highPassHz = filter.HighPassHz;
            _highPassType = filter.HighPassType;

            if (!wantActiveHp)
            {
                _hp1 = StreamingIirCascade.Bypass();
                _hp2 = StreamingIirCascade.Bypass();
                _hpRelativeCutoff = 0.0;
            }
            else
            {
                double cutoff = ApxEllipticLowPass.RelativeCutoff(_highPassHz, SampleRate);
                if (!sameTopology || Math.Abs(cutoff - _hpRelativeCutoff) > _hpRelativeCutoff * ApxEllipticLowPass.RelativeRetuneTolerance)
                {
                    var (left, right) = _highPassType == HighPassFilterType.Elliptic5
                        ? ApxEllipticLowPass.CreateEllipticHighPassStereo(_highPassHz, SampleRate)
                        : (StreamingIirCascade.CreateButterworthHighPass(4, _highPassHz, SampleRate),
                           StreamingIirCascade.CreateButterworthHighPass(4, _highPassHz, SampleRate));
                    if (sameTopology)
                    {
                        left.CopyHistoryFrom(_hp1);
                        right.CopyHistoryFrom(_hp2);
                    }
                    (_hp1, _hp2) = (left, right);
                    _hpRelativeCutoff = cutoff;
                }
            }
        }

        // 所有读速（4/16/20/50/64 Hz）统一支持 APxRT 8 阶椭圆低通（AES17 / Elliptic8），不再仅限 4 Hz 面板。
        bool isEllipticLp = filter.LowPassType is LowPassFilterType.Aes17 or LowPassFilterType.Elliptic8;
        if ((isEllipticLp ? _lowPassHz != filter.LowPassHz : Math.Abs(_lowPassHz - filter.LowPassHz) > 1e-6)
            || _lowPassType != filter.LowPassType)
        {
            bool wasEllipticLp = _lowPassHz > 0 && _lowPassType is LowPassFilterType.Aes17 or LowPassFilterType.Elliptic8;
            _lowPassHz = filter.LowPassHz;
            _lowPassType = filter.LowPassType;
            if (_lowPassHz > 0 && isEllipticLp)
            {
                double cutoff = ApxEllipticLowPass.RelativeCutoff(_lowPassHz, SampleRate);
                if (!wasEllipticLp || Math.Abs(cutoff - _ellipticRelativeCutoff) > _ellipticRelativeCutoff * ApxEllipticLowPass.RelativeRetuneTolerance)
                {
                    var (left, right) = ApxEllipticLowPass.CreateStereo(cutoff, SampleRate);
                    if (wasEllipticLp)
                    {
                        left.CopyHistoryFrom(_lp1);
                        right.CopyHistoryFrom(_lp2);
                    }
                    (_lp1, _lp2) = (left, right);
                    _ellipticRelativeCutoff = cutoff;
                }
            }
            else
            {
                bool useButterLp = _lowPassHz > 0.0 && _lowPassType == LowPassFilterType.Butterworth8;
                _lp1 = useButterLp ? StreamingIirCascade.CreateButterworthLowPass(8, _lowPassHz, SampleRate) : StreamingIirCascade.Bypass();
                _lp2 = useButterLp ? StreamingIirCascade.CreateButterworthLowPass(8, _lowPassHz, SampleRate) : StreamingIirCascade.Bypass();
            }
        }

        // 计权只在类型变化时重建，每个声道都使用独立的状态，否则一侧的瞬态会污染另一侧。
        if (_weighting != filter.Weighting)
        {
            _weighting = filter.Weighting;
            _wt1 = ApxAnsiWeighting.Create(_weighting, SampleRate);
            _wt2 = ApxAnsiWeighting.Create(_weighting, SampleRate);
        }

        _bpSelectivity = filter.BandpassSelectivity;
        _bpTuneMode = filter.BandpassTuneMode;
        _bpFixedCenterHz = filter.BandpassCenterHz;

        // 从各自上一次发布的频率读数求带通中心，两路实测频率可以不同。
        TuneBandpass(ref _bp1, ref _activeBpCenter1, Latest?.Phase.Ch1FrequencyHz ?? 0, bpModeChanged);
        TuneBandpass(ref _bp2, ref _activeBpCenter2, Latest?.Phase.Ch2FrequencyHz ?? 0, bpModeChanged);
    }

    // 该设置区间映射至专用交流耦合通路，其他正截止值使用通用高通设计。
    private bool UseAcCoupling => Math.Abs(_highPassHz - 10.0) < 0.05;

    // 实测跟踪的分数倍频程带通有独立的调谐规则，固定 Q 的二阶带通走另一分支。
    private bool TracksMeasuredOctave => _bpTuneMode == NotchTuningMode.Measured && _bpSelectivity != BandpassSelectivity.SecondOrderQ10;

    // 确定下一段的目标中心。实测值尚不可用时，部分模式会保留当前中心或使用发生器设置，使带通仍能工作。
    private double ResolveBandpassTargetCenter(double measuredFreqHz, double activeCenter)
    {
        // The previous measured frequency is zero on startup. The APx octave
        // filter bounds its center to 5 Hz until the first detector reading.
        if (TracksMeasuredOctave) return Math.Clamp(measuredFreqHz, 5.0, SampleRate * 0.495);
        double center = _bpTuneMode switch
        {
            NotchTuningMode.Fixed when _bpFixedCenterHz >= 10.0 => _bpFixedCenterHz,
            NotchTuningMode.Measured when measuredFreqHz <= 10 && activeCenter > 0 => activeCenter,
            NotchTuningMode.Measured => measuredFreqHz > 10.0 ? measuredFreqHz : _generatorFreqHz,
            NotchTuningMode.Generator when _transportClock => _generatorFreqHz,
            _ when _bpFixedCenterHz > 10.0 => _bpFixedCenterHz,
            _ => _generatorFreqHz
        };
        return Math.Clamp(center, 10.0, SampleRate * 0.48);
    }

    // 比较的是相对频率变化。显著变化、强制重设以及实测倍频程更新可能启动新的空状态。
    private void TuneBandpass(ref StreamingIirCascade filter, ref double activeCenter, double measured, bool force = false)
    {
        double desired = ResolveBandpassTargetCenter(measured, activeCenter);
        double change = Math.Abs(desired - activeCenter) / Math.Max(1, activeCenter);
        if (!force && change <= .0001) return;
        // 按选择性建立固定 Q 的二阶通路或分数倍频程的多节通路，两者的带宽定义不同。
        var updated = _bpSelectivity == BandpassSelectivity.SecondOrderQ10
            ? StreamingIirCascade.CreateSecondOrderBandpassQ(desired, 10, SampleRate)
            : StreamingIirCascade.CreateButterworthBandpass(desired, (int)_bpSelectivity, SampleRate);
        // Measured octave filters restart the filter sections on a retune.
        // The downstream RMS detector retains its history across that boundary.
        // 对满足条件的小幅调谐继承上包的延迟值，避免随着轻微频漂不断重复启动暂态。
        if (!TracksMeasuredOctave && !force && change < .005) updated.CopyHistoryFrom(filter);
        activeCenter = desired;
        filter = updated;
    }

    // 从音频回调的任意块长拼出仪表需要的内部包。尚未构成一包的部分留在缓冲中，绝对位置按实际到达样本累加。
    /// <summary>向连续状态机送入任意长度的音频回调采样块。</summary>
    public void Process(ReadOnlySpan<double> left, ReadOnlySpan<double> right) => Process(left, right, null);

    internal void Process(ReadOnlySpan<double> left, ReadOnlySpan<double> right, Action<StreamingBenchReading>? report)
    {
        int count = Math.Min(left.Length, right.Length);
        int offset = 0;
        while (offset < count)
        {
            // Drivers can split protocol reports at any sample. A completed report
            // advances DSP input, and a completed reading advances its feedback.
            int take = Math.Min(_packetSize - _pendingCount, count - offset);
            left.Slice(offset, take).CopyTo(_pendingL.AsSpan(_pendingCount));
            right.Slice(offset, take).CopyTo(_pendingR.AsSpan(_pendingCount));
            _pendingCount += take;
            offset += take;
            _processedSamples += take;

            // 满包才进入滤波和报告流程，因此音频回调边界本身不会强制更新仪表。
            if (_pendingCount == _packetSize)
                ProcessPendingPacket(report);
        }
    }

    // 面板输入结束时允许处理剩余的短包。它仍要经过正常的包计数判据，才可能触发完整面板读数。
    /// <summary>At WAV EOF the original Panel processes the final short input report.</summary>
    public void CompleteInput() => CompleteInput(null);

    internal void CompleteInput(Action<StreamingBenchReading>? report)
    {
        if (_transportClock && _pendingCount > 0)
            ProcessPendingPacket(report);
    }

    // 在包边界应用待定配置，再处理当前实际点数。达到设定包数才发布，然后计算下一包长度。
    private void ProcessPendingPacket(Action<StreamingBenchReading>? report)
    {
        if (_pendingPanelConfiguration is { } pending)
        {
            // Only the settings attached to a complete file report reach the
            // original processor. Intermediate changes must not reset it.
            ApplyConfiguration(pending.Filter, pending.Gain1Db, pending.Gain2Db, pending.GeneratorHz);
            _pendingPanelConfiguration = null;
        }
        if (_pendingReadingRateHz is double rate)
        {
            _pendingReadingRateHz = null;
            if (rate != _readingRateHz)
            {
                _readingRateHz = rate;
                _packetsPerReading = (int)Math.Ceiling(1000 / rate);
                _packetCount = 0;
                _weightedRms1.SetReadingRate(rate);
                _weightedRms2.SetReadingRate(rate);
                _acRms1.SetReadingRate(rate);
                _acRms2.SetReadingRate(rate);
                _bpRms1.SetReadingRate(rate);
                _bpRms2.SetReadingRate(rate);
                _residual1.SetReadingRate(rate);
                _residual2.SetReadingRate(rate);
                _imd.SetReadingRate(rate);
            }
        }
        ProcessPacket(_pendingCount);
        if (++_packetCount == _packetsPerReading)
        {
            PublishReading();
            report?.Invoke(Latest!);
        }
        _pendingCount = 0;
        _packetSize = NextPacketSize();
    }

    // 若 Fs/1000 非整数，将小数误差逐包累积，凑足一个样本便给本包加 1，保持长期平均包长。
    private int NextPacketSize()
    {
        _packetRemainder += _fractionalPacketSize;
        if (_packetRemainder < 1) return _basePacketSize;
        _packetRemainder -= 1;
        return _basePacketSize + 1;
    }

    // 从同一包派生原始、交流、带通和残余等读数，先保存仍要分流的数据，再就地复用主工作区。
    private void ProcessPacket(int count)
    {
        var workL = _workL1.AsSpan(0, count);
        var workR = _workR1.AsSpan(0, count);
        var acL = _acL.AsSpan(0, count);
        var acR = _acR.AsSpan(0, count);
        var bandpassL = _bpL.AsSpan(0, count);
        var bandpassR = _bpR.AsSpan(0, count);
        // 电压幅度的 dB 增益换成 10^(dB/20)，分别作用于每个声道的全部样本。
        double scale1 = Math.Abs(_ch1GainDb) > 1e-6 ? Math.Pow(10.0, _ch1GainDb / 20.0) : 1.0;
        double scale2 = Math.Abs(_ch2GainDb) > 1e-6 ? Math.Pow(10.0, _ch2GainDb / 20.0) : 1.0;

        // 1. 通道增益微调 + 滤波前 DC 均值与 Raw Peak/Clipping
        // 增益后的原波形直接累计有符号和及绝对峰值。该分支包含偏置，削波标志由这里的峰值形成。
        _readingSampleCount += count;
        for (int i = 0; i < count; i++)
        {
            double l = _pendingL[i] * scale1;
            double r = _pendingR[i] * scale2;
            workL[i] = l;
            workR[i] = r;
            _rawDcSum1 += l;
            _rawDcSum2 += r;
            double al = Math.Abs(l);
            double ar = Math.Abs(r);
            if (al > _rawPeak1) _rawPeak1 = al;
            if (ar > _rawPeak2) _rawPeak2 = ar;
        }

        // 1b. APxRT SetEqFilter 原生双槽位 IIR 输入均衡（位于频率/相位检测与主带宽滤波之前，APxRT.c 8263-8282）
        _inputEq.ProcessInPlace(workL, workR);

        // 2. 交流参考支路
        // 同一份输入分两路。交流副本用于寻找过零和测交流参考，原样本留给主路径。
        // 若在分路前减掉每帧均值，直流档的恒定输入也会被消掉。
        _acOnly1.Process(workL, acL);
        _acOnly2.Process(workR, acR);

        // 耦合后的波形另行取峰值，并交给交流 RMS 检波，能与保留直流的原始峰值分开观察。
        for (int i = 0; i < count; i++)
        {
            double al = Math.Abs(acL[i]);
            double ar = Math.Abs(acR[i]);
            if (al > _acPeak1) _acPeak1 = al;
            if (ar > _acPeak2) _acPeak2 = ar;
        }
        _acRms1.Process(acL);
        _acRms2.Process(acR);

        // APxRT Process sends the conditioned input to the frequency/phase pair
        // before applying the main HP/LP and weighting (FUN_10008104).
        // 交越门限使用上个报告期留下的峰值估计；即使此包幅度突然变化，也要等报告时才更新门限。
        _phase1.ProcessBlock(acL, SampleRate, _prevPeak1);
        _phase2.ProcessBlock(acR, SampleRate, _prevPeak2);

        // 3. 特殊交流耦合档复用交流参考波形。软件路径按 APxRT 顺序：先低通、再高通、最后计权，均支持分声道开关。
        bool useAcCoupling = UseAcCoupling && _highPassType == HighPassFilterType.Butterworth4;
        if (useAcCoupling)
        {
            if (_hpOn1) acL.CopyTo(workL);
            if (_hpOn2) acR.CopyTo(workR);
        }
        if (_lpOn1) _lp1.ProcessInPlace(workL);
        if (_lpOn2) _lp2.ProcessInPlace(workR);
        if (_highPassHz > 0.0 && !useAcCoupling)
        {
            if (_hpOn1) _hp1.ProcessInPlace(workL);
            if (_hpOn2) _hp2.ProcessInPlace(workR);
        }

        // 4. APxRT 把计权写回主波形，带通、主电平和实时 IMD 都从这一步的结果取样。
        if (_wtOn1) _wt1.ProcessInPlace(workL);
        if (_wtOn2) _wt2.ProcessInPlace(workR);

        // 从同一计权波形复制出带通分支，单独读窄带电平；后续宽带、IMD 和残余仪表继续使用原主路数据。
        _bp1.Process(workL, bandpassL);
        _bp2.Process(workR, bandpassR);
        _bpRms1.Process(bandpassL);
        _bpRms2.Process(bandpassR);

        // 5. 主路径连续 RMS、DC/峰值与实时 IMD 检波（必须在破坏性陷波覆盖工作区之前消费）
        _weightedRms1.Process(workL);
        _weightedRms2.Process(workR);
        _dc1.Process(workL);
        _dc2.Process(workR);
        if (_enableRealtimeImd) _imd.Process(workL, workR);
        // Residual processing may overwrite the main buffer. All other detectors
        // have consumed it, and tuning still uses the previous published frequency.
        _residual1.Process(workL, NotchCenter(Latest?.Phase.Ch1FrequencyHz ?? 0), _residualWeighting, _highPassHz <= 0 || !_hpOn1);
        _residual2.Process(workR, NotchCenter(Latest?.Phase.Ch2FrequencyHz ?? 0), _residualWeighting, _highPassHz <= 0 || !_hpOn2);
    }

    // 固定、实测和发生器模式分别给陷波器不同的中心 Hz；实测来自上次发布的频率。
    private double NotchCenter(double measured) => _notchMode switch
    {
        NotchTuningMode.Fixed => _fixedNotchHz,
        NotchTuningMode.Measured => measured,
        _ => _generatorFreqHz
    };

    // 在报告边界读完各支路的累计量，形成一条带有绝对样本位置的同步读数。
    private void PublishReading()
    {
        double wRms1 = _weightedRms1.Read();
        double wRms2 = _weightedRms2.Read();
        double acRms1 = _acRms1.Read();
        double acRms2 = _acRms2.Read();
        double bpRms1 = _bpRms1.Read();
        double bpRms2 = _bpRms2.Read();
        var (dc1, peak1) = _dc1.Read();
        var (dc2, peak2) = _dc2.Read();
        double residual1 = _residual1.Read();
        double residual2 = _residual2.Read();
        var (imd1, imd2) = _enableRealtimeImd ? _imd.Read() : (double.NaN, double.NaN);
        // 先记住本报告真正使用的带通中心，随后测得的新频率仅调谐未来样本。
        double measuredCenter1 = _activeBpCenter1, measuredCenter2 = _activeBpCenter2;

        // 正弦峰值等于 RMS × √2。把这次的交流读数换成等效正弦峰值，留作下一帧过零门限的尺度。
        // 有尖峰的波形，其实际最大值可能大得多，实际峰值已单独记录。
        // 交流 RMS 乘 √2 是正弦峰值估计，供下一次交越检测；非正弦波的实际峰值仍由样本峰值分支给出。
        _prevPeak1 = acRms1 * Math.Sqrt(2.0);
        _prevPeak2 = acRms2 * Math.Sqrt(2.0);

        var p1 = _phase1.Read();
        var p2 = _phase2.Read();
        // 只有双路均已得到频率才产生普通相对相位，按第一声道周期把交越时刻差换算成角度。
        double? relPhase = (p1.FrequencyHz > 0 && p2.FrequencyHz > 0)
            ? ApxFrequencyPhaseDetector.RelativePhaseDeg(p1.CrossingTimeSeconds, p2.CrossingTimeSeconds, p1.FrequencyHz)
            : null;

        // Measured 带通模式下平滑跟踪新测得频率
        if (_bpTuneMode == NotchTuningMode.Measured)
        {
            TuneBandpass(ref _bp1, ref _activeBpCenter1, p1.FrequencyHz);
            TuneBandpass(ref _bp2, ref _activeBpCenter2, p2.FrequencyHz);
        }

        var phaseReading = new ApxPhaseReading(
            p1.FrequencyHz, p2.FrequencyHz,
            p1.CrossingTimeSeconds, p2.CrossingTimeSeconds,
            acRms1, acRms2, relPhase);

        (double Ch1, double Ch2)? panelPhaseDegrees = null;
        // 面板的指定参考声道显示为零，另一声道的交越秒差乘该参考声道的频率。
        if (_transportClock)
        {
            // The selected reference supplies both the crossing and frequency.
            // For unequal channel frequencies the two reference choices are not opposites.
            panelPhaseDegrees = _panelReportParameters?.PhaseReferenceIsCh2 == true
                ? (ApxFrequencyPhaseDetector.RelativePhaseDeg(p2.CrossingTimeSeconds, p1.CrossingTimeSeconds, p2.FrequencyHz), 0)
                : (0, ApxFrequencyPhaseDetector.RelativePhaseDeg(p1.CrossingTimeSeconds, p2.CrossingTimeSeconds, p1.FrequencyHz));
        }

        // 此次读数区分主路加权 RMS、专用交流 RMS、带通 RMS 与滤后 DC，名称相近但取样位置不同。
        LatestPhaseReading = new ApxRealtimeReading(phaseReading, _processedSamples, SampleRate, _readingRateHz);
        Latest = new StreamingBenchReading(
            Phase: phaseReading,
            EndSampleIndex: _processedSamples,
            SampleRate: SampleRate,
            ReadingRateHz: _readingRateHz,
            WeightedCh1RmsLinear: wRms1,
            WeightedCh2RmsLinear: wRms2,
            AcCh1RmsLinear: acRms1,
            AcCh2RmsLinear: acRms2,
            AcCh1PeakLinear: _acPeak1,
            AcCh2PeakLinear: _acPeak2,
            RawCh1PeakLinear: _rawPeak1,
            RawCh2PeakLinear: _rawPeak2,
            Ch1Clipping: _rawPeak1 >= 0.999,
            Ch2Clipping: _rawPeak2 >= 0.999,
            Ch1DcFs: _rawDcSum1 / _readingSampleCount,
            Ch2DcFs: _rawDcSum2 / _readingSampleCount,
            BandpassCenterHz: measuredCenter1,
            BandpassCh1RmsLinear: bpRms1,
            BandpassCh2RmsLinear: bpRms2)
        {
            ReportSampleCount = _readingSampleCount,
            PanelPhaseDegrees = panelPhaseDegrees,
            BandpassCh2CenterHz = measuredCenter2,
            BandpassRecordCh1Rms = BandpassRecordRms(_bpL.AsSpan(0, Math.Min(_readingSampleCount, _bpL.Length))),
            BandpassRecordCh2Rms = BandpassRecordRms(_bpR.AsSpan(0, Math.Min(_readingSampleCount, _bpR.Length))),
            BandpassInput = new(_highPassHz, _lowPassHz, _lowPassType, _bpSelectivity, _bpTuneMode, _weighting),
            FilteredCh1DcFs = dc1,
            FilteredCh2DcFs = dc2,
            FilteredCh1Peak = peak1,
            FilteredCh2Peak = peak2,
            Ch1ResidualRms = residual1,
            Ch2ResidualRms = residual2,
            Ch1ImdRatio = imd1,
            Ch2ImdRatio = imd2
        };
        // 报告发布后清除本轮样本计数、直流求和与峰值，滤波器和交越计数器继续沿用其自身历史。
        _packetCount = _readingSampleCount = 0;
        _rawDcSum1 = _rawDcSum2 = _rawPeak1 = _rawPeak2 = _acPeak1 = _acPeak2 = 0;
    }

    // Sequence's Level worksheet receives the filtered acquisition; Bench's RMS
    // detector additionally retains its envelope across acquisitions and retunes.
    // 对常规包的带通波形单独计算窗加权交流 RMS。未建立记录窗时用 NaN 表示该项不可用。
    private double BandpassRecordRms(ReadOnlySpan<double> samples)
    {
        var window = samples.Length == _basePacketSize
            ? _bandpassRecordWindow
            : samples.Length == _basePacketSize + 1 ? _bandpassRecordWindowPlusOne : null;
        if (window is null) return double.NaN;
        var w = window.Coefficients;
        double mean = 0;
        // 先按窗系数和求直流并减去，再按窗平方和归一功率，开方得到与输入同单位的有效值。
        for (int i = 0; i < samples.Length; i++) mean += w[i] * samples[i];
        mean /= samples.Length * window.CoherentGain;
        double power = 0;
        for (int i = 0; i < samples.Length; i++)
        {
            double value = w[i] * (samples[i] - mean);
            power += value * value;
        }
        return Math.Sqrt(power / (samples.Length * window.NoisePowerGain));
    }
}

/// <summary>连续实时测量状态机输出快照。</summary>
public readonly record struct BandpassInputSettings(double HighPassHz, double LowPassHz, LowPassFilterType LowPassType,
    BandpassSelectivity Selectivity, NotchTuningMode Tuning, AnalyzerWeighting Weighting = AnalyzerWeighting.Flat);

public sealed record StreamingBenchReading(
    ApxPhaseReading Phase,
    long EndSampleIndex,
    int SampleRate,
    double ReadingRateHz,
    double WeightedCh1RmsLinear,
    double WeightedCh2RmsLinear,
    double AcCh1RmsLinear,
    double AcCh2RmsLinear,
    double AcCh1PeakLinear,
    double AcCh2PeakLinear,
    double RawCh1PeakLinear,
    double RawCh2PeakLinear,
    bool Ch1Clipping,
    bool Ch2Clipping,
    double Ch1DcFs,
    double Ch2DcFs,
    double BandpassCenterHz,
    double BandpassCh1RmsLinear,
    double BandpassCh2RmsLinear)
{
    public int ReportSampleCount { get; init; }
    public (double Ch1, double Ch2)? PanelPhaseDegrees { get; init; }
    public double BandpassCh2CenterHz { get; init; } = BandpassCenterHz;
    public BandpassInputSettings? BandpassInput { get; init; }
    public double BandpassRecordCh1Rms { get; init; } = double.NaN;
    public double BandpassRecordCh2Rms { get; init; } = double.NaN;
    public double FilteredCh1Peak { get; init; }
    public double FilteredCh2Peak { get; init; }
    // 第一声道残余与交流参考都以线性有效值计，直接相除得到无量纲的 THD+N 比。
    public double Ch1ThdnRatio => ResidualRatio(Ch1ResidualRms, AcCh1RmsLinear);
    // 第二声道使用自己的分子和分母，保留声道间电平及失真差异。
    public double Ch2ThdnRatio => ResidualRatio(Ch2ResidualRms, AcCh2RmsLinear);

    // 双路检波暂态可能造成比值略超 1。本读数只接受正交流参考且比值小于 1.05，轻微超出 1 时封顶。
    private static double ResidualRatio(double residual, double acRms)
    {
        // 分子是陷波后的 RMS，分母取分路处的交流参考 RMS，两者单位相消。改变主低通会影响分子，
        // 交流参考取自低通之前。1 和 1.05 是此表的显示与有效性边界；例如 1.02 显示 100%，1.05 返回 NaN。
        double ratio = residual / acRms;
        return acRms > 0 && ratio < 1.05 ? Math.Min(ratio, 1) : double.NaN;
    }

    public double Ch1ResidualRms { get; init; }
    public double Ch2ResidualRms { get; init; }

    // 将实时残余检波结果合并到已有频谱报告中，谐波列表仍来自其频域分析。
    public ThdNResult ApplyResidualLevels(ThdNResult spectrum) => spectrum with
    {
        Ch1 = MergeResidual(spectrum.Ch1, Ch1ResidualRms, Ch1ThdnRatio),
        Ch2 = MergeResidual(spectrum.Ch2, Ch2ResidualRms, Ch2ThdnRatio)
    };

    // 从 RMS 幅度比换算百分比与 dB，SINAD 取该比值 dB 的相反数。绝对残余以满幅正弦 RMS 为基准换算 dBFS。
    private static ThdNChannelResult MergeResidual(ThdNChannelResult spectrum, double residual, double ratio)
    {
        double ratioDb = double.IsFinite(ratio) ? SignalGeneratorDsp.LinearToDbfs(ratio) : double.NaN;
        return spectrum with
        {
            ThdNRatioPercent = 100 * ratio,
            ThdNRatioDb = ratioDb,
            SinadDb = -ratioDb,
            ThdNLevelDbfs = SignalGeneratorDsp.LinearToDbfs(residual * Math.Sqrt(2))
        };
    }

    public double FilteredCh1DcFs { get; init; }
    public double FilteredCh2DcFs { get; init; }
    public double Ch1ImdRatio { get; init; } = double.NaN;
    public double Ch2ImdRatio { get; init; } = double.NaN;

    /// <summary>当实时流开启了明确标准（SMPTE / DFD / CCIF / DIM）的 APxRT 时域 IMD 检波时，用连续检波结果覆盖快照 IMD。</summary>
    public ImdResult? ApplyRealtimeImd(ImdResult? snapshot, ImdMeasurementStandard standard)
    {
        if (standard is not (ImdMeasurementStandard.Smpte or ImdMeasurementStandard.Dfd
            or ImdMeasurementStandard.Ccif or ImdMeasurementStandard.DimEmulation))
            return snapshot;
        if (!double.IsFinite(Ch1ImdRatio) && !double.IsFinite(Ch2ImdRatio))
            return snapshot;
        string tag = standard switch
        {
            ImdMeasurementStandard.Smpte => "SMPTE (RT)",
            ImdMeasurementStandard.Dfd => "DFD (RT)",
            ImdMeasurementStandard.Ccif => "CCIF (RT)",
            ImdMeasurementStandard.DimEmulation => "DIM (RT)",
            _ => standard.ToString()
        };
        static ImdChannelResult Channel(double ratio, ImdChannelResult? fallback, string label) =>
            double.IsFinite(ratio)
                ? new ImdChannelResult(true, label, ratio * 100.0, 20.0 * Math.Log10(Math.Max(ratio, 1e-9)))
                : (fallback ?? new ImdChannelResult(false, label, double.NaN, double.NaN, "No valid RT IMD"));
        return new ImdResult(
            Channel(Ch1ImdRatio, snapshot?.Ch1, tag),
            Channel(Ch2ImdRatio, snapshot?.Ch2, tag));
    }
}
