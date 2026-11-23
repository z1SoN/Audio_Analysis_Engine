// 一次测量要跨几层传递含义。SignalPathConfig 描述打算怎样采，StereoAudioBlock 描述实际采到什么，后面的结果记录保存算出了什么。核对单位或通道含义时先看对应字段。
using System.Text.Json.Serialization;

namespace ApxRemix.Core.Models;

/// <summary>描述一个可用的本地音频设备（WASAPI 端点或 ASIO 驱动）。</summary>
public sealed record AudioDeviceInfo(
    string Id,
    string Name,
    AudioBackendMode Backend,
    int ChannelCount = 2);

/// <summary>Signal Path Setup（信号路径设置）配置模型。</summary>
// 均衡表随信号路径保存，输入和输出各用一组通道表。TrackFirst 让第二路跟随第一路的设置。
public sealed record PathEqCurve(bool Enabled, string Name, BenchTrace Table);
public sealed record PathEqSettings(bool TrackFirst, PathEqCurve First, PathEqCurve Second);
// 测量层以第一、第二路称呼波形，驱动则按物理通道编号。每个驱动有自己的映射，下次打开同一驱动时可以恢复所选端口。
public sealed record AsioChannelMapping(bool Automatic, int Inputs, int Outputs,
    int Input1, int Input2, int Output1, int Output2);

public sealed class SignalPathConfig
{
    // 四个 Ch 字段区分录音与播放的逻辑映射。WASAPI 及输入文件使用零起算索引，ASIO 的物理映射另存。
    public int AudioInputCh1 { get; set; }
    public int AudioInputCh2 { get; set; } = 1;
    public int AudioOutputCh1 { get; set; }
    public int AudioOutputCh2 { get; set; } = 1;
    public PathEqSettings? InputEq { get; set; }
    public PathEqSettings? OutputEq { get; set; }
    public AudioBackendMode BackendMode { get; set; } = AudioBackendMode.Wasapi;
    public AudioOutputConnector? OutputConnector { get; set; }
    [System.Text.Json.Serialization.JsonIgnore]
    // 旧工程可能只记录输入后端。输出连接器缺省时由该后端推导，显式选择的输出优先。
    public AudioOutputConnector EffectiveOutput => OutputConnector ?? DefaultOutput(BackendMode);
    public static AudioOutputConnector DefaultOutput(AudioBackendMode input) => input switch
    {
        AudioBackendMode.VirtualSimulator => AudioOutputConnector.VirtualLoopback,
        AudioBackendMode.Wasapi => AudioOutputConnector.Wasapi,
        AudioBackendMode.Asio => AudioOutputConnector.Asio,
        AudioBackendMode.WavFile => AudioOutputConnector.ExternalSource,
        _ => throw new ArgumentOutOfRangeException(nameof(input))
    };

    // 信号路径保存的是请求的采样率。经过设备协商后，采集块应带实际速率供 FFT、滤波和样本时间换算。
    public int SampleRate { get; set; } = 48000;
    // Fs 决定两个样本相隔多久，块长决定一次回调交付多少样本。例如 48 kHz 下 1,024 点约占 21.33 ms。
    public int BlockSize { get; set; } = 1024;

    // 设备 ID 指向具体 Windows 端点，ASIO 名称则用于打开驱动。WASAPI 独占标志只控制播放方向。
    public string? WasapiOutputDeviceId { get; set; }
    public bool WasapiExclusivePlayback { get; set; }
    public string? WasapiInputDeviceId { get; set; }
    public string? AsioDriverName { get; set; }

    // 自动映射按实际通道数配置。手动路由可只使用一路，索引随具体驱动保存在字典里，方便返回该设备时恢复。
    public int AsioInputCh1Index { get; set; } = 0;
    public int AsioInputCh2Index { get; set; } = 1;
    public int AsioOutputCh1Index { get; set; } = 0;
    public int AsioOutputCh2Index { get; set; } = 1;
    public bool? AsioAutoMapChannels { get; set; }
    public int? AsioInputChannelCount { get; set; }
    public int? AsioOutputChannelCount { get; set; }
    public Dictionary<string, AsioChannelMapping> AsioDeviceMappings { get; set; } = new(StringComparer.OrdinalIgnoreCase);

    public SignalPathConfig Snapshot()
    {
        // 快照会跨线程或跨步骤使用，另复制可变的设备映射字典和文件数组，避免后续编辑改写这两个集合。
        var copy = (SignalPathConfig)MemberwiseClone();
        copy.AsioDeviceMappings = new(AsioDeviceMappings, StringComparer.OrdinalIgnoreCase);
        copy.InputFileList = (string[])InputFileList.Clone();
        return copy;
    }

    // Older projects stored only physical indices. A repeated index is a single physical channel.
    [JsonIgnore] public bool UsesAutoAsioMapping => AsioAutoMapChannels ??
        (AsioInputCh1Index == 0 && AsioInputCh2Index == 1 && AsioOutputCh1Index == 0 && AsioOutputCh2Index == 1);

    internal (int First, int Second) ResolveAsioChannels(bool input, int available)
    {
        int first = input ? AsioInputCh1Index : AsioOutputCh1Index;
        int second = input ? AsioInputCh2Index : AsioOutputCh2Index;
        int? count = input ? AsioInputChannelCount : AsioOutputChannelCount;
        // 单路设备允许两个逻辑槽指向同一个物理端口；显式越界则报错，不能静默换端口，否则测得的会是另一路信号。
        if (UsesAutoAsioMapping) (first, second) = (0, available > 1 ? 1 : 0);
        else if (count == 1) second = first;
        if ((!UsesAutoAsioMapping && count is < 1 or > 2) || (uint)first >= (uint)available || (uint)second >= (uint)available)
            throw new InvalidOperationException($"ASIO {(input ? "input" : "output")} mapping is unavailable. The driver reports {available} channels.");
        return (first, second);
    }

    // 旧项目可只保留单个路径。文件列表有值时优先使用列表，统一采样率模式只改变样本的时间解释。
    public string? InputWavFilePath { get; set; }
    public string[] InputFileList { get; set; } = [];
    public Storage.FileInputRateMode FileRateMode { get; set; }
    public int FixedFileRate { get; set; } = 48000;
    public bool LoopInputWav { get; set; } = true;

    // 例如输入满量程标定为 2 Vrms，读到 −6.02 dBFS 的正弦对应约 1 Vrms。输出端的满量程可以不同，增益计算要同时知道两端基准。
    /// <summary>输入端满量程正弦有效值电压标定（0 dBFS = V_IN_FS_RMS）。</summary>
    public double FullScaleVoltageVrms { get; set; } = 2.0;

    /// <summary>输出端满量程正弦有效值电压标定（0 dBFS = V_OUT_FS_RMS）。</summary>
    public double OutputFullScaleVoltageVrms { get; set; } = 2.0;

    /// <summary>dBr 相对电平参考值（dBFS，由 Signal Path "Set dBr" 标定）。</summary>
    public SignalReferenceSettings? References { get; set; }
    // A、B 参考决定相对电平的零点。dBm 的负载电阻以 Ω 保存，经 Vᵣₘₛ²/R 算功率。麦克风电压灵敏度以 mV/Pa 表示。
    public double DbrReferenceDbfs { get; set; } = 0.0;
    public double DbrBReferenceDbfs { get; set; } = 0.0;
    public double DbmLoadOhms { get; set; } = 600;
    public double? Channel2MicSensitivityMvPerPa { get; set; }

    /// <summary>输入通道 Ch1/Ch2 灵敏度微调偏置（dB，对齐 APx500 Input Channel Gain）。</summary>
    // 每路微调增益以 dB 存储。应用到本路波形时用 10^(dB/20) 变成线性倍数，进而影响幅度类读数。
    public double InputCh1GainDb { get; set; } = 0.0;
    public double InputCh2GainDb { get; set; } = 0.0;

    /// <summary>被测件标称延迟（秒，对齐 APx500 Device Under Test Delay）。</summary>
    public double DutDelaySeconds { get; set; } = 0.0;

    [JsonIgnore] public VirtualDutConfig VirtualDut { get; set; } = new();
}

/// <summary>虚拟被测件（DUT）物理仿真模型参数。</summary>
// 这个 DUT 用增益、非线性、噪声等设置生成可重复的模拟响应；这些数值描述模型条件。
public sealed class VirtualDutConfig
{
    // 模型增益以 dB 保存，第二路另有电平偏差和延迟。延迟按采样点给出，除以采样率可得到秒。
    public double GainDb { get; set; } = 0.0;
    public double Channel2GainOffsetDb { get; set; } = -0.15;
    public double Channel2DelaySamples { get; set; } = 0.0;
    // 二、三次谐波设置按百分数表示，噪声底按 dBFS 表示。高低截止用 Hz，并有单独的带宽滤波开关。
    public double SecondHarmonicPercent { get; set; } = 0.015;
    public double ThirdHarmonicPercent { get; set; } = 0.008;
    public double NoiseFloorDbfs { get; set; } = -105.0;
    public double HighPassCutoffHz { get; set; } = 15.0;
    public double LowPassCutoffHz { get; set; } = 18000.0;
    public bool EnableBandwidthFilter { get; set; } = true;
}

/// <summary>信号发生器实时参数配置。</summary>
public sealed class GeneratorConfig
{
    // IsEnabled 表示当前是否发波，AutoOn 供测量任务临时开启输出。具体波形再决定哪些频率、相位和幅度字段参与生成。
    public bool IsEnabled { get; set; } = true;
    public bool AutoOn { get; set; } = true;
    public GeneratorWaveform Waveform { get; set; } = GeneratorWaveform.Sine;
    public double FrequencyHz { get; set; } = 1000.0;
    public double SecondaryFrequencyHz { get; set; } = 7000.0;
    // 电平用 dBFS 标度，偏置为有符号满量程比例。电平联动或第二路未设置偏置时，第二路沿用第一路的偏置。
    public double LevelDbfs { get; set; } = -6.0;
    public double DcOffsetFs { get; set; } = 0.0;
    public double? Channel2DcOffsetFs { get; set; }
    public bool EnableDither { get; set; }
    public bool LinkChannelLevels { get; set; } = true;
    public double Channel2LevelDbfs { get; set; } = -6.0;
    // 双音路由决定两个频率分量是否叠加，或由两路输出各取一个分量。另保存次音相对主音的 dB 比和第二路相位差（度）。
    public DualToneRoutingMode DualToneMode { get; set; } = DualToneRoutingMode.Sum;
    public double DualToneSecondaryRatioDb { get; set; } = 0.0;
    public double Channel2PhaseOffsetDeg { get; set; } = 0.0;
    public bool Channel1Enabled { get; set; } = true;
    public bool Channel2Enabled { get; set; } = true;
    public bool InvertChannel2 { get; set; }
    // 文件播放配置保存文件和起点帧，WaveformStartSample 以源文件的采样时间轴计数。波形专属与突发设置保存在独立记录中。
    public string? PlaybackWavFilePath { get; set; }
    public bool LoopPlaybackWav { get; set; } = true;
    public int WaveformPlayCount { get; set; } = 1;
    public long WaveformStartSample { get; set; }
    public GeneratorWaveSettings WaveSettings { get; set; } = new();
    public SineBurstSettings Burst { get; set; } = new();

    // 电平联动时右路派生自左路，解除联动才采用右路保存值；发生器使用最终电平来生成两路波形。
    public double EffectiveChannel2LevelDbfs => LinkChannelLevels ? LevelDbfs : Channel2LevelDbfs;

    // 保存独立的一份发生器参数，使扫描能临时改变激励、结束时恢复原状，且不会直接改写调用者的配置实例。
    public GeneratorConfig Clone() => new()
    {
        IsEnabled = IsEnabled,
        AutoOn = AutoOn,
        Waveform = Waveform,
        FrequencyHz = FrequencyHz,
        SecondaryFrequencyHz = SecondaryFrequencyHz,
        LevelDbfs = LevelDbfs,
        DcOffsetFs = DcOffsetFs,
        Channel2DcOffsetFs = Channel2DcOffsetFs,
        EnableDither = EnableDither,
        LinkChannelLevels = LinkChannelLevels,
        Channel2LevelDbfs = Channel2LevelDbfs,
        DualToneMode = DualToneMode,
        DualToneSecondaryRatioDb = DualToneSecondaryRatioDb,
        Channel2PhaseOffsetDeg = Channel2PhaseOffsetDeg,
        Channel1Enabled = Channel1Enabled,
        Channel2Enabled = Channel2Enabled,
        InvertChannel2 = InvertChannel2,
        PlaybackWavFilePath = PlaybackWavFilePath,
        LoopPlaybackWav = LoopPlaybackWav,
        WaveformPlayCount = WaveformPlayCount,
        WaveformStartSample = WaveformStartSample,
        WaveSettings = WaveSettings,
        Burst = Burst
    };
}

/// <summary>Bench Mode 分析仪滤波、计权、陷波调谐、带通选择性、IMD 模式、THD 参考与稳定判定配置。</summary>
// 滤波设置决定哪些频率计入总电平、哪些进入失真残余。给定相同 PCM，带宽和计权不同就可能报告不同读数。
public sealed class AnalyzerFilterConfig
{
    public AnalyzerFilterConfig Clone() => (AnalyzerFilterConfig)MemberwiseClone();
    // 高低通以 Hz 保存截止频率，低通类型决定滤波器。主路径计权和残余计权分别配置，改变一条路径会直接改变该路径的 RMS。
    public double HighPassHz { get; set; } = 20.0;
    public double LowPassHz { get; set; } = 20000.0;
    public LowPassFilterType LowPassType { get; set; } = LowPassFilterType.Butterworth8;
    public AnalyzerWeighting Weighting { get; set; } = AnalyzerWeighting.Flat;
    public AnalyzerWeighting ResidualWeighting { get; set; } = AnalyzerWeighting.Flat;
    public LevelDisplayUnit LevelUnit { get; set; } = LevelDisplayUnit.Dbfs;
    // 陷波抑制基波供失真测量，带通保留所选窄带供电平测量。中心以 Hz 保存，带通 Q 和倍频程选项描述相对带宽。
    public NotchTuningMode NotchTuningMode { get; set; } = NotchTuningMode.Generator;
    public double FixedNotchFrequencyHz { get; set; } = 1000.0;
    public NotchTuningMode BandpassTuneMode { get; set; } = NotchTuningMode.Generator;
    public double BandpassCenterHz { get; set; } = 0.0;
    public double BandpassQ { get; set; } = 10.0;
    public BandpassSelectivity BandpassSelectivity { get; set; } = BandpassSelectivity.ThirdOctave;
    // 互调模式决定查看哪些频率组，THD 参考模式指定用总 RMS 或基波 RMS 作分母。
    public ImdMeasurementStandard ImdStandard { get; set; } = ImdMeasurementStandard.Auto;
    public ImdMeasurementMode ImdMode { get; set; } = ImdMeasurementMode.D23;
    public ThdReferenceMode ThdReference { get; set; } = ThdReferenceMode.TotalRmsIec;
    // 稳定判定用一连串读数估计是否收敛。容差以 dB 表示，Points 是用于判定的读数数目。
    public SettlingAlgorithm SettlingMode { get; set; } = SettlingAlgorithm.Flat;
    public double SettlingToleranceDb { get; set; } = 0.05;
    public int SettlingPoints { get; set; } = 5;
    public AnalyzerReferenceChannel ReferenceChannel { get; set; } = AnalyzerReferenceChannel.None;
}

/// <summary>
/// 双通道实时音频采样块。
/// <paramref name="HasIndependentDualChannels"/> 明确标识左右声道是否来自真正独立的双路物理输入
/// （单声道 WAV 复制或 ASIO 同一物理输入映射到两路时为 false，阻止误用为双通道 FRF 参考）。
/// </summary>
// 设备回调交付同一时段的两路样本和采样信息。后端沿采样时钟拼接块，不能按界面刷新速度估算时间。
public sealed record StereoAudioBlock(
    double[] Left,
    double[] Right,
    int SampleRate,
    DateTime TimestampUtc,
    // ReferenceLeft 若存在应与采集块逐样本对齐。文件起点与发生器标记是块内偏移，ASIO 往返延迟也是样本数。
    double[]? ReferenceLeft = null,
    bool HasIndependentDualChannels = true,
    int[]? FileStartOffsets = null,
    // ExactPcm 标记文件输入是否具备整数码型来源。数字码参考另带有效位深和每路期望值，用于字级比较。
    bool ExactPcm = false,
    DigitalPatternReference? DigitalPattern = null,
    bool StartsAsioOneShot = false,
    int? AsioRoundTripLatency = null,
    int[]? GeneratorTriggerOffsets = null);

// 参考可只提供一路。Depth 规定有效数字位数，Constant 用于标记待比较的是固定码型。
public sealed record DigitalPatternReference(double[]? Left, double[]? Right, int Depth, bool Constant);
public enum DigitalBitsMode { ActiveBits, DataBits }

/// <summary>带单调样本序号与独立通道元数据的环形缓冲快照。</summary>
// 快照把一次读取的 PCM、有效通道和样本位置放在一起；实时仪表也有自己的读数位置，扫频用它判断结果是否属于新激励。
public sealed record AudioCaptureSnapshot(
    double[] Left,
    double[] Right,
    double[]? SyncReference,
    // EndSampleIndex 是所取记录的右侧边界，N 点窗口从 EndSampleIndex−N 开始。此坐标在当前会话内单调推进。
    long EndSampleIndex,
    int SampleRate,
    bool IsEngineRunning,
    bool HasDualChannel = true,
    ApxRealtimeReading? RealtimeReading = null,
    Dsp.StreamingBenchReading? BenchReading = null,
    // 快照内的文件首帧标记已换成会话内绝对样本位置，可以和窗口的首尾下标直接比较。
    long[]? GeneratorFileStarts = null,
    Dsp.DigitalWordReading? DigitalReading = null)
{
    public Dsp.StreamingBenchReading? PanelReading { get; init; }
}

/// <summary>Continuous AP reading with its own acquisition timestamp; never an output-clock reference.</summary>
public sealed record ApxRealtimeReading(Dsp.ApxPhaseReading Values, long EndSampleIndex, int SampleRate, double ReadingRateHz);

/// <summary>单通道 Level &amp; Gain 测量结果。</summary>
// 单路结果同时保留数字电平与物理单位，便于后续显示不同单位时沿用同一次测量。
public sealed record ChannelLevelResult(
    double RmsDbfs,
    double RmsVrms,
    double PeakDbfs,
    double GainDb,
    double MeasuredFrequencyHz,
    bool IsClipping,
    // 直流量用满量程比例保存。峰值、含直流的原始峰值与峰均比并列保存，避免显示时从 RMS 猜峰值。
    double DcLevelFs = 0.0,
    double CrestFactorDb = 0.0,
    double RawPeakDbfs = -180.0);

/// <summary>双通道 Level &amp; Gain 测量结果。</summary>
public sealed record LevelAndGainResult(
    ChannelLevelResult Ch1,
    ChannelLevelResult Ch2,
    // 通道相对相位以度保存，电平差以 dB 保存。只有两路数据满足对应测量条件时相对量才有意义。
    double InterChannelPhaseDeg,
    double ChannelGainDeltaDb);

/// <summary>单次谐波分量条目（H2~H20，受测量带宽限制）。</summary>
public sealed record HarmonicComponent(
    int Order,
    double FrequencyHz,
    double LevelDbfs,
    double RelativeToFundamentalDb);

/// <summary>单通道 THD+N 测量结果。</summary>
// THD+N 含去掉基波后留下的噪声与失真；谐波列表只描述与基频成整数倍关系的分量。保留参考电平才能解释百分比。
public sealed record ThdNChannelResult(
    double FundamentalFrequencyHz,
    double FundamentalLevelDbfs,
    // 比例与绝对残余电平各自保留。残余相对选定参考的比值可显示为百分数或 dB，绝对 THD+N 电平则以数字满量程为基准。
    double ThdNRatioPercent,
    double ThdNRatioDb,
    double SinadDb,
    double ThdRatioPercent,
    double ThdRatioDb,
    double NoiseFloorDbfs,
    IReadOnlyList<HarmonicComponent> Harmonics,
    bool HasValidSignal = true,
    string? InvalidReason = null,
    double ThdNLevelDbfs = double.NaN);

/// <summary>双通道 THD+N 测量结果。</summary>
public sealed record ThdNResult(
    ThdNChannelResult Ch1,
    ThdNChannelResult Ch2,
    double AnalysisBandwidthLowHz,
    double AnalysisBandwidthHighHz);

/// <summary>单通道互调失真（IMD）测量结果。</summary>
public sealed record ImdChannelResult(
    bool HasValidSignal,
    string StandardName,
    double ImdRatioPercent,
    double ImdRatioDb,
    string? InvalidReason = null);

/// <summary>双通道互调失真测量结果。</summary>
public sealed record ImdResult(ImdChannelResult Ch1, ImdChannelResult Ch2);

/// <summary>窄带带通电平双通道测量结果。</summary>
public sealed record BandpassLevelResult(
    double CenterFrequencyHz,
    double Ch1LevelDbfs,
    double Ch2LevelDbfs)
{
    public double Ch2CenterFrequencyHz { get; init; } = CenterFrequencyHz;
}

/// <summary>Bench Mode 读数稳定判定状态。</summary>
public sealed record SettlingStatus(
    bool IsSettled,
    int StableFrameCount,
    double LastDeltaDb,
    string Label);

/// <summary>Bench Mode 单帧综合仪表快照。</summary>
// 把同一次更新中的电平、失真和已启用的可选仪表一起交付。扫频记录某一点时，可直接取这组对应读数。
public sealed record BenchMeterSnapshot(
    LevelAndGainResult LevelAndGain,
    ThdNResult ThdPlusN,
    double WeightedCh1RmsDbfs,
    double WeightedCh2RmsDbfs,
    BandpassLevelResult? BandpassLevel = null,
    ImdResult? Imd = null)
{
    public (double Ch1, double Ch2)? PanelPhaseDegrees { get; init; }
}

/// <summary>FFT 派生分析结果。</summary>
public sealed record FftDerivedMetrics(
    // 带内积分汇总指定频段功率。噪声幅度密度以每 √Hz 报告，SFDR 表示最大寄生分量相对主信号的量级。
    double IntegratedBandPowerDbfs,
    double NoiseDensityDbfsPerSqrtHz,
    double PeakToAverageDb,
    double SfdrDbc);

/// <summary>FFT 叠加对比参考曲线数据集。</summary>
public sealed record FftTraceDataset(
    string Label,
    double[] FrequenciesHz,
    double[] MagnitudeDbfs,
    string ColorHex);

/// <summary>Measurement Recorder 多指标时间序列采样点（携带指标类型以杜绝混单位；无效样本严格用 NaN 表示）。</summary>
public sealed record RecorderSamplePoint(
    // 时间字段为相对本轮起点的秒数。两路通用值的单位由 MetricKind 决定，多指标模式另带每路的指标数组。
    double ElapsedSeconds,
    double Ch1Value,
    double Ch2Value,
    double Ch1ThdNDb = double.NaN,
    double Ch1FrequencyHz = double.NaN,
    int RunIndex = 1,
    bool IsCh1Valid = true,
    bool IsCh2Valid = true,
    RecorderMetricKind MetricKind = RecorderMetricKind.RmsLevelDbfs,
    double[]? MetricsCh1 = null, double[]? MetricsCh2 = null);

/// <summary>FFT 频谱峰值标注点。</summary>
public sealed record SpectrumPeakMarker(
    double FrequencyHz,
    double MagnitudeDbfs,
    string Label);

/// <summary>Signal Analyzer（FFT 频谱分析）单帧结果。</summary>
// 频谱的横轴是频率，同一记录还可以得到时域 RMS、噪声或峰标记。保留频谱元数据可说明这些量各自的带宽和标尺。
public sealed record SpectrumAnalysisResult(
    double[] FrequenciesHz,
    double[] Ch1MagnitudeDbfs,
    double[] Ch2MagnitudeDbfs,
    IReadOnlyList<SpectrumPeakMarker> TopPeaksCh1,
    int FftSize,
    int SampleRate,
    FftWindowType WindowType,
    // FFT 频点间距是 Fs/N。窗函数改变泄漏与有效噪声带宽，但不会改变这个频点间距。
    double FrequencyResolutionHz,
    FftDerivedMetrics? DerivedCh1 = null)
{
    // Coherent peak amplitude squared before applying the dB display floor.
    // 保留线性峰值幅度平方，可跨频点做功率相关计算。用于显示的 dB 下限不应在这些线性值里产生一层人工噪底。
    public double[] Ch1PeakPower { get; init; } = [];
    public double[] Ch2PeakPower { get; init; } = [];
}

/// <summary>
/// Stepped Sweep / Continuous Sweep / Acoustic Response / Transfer Function 通用测点模型。
/// <para>
/// - <see cref="IsValid"/> 为 false 时（如 FRF 无激励频段），图表在此处断线留空、CSV 标 N/A；
/// - <see cref="ActualGeneratorLevelDbfs"/>、<see cref="RegulationStatus"/> 与 <see cref="RegulationErrorDb"/> 精确记录逐点闭环调节的实际状态。
/// </para>
/// </summary>
[JsonNumberHandling(JsonNumberHandling.AllowNamedFloatingPointLiterals)]
// 一步扫频既有设定横坐标，也有实际激励、两路响应、有效性和相位参考，后续汇总时要能辨认未达标的点。
public sealed record FrequencyResponsePoint(
    double FrequencyHz,
    double Ch1LevelDbfs,
    double Ch2LevelDbfs,
    double Ch1GainDb,
    double Ch2GainDb,
    double Ch1RelativeDb,
    double Ch2RelativeDb,
    double? Ch1PhaseDeg,
    double? Ch2PhaseDeg,
    double? MeasuredPhaseDeg = null,
    // FrequencyHz 保存测点频率，XValue 提供实际横轴数据。电平、直流偏置和相位扫描时，横轴单位随主轴类型改变。
    double XValue = 0.0,
    double Ch1ThdNPercent = double.NaN,
    double Ch1ThdNDb = double.NaN,
    // 相干度是参考和响应的频域相关指标，通常在 0 到 1 之间。空值表示当前结果缺少这项估计。
    double? Coherence = null,
    bool IsValid = true,
    // 逐点调节可能使真实输出偏离初始请求。保留最后设置的发生器电平和是否找到目标，才能辨认未达标点。
    double ActualGeneratorLevelDbfs = double.NaN,
    RegulationOutcome RegulationStatus = RegulationOutcome.NotRequested,
    double RegulationErrorDb = 0.0,
    double Ch2ThdNPercent = double.NaN,
    double Ch2ThdNDb = double.NaN,
    double ExcitationToneDbfs = double.NaN,
    [property: JsonIgnore] BenchMeterSnapshot? MeterReading = null,
    double? InputRateHz = null, double? Ch1ErrorRate = null, double? Ch2ErrorRate = null);

/// <summary>嵌套扫频单条子曲线数据集。</summary>
public sealed record SweepSeriesDataset(
    string Label,
    IReadOnlyList<FrequencyResponsePoint> Points,
    string ColorHex);

/// <summary>Stepped Sweep 扩展配置（含独立输入/输出物理电压标定与逐点调节策略）。</summary>
// 这些参数定义如何做一次步进实验，依次决定改变哪个量、每点如何驱动、何时认为稳定、记录哪些输出。
public sealed class FrequencySweepConfig
{
    // 主轴改变发生器频率、电平、偏置或相位等物理量。SpacingMode 规定取点规则，YMetric 只指定要从测点取哪个输出指标。
    public SweepAxisType AxisType { get; set; } = SweepAxisType.Frequency;
    public SweepSpacingMode SpacingMode { get; set; } = SweepSpacingMode.Logarithmic;
    public SweepYMetric YMetric { get; set; } = SweepYMetric.RelativeDb;

    public double StartFrequencyHz { get; set; } = 20.0;
    public double StopFrequencyHz { get; set; } = 20000.0;
    // 电平轴用 dBFS，偏置轴用满量程比例，相位轴用度。幅度扫描通常保持音调固定，因此另存该固定频率。
    public double StartLevelDbfs { get; set; } = -60.0;
    public double StopLevelDbfs { get; set; } = -3.0;
    public double FixedFrequencyForLevelSweepHz { get; set; } = 1000.0;
    public double StartOffsetFs { get; set; } = -0.2;
    public double StopOffsetFs { get; set; } = 0.2;
    public double StartPhaseDeg { get; set; } = -180;
    public double StopPhaseDeg { get; set; } = 180;
    public SweepToneSource ToneSource { get; set; }
    public double CompanionToneHz { get; set; } = 7000;
    public double DualToneRatioDb { get; set; }

    // 每点在改变设置后需要时间传播和稳定，才截取读数。等待、读数窗口和点间隔均以毫秒给出。
    public int PointCount { get; set; } = 31;
    public double SweepLevelDbfs { get; set; } = -6.0;
    public int PreSweepDelayMs { get; set; } = 0;
    public int SettlingTimeMs { get; set; } = 50;
    public SweepSettlingConfig Settling { get; set; } = new();
    public bool CaptureBenchMeters { get; set; }
    public bool UseClockedBandpass { get; set; }
    public bool SettleBandpassLevel { get; set; }
    public bool UseApxRealtimeReadings { get; set; }
    public int ReadingWindowMs { get; set; }
    public int PointIntervalMs { get; set; }
    // 外部驱动改变频率时，用指定输入通道跟踪实测频率。相对或线性结果另需参考点、统计频宽或电平区间。
    public InputChannelSelection TrackingChannel { get; set; } = InputChannelSelection.Channel1;
    public bool UseExternalFrequency { get; set; }
    public SweepYMetric? SecondaryMetric { get; set; }
    public double ReferenceFrequencyHz { get; set; } = 1000.0;
    public decimal DeviationMinHz { get; set; } = 20;
    public decimal DeviationMaxHz { get; set; } = 20000;
    public decimal LinearityReferenceDbfs { get; set; }
    public decimal RelativeMinHz { get; set; } = 20;
    public decimal RelativeMaxHz { get; set; } = 20000;
    public bool CenterRelativeInLimits { get; set; }
    public bool CenterLinearityInLimits { get; set; }
    public decimal LinearityMinDbfs { get; set; } = -120;
    public decimal LinearityMaxDbfs { get; set; }

    // 相位需选明相对哪一路信号。低于有效门限或迟迟未稳定的点受读数门限和超时规则限制。
    public SweepPhaseReferenceMode PhaseReferenceMode { get; set; } = SweepPhaseReferenceMode.Auto;
    public double MinValidSignalDbfs { get; set; } = -100.0;
    public int StepTimeoutMs { get; set; } = 2500;

    /// <summary>扫频内嵌 THD+N 分析共用的分析仪带宽与计权设置（null 表示默认 20Hz–20kHz Flat）。</summary>
    public AnalyzerFilterConfig? AnalyzerFilter { get; set; }

    /// <summary>输入与输出满量程电压标定（用于统一物理电压增益 Gain = dBFS_in − dBFS_out + 20·log10(V_in / V_out)）。</summary>
    public double InputFullScaleVoltageVrms { get; set; } = 2.0;
    public double OutputFullScaleVoltageVrms { get; set; } = 2.0;

    // 自定义主轴点按主轴单位保存。二级扫描的数值另按 SecondarySource 的单位解释，电平二级扫描使用 NestedLevelsDbfs。
    public double[]? CustomPoints { get; set; }

    /// <summary>逐点闭环电平调节配置与未达标处理策略。</summary>
    // 闭环目标用输入 dBFS 表示，输出只能在允许的发生器电平范围变化。容差和失败策略共同决定该点如何收尾。
    public bool EnableTargetRegulation { get; set; }
    public double TargetRegulatedLevelDbfs { get; set; } = -10.0;
    public double RegulationToleranceDb { get; set; } = 0.15;
    public double RegulationMaxGeneratorDbfs { get; set; } = -0.5;
    public double RegulationMinGeneratorDbfs { get; set; } = -90.0;
    public bool AbortOnRegulationFailure { get; set; }

    public bool EnableNestedSweep { get; set; }
    public BenchSweepSource SecondarySource { get; set; } = BenchSweepSource.GeneratorLevel;
    public double[]? SecondaryValues { get; set; }
    internal FrequencySweepConfig CopyForNestedPass() => (FrequencySweepConfig)MemberwiseClone();
    public double[] NestedLevelsDbfs { get; set; } = [-24.0, -12.0, -3.0];
}

/// <summary>Frequency Response / Stepped Sweep 汇总报告。</summary>
[JsonNumberHandling(JsonNumberHandling.AllowNamedFloatingPointLiterals)]
public sealed record FrequencyResponseSummary(
    IReadOnlyList<FrequencyResponsePoint> Points,
    double MinDeviationDb,
    double MaxDeviationDb,
    double FlatnessSpanDb,
    double? LowMinus3DbHz,
    double? HighMinus3DbHz,
    SweepPhaseReferenceMode EffectivePhaseReference = SweepPhaseReferenceMode.None,
    string PhaseReferenceDescription = "Phase N/A (No Sync Reference)",
    IReadOnlyList<SweepSeriesDataset>? NestedSeries = null,
    int RegulationUnmetCount = 0);

public sealed record CapturedSweep(string Label, DateTime TimeUtc, FrequencySweepConfig Settings, FrequencyResponseSummary Results)
{
    // 保存明确标志供后续识别来源，兼容旧工程只在标签中写下 Sequence Chirp 的记录。
    public bool IsSequenceResponse { get; init; }
    [JsonIgnore] public bool FromSequence => IsSequenceResponse || Label?.Contains(" · Sequence chirp · ", StringComparison.Ordinal) == true;
}

/// <summary>Continuous Sweep（Chirp 反卷积）配置参数。</summary>
// 快扫把多个离散音预排成一段波形，每点包括切换、等待和测量区间。这组设置同时决定驱动和回读切片。
public sealed record FastSweepOptions
{
    public SweepSpacingMode Spacing { get; init; } = SweepSpacingMode.Logarithmic;
    public int Points { get; init; } = 24;
    public string TableHz { get; init; } = "100, 200, 500, 1000, 2000, 5000";
    public bool AutoTime { get; init; } = true;
    public bool MeasureThdn { get; init; } = true;
    // 时间窗既能按周期定义，也能用毫秒指定。例如 100 Hz 的 2 周期需要 20 ms，1 kHz 的同一设置只需 2 ms。
    public double TransitionCycles { get; init; } = 1;
    public double DiscardCycles { get; init; } = 2;
    public double MeasureCycles { get; init; } = 10;
    public double TransitionMs { get; init; } = 1;
    public double DiscardMs { get; init; } = 2;
    public double MeasureMs { get; init; } = 12;
}

public enum ChirpCrosstalkMode { None, OneChannelDriven, OneChannelUndriven, HighSpeed }
public enum SweepGeneratorEqMode { None, Relative, Absolute }
public sealed record SweepChannelSettings
{
    public bool TrackFirst { get; init; } = true;
    public double LevelCh2Dbfs { get; init; } = -20;
    // 直流分量按数字满量程比例定义。EQ 保存频率表与相对或绝对模式，后续生成扫频时按该模式解释表值。
    public double DcCh1 { get; init; }
    public double DcCh2 { get; init; }
    public SweepGeneratorEqMode EqMode { get; init; }
    public string EqName { get; init; } = "Sweep EQ";
    public BenchTrace? EqTable { get; init; }
}

public sealed class ContinuousSweepConfig
{
    // 开环可先导出激励，再从外部录音分析，闭环可由当前引擎直接发波和采集。导频和 Fast Sweep 设置分别控制相关实验条件。
    public bool OpenLoop { get; set; }
    public SweepChannelSettings Channels { get; set; } = new();
    public ChirpCrosstalkMode CrosstalkMode { get; set; }
    public PilotSweepOptions Pilot { get; set; } = new();
    public FastSweepOptions FastSweep { get; set; } = new();
    public ContinuousSweepConfig Clone() => (ContinuousSweepConfig)MemberwiseClone();

    public double StartFrequencyHz { get; set; } = 20.0;
    public double StopFrequencyHz { get; set; } = 20000.0;
    // 扫过起止频率的时长用秒，前置段与响应尾部用毫秒。尾部保留激励结束后仍可能存在的延迟与衰减。
    public double DurationSeconds { get; set; } = 1.0;
    public double LevelDbfs { get; set; } = -6.0;
    public ContinuousChirpType ChirpType { get; set; } = ContinuousChirpType.ApChirp;
    public bool ExtractHarmonics { get; set; } = true;
    public int PreExcitationMs { get; set; } = 100;
    public int ResponseTailMs { get; set; } = 350;
    public int OutputPoints { get; set; } = 200;
    public ContinuousPhaseMode PhaseMode { get; set; } = ContinuousPhaseMode.RelativeToCh1;
    public decimal? ExcessMinHz { get; set; }
    public decimal? ExcessMaxHz { get; set; }
    public double InputFullScaleVoltageVrms { get; set; } = 2.0;
    public double OutputFullScaleVoltageVrms { get; set; } = 2.0;
}

// 开环测量可在扫频前发出导频。它的门限、时长和间隔约定录音端如何从一长串样本中找到正式激励。
public sealed record PilotSweepOptions
{
    public bool Enabled { get; init; }
    public double Frequency { get; init; } = 1000;
    public double LevelDbfs { get; init; } = -20;
    public double DurationMs { get; init; } = 200;
    public double SilenceMs { get; init; } = 100;
    public double AdditionalSilenceMs { get; init; }
    public int Iterations { get; init; } = 1;
    public bool CorrectFrequency { get; init; } = true;
    // 触发门限以 dBFS 保存，超时用秒。触发通道选择决定从哪个返回信号寻找导频。
    public double ThresholdDbfs { get; init; } = -60;
    public double TimeoutSeconds { get; init; } = 30;
    public string TriggerChannel { get; init; } = "Auto";
}

/// <summary>Continuous Sweep（ApChirp / LogChirp 反卷积 + Farina 谐波分离 + ETC）完整分析结果。</summary>
public sealed record ContinuousSweepResult(
    IReadOnlyList<FrequencyResponsePoint> ResponsePoints,
    double[] ImpulseResponse,
    // 冲激横轴以毫秒表示，估计延迟同时提供毫秒和样本数。两种单位可用采样率换算，例如 48 kHz 下 48 个样本对应 1 ms。
    double[] ImpulseTimeMs,
    double EstimatedDelayMs,
    int EstimatedDelaySamples,
    double PeakImpulseValue,
    int SampleRate,
    bool WasClipping = false,
    string SourceDescription = "",
    // 根据激励和分析条件记录可用的谐波、THD、残余、能量时间曲线等派生数据。空的可选项表示此次分析无此结果。
    IReadOnlyList<FrequencyResponsePoint>? Harmonic2Points = null,
    IReadOnlyList<FrequencyResponsePoint>? Harmonic3Points = null,
    IReadOnlyList<FrequencyResponsePoint>? ThdVsFreqPoints = null,
    double[]? EnergyTimeCurveDb = null,
    IReadOnlyList<FrequencyResponsePoint>? FastThdnPoints = null,
    double[]? Acquired = null, double[]? Reference = null,
    ChirpCrosstalkMode CrosstalkMode = ChirpCrosstalkMode.None, BenchTrace? Crosstalk = null, double[]? LeakageWaveform = null,
    ApxRemix.Core.Dsp.RubAndBuzzSweep? RubAndBuzz = null,
    Dictionary<int,BenchTrace>? ProductLevels = null, BenchTrace? GroupDelay = null, double? PilotShiftPercent = null,
    SweepSpectrum? MeasuredSpectrum = null);

public sealed record SweepSpectrum(double[] Frequencies, double[] GainDb, double[] LevelDbfs,
    double[] RelativeDb, double[] PhaseDeg);

/// <summary>Acoustic Response（声学响应）配置。</summary>
// 声学计算还需要麦克风灵敏度和时间窗。前者把电压换成声压，后者决定哪些迟到的反射进入频响。
public sealed class AcousticResponseConfig
{
    public double RubBuzzHighPassFactor {get;set;}=10;
    public AcousticResponseConfig Clone() => (AcousticResponseConfig)MemberwiseClone();
    public ContinuousSweepConfig Chirp { get; set; } = new();
    // 灵敏度 10 mV/Pa 表示 10 mV 对应 1 Pa。换算前需统一电压单位，再用选定输入路的灵敏度换成声压。
    public double MicSensitivityMvPerPa { get; set; } = 10.0;
    public int Averages { get; set; } = 1;
    // 自动起点以检测到的冲激峰为基准向前留出窗口，手动起点直接用毫秒指定。可选终点或峰后的窗口宽度控制保留的反射范围。
    public bool AutoWindowStart { get; set; } = true;
    public double ManualWindowStartMs { get; set; }
    public bool EnableIrWindow { get; set; } = true;
    public double WindowLeftMs { get; set; } = 1.5;
    public double WindowRightMs { get; set; } = 5.0;
    public double? WindowEndMs { get; set; }
    public InputChannelSelection InputChannel { get; set; } = InputChannelSelection.Channel1;
}

/// <summary>Acoustic Response 声学分析结果（记录由参考激励实测得到的真实参考电平，不受界面发生器滑块污染）。</summary>
public sealed record AcousticResponseResult(
    // 同时保留加窗和未加窗的声压级。对照两条响应可看出较晚的反射对频域结果的影响。
    IReadOnlyList<FrequencyResponsePoint> WindowedSplPoints,
    IReadOnlyList<FrequencyResponsePoint> UnwindowedSplPoints,
    double[] ImpulseResponse,
    double[] ImpulseTimeMs,
    double DirectPeakTimeMs,
    double WindowStartMs,
    double WindowEndMs,
    double EstimatedDelayMs,
    double AveragePassbandSplDb,
    bool WasClipping = false,
    string SourceDescription = "",
    double ActualReferenceLevelDbfs = double.NaN,
    ContinuousSweepResult? Deconvolution = null,
    double[]? WindowWeights = null);

/// <summary>Transfer Function（Welch 双通道传递函数）配置。</summary>
public sealed record TransferFileSettings
{
    public GeneratorWaveform Stimulus { get; init; } = GeneratorWaveform.WhiteNoise;
    public string StimulusFile { get; init; } = "";
    // 发生器电平以 dBFS 表示，直流偏置用满量程比例。Drive 和两个通道开关决定是否由本机向对应通道发波。
    public double StimulusLevel { get; init; } = -20;
    public double DcOffset { get; init; }
    public bool Drive { get; init; } = true;
    public bool DriveLeft { get; init; } = true;
    public bool DriveRight { get; init; } = true;
    public string Trigger { get; init; } = "Match";
    public InputChannelSelection TriggerChannel { get; init; }
    public double TriggerLevel { get; init; } = -30;
    public bool Rising { get; init; } = true;
    // Match 门限约束波形匹配程度，TimeoutSec 是寻找有效起点的最长秒数。起点确定后才进行对齐和后续分析。
    public double MatchThreshold { get; init; } = 0.5;
    public double TimeoutSec { get; init; } = 30;
    public string Align { get; init; } = "None";
    public bool SaveAcquisition { get; init; }
    public bool SaveImpulse { get; init; }
    public string AcquisitionPath { get; init; } = "";
    public string ImpulsePath { get; init; } = "";
    public bool ReplaceFiles { get; init; }
}

// 传递函数要明确参考和响应的路由，FFT、重叠与平均决定每次估计用了多少共同时间段。
public sealed class TransferFunctionConfig
{
    public TransferFunctionCalculationMode CalculationMode { get; set; } = TransferFunctionCalculationMode.H1;
    // 传递函数参考和响应可来自不同量程。先以各自满量程 Vrms 建立电压尺度，得到可比的物理幅值关系。
    public double ReferenceFullScaleVoltageVrms { get; set; } = 1.0;
    public double MeasurementFullScaleVoltageVrms { get; set; } = 1.0;
    public int FftSize { get; set; } = 4096;
    // 重叠度用比例保存。例如 4096 点的 0.5 重叠使相邻记录起点差 2048 点，再用窗函数及平均模式估计频谱。
    public double OverlapRatio { get; set; } = 0.5;
    public FftWindowType WindowType { get; set; } = FftWindowType.Hann;
    public int Averages { get; set; }
    // 先用绝对门限检查通道是否有信号，再结合相对谱峰范围和绝对功率下限剔除参考不足的频点，防止除以过小参考谱。
    public double MinReferencePowerDbfs { get; set; } = -90.0;
    public double DynamicRangeBelowPeakDb { get; set; } = 45.0;
    public int OutputPoints { get; set; } = 200;
}

/// <summary>Transfer Function（FRF 幅相 + 相干度 γ² + 系统延迟）分析结果（明确区分参考有效、输出有效与延迟有效）。</summary>
public sealed record TransferSpectra(double[] Frequencies, double[] InputFftDbfs, double[] ReferenceFftDbfs,
    double[] InputPsd, double[] ReferencePsd, double[] LagMs, double[] Correlation);

public sealed record TransferFunctionResult(
    // 参考过弱、响应不可用、延迟不可用是不同状态，调用方按自身要显示的结果检查对应标志。
    bool HasValidReference,
    string? InvalidReason,
    IReadOnlyList<FrequencyResponsePoint> FrfPoints,
    double MeanCoherence,
    double EstimatedDelayMs,
    double EstimatedDelaySamples,
    int ValidBandCount = 0,
    int TotalBandCount = 0,
    bool IsNarrowbandReference = false,
    string ReferenceRoutingDescription = "",
    bool HasValidOutput = true,
    bool HasValidDelay = true,
    string? OutputStatusNote = null,
    double[]? ImpulseResponse = null,
    TransferSpectra? Spectra = null);

/// <summary>Sequence Mode 单步自动化测量结果条目。</summary>
public sealed record SequenceStepResult(
    NavigationViewKind Measurement,
    string DisplayName,
    bool Passed,
    string SummaryText,
    string LimitDescription,
    bool HasLimit = true,
    bool AcquisitionFailed = false,
    SequenceResultDetail[]? Details = null)
{
    public string Verdict
    {
        get
        {
            // 先区分采集出错、未生成结果和无可比较数据，再给有判据的读数判定通过或失败，防止把缺测当成合格。
            if (AcquisitionFailed) return "Error";
            if (Details is { Length: 0 }) return "No results";
            if (Details is null && !HasLimit) return "Measured";
            if (Details?.Any(d => d.Verdict == "No data") == true && Details.All(d => d.Verdict != "FAIL"))
                return "No data";
            return Passed ? "PASS" : "FAIL";
        }
    }
}

public sealed record SequenceResultDetail(string Name, string Reading, string Unit, string LimitDescription, bool Passed, bool HasLimit)
{
    // 比对层用这个标志说明有没有落在限值判据范围内的可比较读数。缺少可比较数据的有判据条目显示 No data。
    public bool HasComparableData { get; init; } = true;
    public string Verdict => HasLimit && !HasComparableData ? "No data" : Passed ? "PASS" : "FAIL";
    public string DisplayReading => string.IsNullOrEmpty(Unit) ? Reading : $"{Reading} {Unit}";
}

public sealed record SequenceResultSettings(string Name, bool Included, BenchGraphLimitSettings? Limits);
public sealed record SequenceStepSettings(NavigationViewKind Measurement, bool Included, List<SequenceResultSettings>? Results = null,
    bool Present = true, string? CustomName = null);
public sealed record SequenceDisplaySettings(string LevelResult, string ThdResult, string SignalResult,
    string FrequencyResult, decimal DeviationFromHz, decimal DeviationToHz, bool ShowSecondChannel,
    InputChannelSelection ProductChannel);

public sealed record BenchMeterSettings(BenchMeterKind Kind, LevelDisplayUnit Level, DistortionDisplayUnit Ratio, string FrequencyUnit,
    bool ShowBars = true, bool AutoScale = true, bool LogScale = false, double RangeMin = -80, double RangeMax = 5,
    double? Lower1 = null, double? Upper1 = null, double? Lower2 = null, double? Upper2 = null);

/// <summary>可持久化保存与重开的 APx Remix 工程文件快照（向后兼容旧版 JSON）。</summary>
// 持久化工程要保留激励、路由、标定、分析和显示条件；否则同样一份 PCM 下次打开可能以另一套条件计算。
public sealed class ProjectStateFile
{
    public string SchemaVersion { get; set; } = "2.3";
    public DateTime SavedAtUtc { get; set; } = DateTime.UtcNow;
    public AppOperationMode OperationMode { get; set; } = AppOperationMode.Bench;
    public BandpassSelectivity CatalogBandpassSelectivity { get; set; } = BandpassSelectivity.TwentyFourthOctave;
    public BandpassSelectivity SequenceSweepBandpassWidth { get; set; } = BandpassSelectivity.TwentyFourthOctave;
    public NotchTuningMode CatalogBandpassTuning { get; set; } = NotchTuningMode.Generator;
    public decimal CatalogBandpassFixedHz { get; set; } = 1000m;
    public InputChannelSelection CatalogLevelRatioReference { get; set; } = InputChannelSelection.Channel1;
    public decimal CatalogNoiseDelaySeconds { get; set; } = 0.30m;
    public decimal CatalogNoiseAcquisitionSeconds { get; set; } = 0.25m;
    public bool CatalogNoiseUsePathHighpass { get; set; }
    public bool CatalogNoiseUsePathLowpass { get; set; }
    // 保存 Sequence 选中步骤及每项限值；上次报告也可随工程保存，重新打开时仍只是历史读数。
    public List<SequenceStepSettings>? SequenceSteps { get; set; }
    public bool SequencePathIncluded { get; set; } = true;
    public List<SequenceStepResult>? SequenceReport { get; set; }
    public NavigationViewKind SequenceView { get; set; } = NavigationViewKind.LevelAndGain;
    public SequenceDisplaySettings? SequenceDisplay { get; set; }
    public decimal SequenceCrosstalkHz { get; set; } = 10000;
    public decimal SequenceRouteDelayMs { get; set; } = 200;
    public InputChannelSelection SequencePhaseReference { get; set; } = InputChannelSelection.Channel1;
    public SequencePhaseMeterRange SequencePhaseRange { get; set; } = SequencePhaseMeterRange.Minus90To270;
    public AnalyzerWeighting SequenceNoiseWeighting { get; set; } = AnalyzerWeighting.Flat;
    public bool IsDarkTheme { get; set; }
    // 测量如何连线、如何设置激励和分析由这些子模型恢复。界面显示、历史数据集与测量配置另有自己的状态字段。
    public SignalPathConfig SignalPath { get; set; } = new();
    public GeneratorConfig Generator { get; set; } = new();
    public AnalyzerFilterConfig Analyzer { get; set; } = new();
    public FrequencySweepConfig Sweep { get; set; } = new();
    public bool SweepUseRelativeDb { get; set; } = true;
    public bool SweepShowPhase { get; set; }
    public bool SweepShowChannel2 { get; set; } = true;
    public bool SweepRepeat { get; set; }
    public bool SweepAppend { get; set; }
    public List<CapturedSweep> SweepDatasets { get; set; } = new();
    public int ActiveSweepDataset { get; set; } = -1;
    public string GeneratorDisplayUnit { get; set; } = "dBFS";
    public Dictionary<string,string>? AccessoryPreset { get; set; }
    public MonitorDisplaySettings? MonitorDisplay { get; set; }
    public bool BenchMonitorsEnabled { get; set; } = true;
    public BenchMeasurementTab ActiveBenchTab { get; set; }
    public Dictionary<string, BenchWorkspaceSettings>? BenchWorkspaces { get; set; }
    public DigitalBitsMode BitsMeterMode { get; set; }
    public List<BenchMeterSettings>? BenchMeterSettings { get; set; }
    public List<BenchMeterKind> ActiveBenchMeters { get; set; } = new();

    // FFT 参数（含触发模式/门限/延时/超时/连续状态，旧文件缺失时走默认值）
    public int FftSize { get; set; } = 16384;
    public bool FftAcquisitionAuto { get; set; } = true;
    public int FftAcquisitionSamples { get; set; } = 16384;
    public FftWindowType FftWindow { get; set; } = FftWindowType.BlackmanHarris4;
    public bool FftMinHoldEnabled { get; set; }
    public bool FftPeakHoldEnabled { get; set; }
    public FftAverageMode FftAverageMode { get; set; } = FftAverageMode.Exponential;
    public int FftAverages { get; set; } = 4;
    // 触发源与触发通道选择在哪里找边沿。门限是线性满量程比例，延时以样本计，超时以秒计。
    public FftTriggerMode FftTriggerMode { get; set; } = FftTriggerMode.FreeRun;
    public InputChannelSelection FftTriggerChannel { get; set; } = InputChannelSelection.Channel1;
    public double FftTriggerThresholdLinear { get; set; } = 0.05;
    public int FftTriggerDelaySamples { get; set; } = 0;
    public double FftTriggerTimeoutSeconds { get; set; } = 10.0;
    public bool IsFftContinuous { get; set; } = true;

    // Continuous / Acoustic / Transfer / Recorder 完整操作设置
    public ContinuousSweepConfig ContinuousSweep { get; set; } = new();
    public ChirpExecutionMode ContinuousExecutionMode { get; set; } = ChirpExecutionMode.LiveEngine;
    public InputChannelSelection ContinuousInputChannel { get; set; } = InputChannelSelection.Channel1;
    public bool ContinuousOutputCh1 { get; set; } = true;
    public bool ContinuousOutputCh2 { get; set; } = true;
    // 开环参考和响应分别指向文件，供重新打开工程时找回同一对波形。活动输出开关用于本机驱动路径。
    public string? OpenLoopChirpWavPath { get; set; }
    public string? OpenLoopResponseWavPath { get; set; }

    public string? AcousticReferenceWavPath { get; set; }
    public string? AcousticResponseWavPath { get; set; }
    public AcousticResponseConfig AcousticResponse { get; set; } = new();
    public ChirpExecutionMode AcousticExecutionMode { get; set; } = ChirpExecutionMode.LiveEngine;
    public bool AcousticOutputCh1 { get; set; } = true;
    public bool AcousticOutputCh2 { get; set; } = true;

    public TransferFunctionConfig TransferFunction { get; set; } = new();
    public TransferFileSettings? TransferFiles { get; set; }
    // 路由决定哪路作为参考及响应。显示相干、相位或冲激的开关只改变要呈现的结果。
    public string TransferRouting { get; set; } = "Auto";
    public bool IsTransferContinuous { get; set; }
    public bool TransferShowCoherence { get; set; } = true;
    public bool TransferShowPhase { get; set; }
    public bool TransferShowImpulse { get; set; }

    public RecorderMetricKind RecorderMetric { get; set; } = RecorderMetricKind.MultiAll;
    // 记录仪的读数间隔以毫秒保存。触发等待、录音时长以秒保存，预延时以毫秒保存，便于按独立阶段复现。
    public decimal RecorderIntervalMs { get; set; } = 31.25m;
    public RecorderTriggerMode RecorderTriggerMode { get; set; } = RecorderTriggerMode.Immediate;
    public InputChannelSelection RecorderTriggerChannel { get; set; } = InputChannelSelection.Channel1;
    public double RecorderTriggerTimeoutSeconds { get; set; } = 30;
    public int RecorderPreDelayMs { get; set; }
    public double RecorderDurationSeconds { get; set; }
    public RecorderEdgeSlope RecorderSlope { get; set; }
    // 瞬时过阈值比较用线性满量程值和边沿方向，传统电平触发则用 dBFS 门限。两种门限分别保存。
    public double RecorderEdgeThreshold { get; set; } = 0.010;
    public double RecorderTriggerThresholdDbfs { get; set; } = -40.0;
    public bool RecorderAppendMode { get; set; } = true;
    public bool RecorderSettledOnly { get; set; }
    public bool RecorderShowFullHistory { get; set; }
    public bool RecorderShowChannel2 { get; set; } = true;
    public bool RecorderShowThdOverlay { get; set; } = true;
}
