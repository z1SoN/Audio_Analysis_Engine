// 这里给测量方式、通道和显示单位等有限选项命名。同一个枚举既出现在界面选择，也出现在 Core 的分支和保存的设置里，便于追踪一个选项会触发哪条路径。
namespace ApxRemix.Core.Models;

/// <summary>APx500 两大核心工作模式（Bench Mode / Sequence Mode）。</summary>
// Bench 由连续采集和仪表时钟驱动，Sequence 则按步骤取得记录。这个选择影响同名测量会进入哪条计算路径。
public enum AppOperationMode
{
    Bench,
    Sequence
}

/// <summary>Sequence Mode 左侧导航栏的视图/测量项类型。</summary>
public enum NavigationViewKind
{
    SignalPathSetup,
    LevelAndGain,
    ThdPlusN,
    SignalAnalyzer,
    FrequencyResponse,
    SequenceReport,
    SignalToNoiseRatio,
    CrosstalkOneChannelUndriven,
    InterchannelPhase,
    // Keep the original project-file values above. New catalog entries use 100 + APx MeasurementType.
    CrosstalkOneChannelDriven = 106,
    CrosstalkCustom = 108,
    DCLevel = 110,
    ContinuousSweep = 111,
    BandpassFrequencySweep = 112,
    SteppedFrequencySweep = 113,
    SteppedLevelSweep = 114,
    BandpassLevelSweep = 115,
    MeasurementRecorder = 117,
    MetadataRecorder = 118,
    MultitoneAnalyzer = 119,
    RegulatedFrequencySweep = 120,
    MaxOutput = 121,
    MaxOutputBurst = 122,
    Cmrr = 123,
    CmrrIec = 124,
    Noise = 125,
    Frequency = 126,
    DigitalErrorRate = 127,
    DynamicRange = 128,
    AcousticResponse = 129,
    DimImd = 130,
    Imd = 131,
    ImdLevelSweep = 132,
    DimLevelSweep = 133,
    BandpassLevel = 134,
    PassFail = 135,
    Sinad = 136,
    QpeakNoise = 137,
    NoiseRecorder = 138,
    ImdFrequencySweep = 139,
    CrosstalkFrequencySweepOneChannelUndriven = 140,
    CrosstalkFrequencySweepOneChannelDriven = 141,
    CrosstalkFrequencySweepCustom = 142,
    Psr = 143,
    PsrSweep = 144,
    Pesq = 145,
    PesqAverage = 146,
    Polqa = 147,
    PolqaAverage = 148,
    CompareEncodedBitstream = 149,
    DcLevelSweep = 150,
    VddRamp = 151,
    ModulatedNoise = 152,
    ImpedanceThieleSmall = 153,
    LoudspeakerProductionTest = 154,
    DcLevelDcx = 155,
    ResistanceDcx = 156,
    JitterFrequencySweep = 157,
    JitterLevelSweep = 158,
    LevelRatio = 159,
    SampleRate = 160,
    TedsData = 161,
    SignalAcquisition = 162,
    DutDelay = 163,
    TransferFunction = 164,
    AnsiS322ImdFrequencySweep = 165,
    Iec60118ImdFrequencySweep = 166,
}

public enum SequencePhaseMeterRange { Minus90To270, Minus180To180, ZeroTo360 }

/// <summary>Bench Mode 中央主工作区激活的测量面板。</summary>
public enum BenchMeasurementTab
{
    FftSpectrum,
    SteppedSweep,
    Recorder,
    ContinuousSweep,
    AcousticResponse,
    TransferFunction,
    SignalPath
}

/// <summary>Bench Mode 可动态添加/移除的实时仪表类型。</summary>
public enum BenchMeterKind
{
    RmsLevel,
    Gain,
    ThdPlusNRatio,
    ThdPlusNLevel,
    Sinad,
    PureThd,
    BandpassLevel,
    ImdRatio,
    Frequency,
    PeakLevel,
    NoiseFloor,
    InterChannelPhase,
    DcLevel,
    CrestFactor,
    ChannelBalance,
    RmsLevelRatio,
    InputSampleRate,
    Bits,
    DigitalErrorRate
}

/// <summary>步进扫频相位参考模式（滞后为负：φ = −360° × f × τ，折返至 (-180°, +180°]）。</summary>
// 同一波形可以相对于激励也可以相对于另一路来报告相位，记录参考方式才有可比性。
public enum SweepPhaseReferenceMode
{
    Auto,
    SynchronousExcitation,
    Ch2RelativeToCh1,
    None
}

/// <summary>逐点电平闭环调节结果状态（严格区分未启用、已达标、触及幅值上限与未收敛）。</summary>
// 闭环达到目标、触碰输出限幅及迭代耗尽分别表示不同的实验结果，调用方可据此保存或中止。
public enum RegulationOutcome
{
    /// <summary>未启用逐点调节。</summary>
    NotRequested,

    /// <summary>实测电平收敛至目标容差内（达标）。</summary>
    Converged,

    /// <summary>发生器触及允许输出上限/下限仍无法达到目标电平（未达标）。</summary>
    LimitReached,

    /// <summary>迭代耗尽仍未收敛至目标容差内（未达标）。</summary>
    NotConverged
}

/// <summary>Continuous Sweep 与 Acoustic Response 的测量来源模式（清晰隔离实测、开环与虚拟演示）。</summary>
public enum ChirpExecutionMode
{
    /// <summary>实测当前活跃音频后端（播放真实一次性 Chirp 并采集完整响应；拔线/无信号时明确失败，绝不回退仿真）。</summary>
    LiveEngine,

    /// <summary>开环 WAV 文件分析（先导出激励 Chirp WAV，再导入外部录制响应 WAV 反卷积）。</summary>
    OpenLoopWav,

    /// <summary>旧工程的演示模式标记，打开时迁移到 LiveEngine。</summary>
    VirtualDemo
}

/// <summary>连续扫频的相位参考及显示方式。</summary>
public enum ContinuousPhaseMode
{
    IrPeakAligned,
    Wrapped,
    Unwrapped,
    RelativeToCh1,
    Excess
}

/// <summary>单输入测量通道选择（用于 Acoustic Response 等指定麦克风输入路）。</summary>
public enum InputChannelSelection
{
    Channel1,
    Channel2
}

/// <summary>Analyzer 陷波器调谐模式。</summary>
// 陷波器中心决定从总信号中去掉哪一个音。配置它如何跟踪，才可解释剩余能量是否真的属于谐波和噪声。
public enum NotchTuningMode
{
    Generator,
    Measured,
    Fixed
}

/// <summary>Analyzer 仪表参考通道选择。</summary>
public enum AnalyzerReferenceChannel
{
    None,
    Channel1,
    Channel2
}

/// <summary>双音发生器路由模式。</summary>
public enum DualToneRoutingMode
{
    Sum,
    Split
}

/// <summary>FFT 采集触发模式。</summary>
// 触发模式决定哪一段样本会成为记录；平均模式规定跨多段时如何组合，二者分别改变时间选择与统计结果。
public enum FftTriggerMode
{
    FreeRun,
    RisingEdge,
    FallingEdge,
    PositiveZeroCrossing,
    Generator
}

/// <summary>FFT 频谱平均模式。</summary>
public enum FftAverageMode
{
    None,
    Exponential,
    Linear,
    Synchronous,
    MaximumHold,
    MinimumHold
}

/// <summary>步进扫频自变量类型。</summary>
// 步进扫既能改变频率也能改变幅度、偏置或相位，横轴的量纲由此选择。
public enum SweepAxisType
{
    Frequency,
    Level,
    Phase,
    GeneratorOffset
}

public enum SweepToneSource
{
    Sine,
    DualToneA,
    DualToneB,
    ImdFrequency1,
    ImdFrequency2,
    DfdMeanFrequency,
    DfdDifferenceFrequency
}

/// <summary>步进扫频横轴步进分布方式。</summary>
public enum SweepSpacingMode
{
    Logarithmic,
    Linear,
    CustomTable,
    R10,
    R20,
    R40,
    R80
}

/// <summary>步进扫频纵轴展示指标。</summary>
public enum SweepYMetric
{
    RelativeDb,
    GainDb,
    LevelDbfs,
    ThdPlusNPercent,
    ThdPlusNDb
}

/// <summary>Bench Mode Measurement Recorder 跟踪指标枚举。</summary>
public enum RecorderMetricKind
{
    RmsLevelDbfs,
    ThdPlusNDb,
    FrequencyHz,
    MultiAll,
    PeakLevelDbfs,
    BandpassLevelDbfs,
    ThdNLevelDbfs,
    SinadDb,
    CrestFactorDb,
    DcLevelMv,
    GainDb,
    PhaseDeg,
    ImdRatioPercent,
    RmsLevelRatioDb,
    InputSampleRate,
    ErrorRate
}

/// <summary>Recorder 触发启动方式。</summary>
// 记录仪按触发条件选开始时刻；需要过阈值时还要指定上升或下降方向。
public enum RecorderTriggerMode
{
    Immediate,
    LevelAboveThreshold,
    LevelBelowThreshold,
    PositiveZeroCrossing,
    Generator,
    InstantaneousLevel
}

public enum RecorderEdgeSlope { Rising, Falling }

/// <summary>分析仪电平显示单位。</summary>
public enum LevelDisplayUnit
{
    Dbfs,
    Vrms,
    Dbv,
    Dbu,
    Dbr,
    DbrB,
    Dbm,
    DbSpl,
    DbSpl1,
    DbSpl2,
    Watts
}

/// <summary>失真比仪表显示单位（百分比或 dB）。</summary>
public enum DistortionDisplayUnit
{
    Percent,
    Db
}

/// <summary>示波器残余波形增益模式（对齐 APx500 ResidualDisplay：Off / x1 / x10 / x100 / x1K / x10K / x100K）。</summary>
public enum ResidualDisplayGain
{
    Off = 0,
    X1 = 1,
    X10 = 10,
    X100 = 100,
    X1K = 1000,
    X10K = 10000,
    X100K = 100000
}

/// <summary>分析仪计权与去加重滤波器类型（对齐 APx500 WtFilterType）。</summary>
// 计权先改变各个频段参与结果的程度；随后的 RMS 或失真分析会继承这个频率选择。
public enum AnalyzerWeighting
{
    Flat,
    AWeighting,
    CWeighting,
    Ccir1k,
    Ccir2k,
    Deemphasis50us,
    Deemphasis75us,
    BWeighting,
    Deemphasis50usA,
    Deemphasis75usA,
    Ccitt,
    CMessage
}

/// <summary>分析仪高通滤波器拓扑（对齐 APx500 HP4 Butterworth 与 HP5 Elliptic 高通）。</summary>
public enum HighPassFilterType
{
    Butterworth4,
    Elliptic5
}

/// <summary>分析仪低通滤波器拓扑（对齐 APx500 LP8 Butterworth 与 AES17 砖墙低通）。</summary>
public enum LowPassFilterType
{
    Butterworth8,
    Aes17,
    Elliptic8
}

/// <summary>带通电平表选择性（对齐 APx500 6阶 Butterworth 1/3、1/6、1/12、1/24 倍频程带通）。</summary>
public enum BandpassSelectivity
{
    ThirdOctave = 3,
    OneOctave = 1,
    NinthOctave = 9,
    SixthOctave = 6,
    TwelfthOctave = 12,
    TwentyFourthOctave = 24,
    SecondOrderQ10 = 0
}

/// <summary>互调失真测量标准（对齐 APx500 IMDType：SMPTE / CCIF / DFD）。</summary>
public enum ImdMeasurementStandard
{
    Auto,
    Smpte,
    Ccif,
    Dfd,
    DimEmulation,
    Mod,
    Iec60118
}

/// <summary>DFD / CCIF 互调产物阶次组合（对齐 APx500 IMDMode：d2 / d3 / d23 / d25）。</summary>
public enum ImdMeasurementMode
{
    D2,
    D3,
    D23,
    D25
}

/// <summary>THD 分母参考标准（对齐 APx500 默认总 RMS 参考与 IEEE 基波参考）。</summary>
// 百分比或 dB 描述的是比值，分母采用基波或其他指定参考会改变结果，所以需要记录口径。
public enum ThdReferenceMode
{
    /// <summary>APx500 默认（IEC）：THD = Harmonics / Total RMS。</summary>
    TotalRmsIec,

    /// <summary>IEEE 标准：THD = Harmonics / Fundamental RMS。</summary>
    FundamentalIeee
}

/// <summary>读数稳定判定算法（对齐 APx500 TimeDomainFilters.SettlingType）。</summary>
public enum SettlingAlgorithm
{
    None,
    Average,
    Flat
}

/// <summary>连续扫频激励波形类型（对齐 APx500 ApChirp 与标准对数 LogChirp）。</summary>
public enum ContinuousChirpType
{
    ApChirp,
    LogChirp,
    FastSweep
}

/// <summary>音频引擎后端模式。</summary>
// 信号既可来自设备也可来自文件或模型，原始采样的来源决定时间戳、通道及硬件标定能代表什么。
public enum AudioBackendMode
{
    VirtualSimulator,
    Wasapi,
    Asio,
    WavFile
}

public enum AudioOutputConnector
{
    VirtualLoopback, Wasapi, Asio, ExternalSource
}

/// <summary>信号发生器波形类型。</summary>
public enum GeneratorWaveform
{
    Sine,
    DualTone,
    Square,
    Multitone,
    ImdSmpte,
    ImdCcif,
    WhiteNoise,
    PinkNoise,
    WavPlayback,
    SineBurst,
    Dim30,
    Dim100,
    DimB,
    DimB8,
    IecProgrammeNoise,
    EnProgrammeNoise,
    SineVarPhase,
    ConstantValue,
    WalkingOnes,
    WalkingZeros,
    BitTest
}

/// <summary>FFT 分析所用的时域加窗函数类型。</summary>
// 窗口改变截断记录的频谱泄漏与等效带宽，进而影响峰值与噪声密度的估计。
public enum FftWindowType
{
    Hann,
    BlackmanHarris,
    FlatTop,
    Rectangular,
    BlackmanHarris4,
    Equiripple,
    Dolph150,
    Dolph200,
    Dolph250,
    NoneMoveToBinCenter
}

/// <summary>APx transfer function estimators.</summary>
// H1、H2 等估计方式改变参考自谱、响应自谱及交叉谱的组合，选择会影响噪声进入估计的方式。
public enum TransferFunctionCalculationMode
{
    H1,
    H2,
    MagnitudeOnly
}
