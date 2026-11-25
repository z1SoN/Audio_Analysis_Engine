using Group = ApxRemix.Core.Models.ApxSequenceCategory;

namespace ApxRemix.Core.Models;

// 一个测量项可属于多个分类，所以分类值按位编码。目录列出候选项目，设备实际提供的集合还受连接设备和安装模块影响。
[Flags]
public enum ApxSequenceCategory
{
    None = 0,
    Basic = 1,
    Meters = 2,
    Sweeps = 4,
    Distortion = 8,
    Crosstalk = 16,
    Perceptual = 32,
    Acoustic = 64,
    All = 128
}

public sealed record ApxSequenceCatalogEntry(string Id, string Name, ApxSequenceCategory Category);

/// <summary>
/// Built-in candidates for the APx500 10.0 Add Measurement dialog. Connected devices expose
/// their available subset through GetAllMeasurementNames, which also includes installed custom measurements.
/// </summary>
public static class ApxSequenceCatalog
{
    // The dialog creates All first, then adds tabs in MeasurementGroup enum order.
    public static IReadOnlyList<ApxSequenceCategory> Categories { get; } = Array.AsReadOnly<Group>(
        [Group.All, Group.Basic, Group.Meters, Group.Sweeps, Group.Distortion, Group.Crosstalk, Group.Perceptual, Group.Acoustic]);

    // MeasurementGroups.Groups / UserModelExtensions.GetAvailableMeasurements, APx500 10.0.0.17.
    // Long names come from the English MeasurementType resources used in the Add dialog.
    // 用稳定的内部 ID 识别项目，用 Name 给人阅读。组合标志决定可在哪些选项卡中找到同一个测量。
    public static IReadOnlyList<ApxSequenceCatalogEntry> All { get; } = Array.AsReadOnly<ApxSequenceCatalogEntry>(
    [
        new("AcousticResponse", "Acoustic Response", Group.Sweeps | Group.Distortion | Group.Acoustic | Group.All),
        new("AnsiS322ImdFrequencySweep", "ANSI S3.22 IMD Frequency Sweep", Group.Sweeps | Group.Distortion | Group.Acoustic | Group.All),
        new("BandpassFrequencySweep", "Bandpass Frequency Sweep", Group.Sweeps | Group.All),
        new("BandpassLevel", "Bandpass Level", Group.Meters | Group.All),
        new("BandpassLevelSweep", "Bandpass Level Sweep", Group.Sweeps | Group.All),
        new("Cmrr", "CMRR", Group.Meters | Group.All),
        new("CmrrIec", "CMRR IEC", Group.Meters | Group.All),
        new("CompareEncodedBitstream", "Compare Encoded Bitstream to Reference", Group.All),
        new("ContinuousSweep", "Continuous Sweep", Group.Sweeps | Group.Distortion | Group.Crosstalk | Group.All),
        new("CrosstalkFrequencySweepCustom", "Crosstalk Sweep, Custom", Group.Sweeps | Group.Crosstalk | Group.All),
        new("CrosstalkFrequencySweepOneChannelDriven", "Crosstalk Sweep, One Channel Driven", Group.Sweeps | Group.Crosstalk | Group.All),
        new("CrosstalkFrequencySweepOneChannelUndriven", "Crosstalk Sweep, One Channel Undriven", Group.Sweeps | Group.Crosstalk | Group.All),
        new("CrosstalkCustom", "Crosstalk, Custom", Group.Meters | Group.Crosstalk | Group.All),
        new("CrosstalkOneChannelDriven", "Crosstalk, One Channel Driven", Group.Meters | Group.Crosstalk | Group.All),
        new("CrosstalkOneChannelUndriven", "Crosstalk, One Channel Undriven", Group.Basic | Group.Meters | Group.Crosstalk | Group.All),
        new("DCLevel", "DC Level", Group.Meters | Group.All),
        new("DcLevelDcx", "DC Level (DCX)", Group.Meters | Group.All),
        new("DcLevelSweep", "DC Level Sweep", Group.Sweeps | Group.All),
        new("DigitalErrorRate", "Digital Error Rate", Group.All),
        new("DimImd", "DIM", Group.Meters | Group.Distortion | Group.All),
        new("DimLevelSweep", "DIM Level Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("DutDelay", "DUT Delay", Group.Basic | Group.All),
        new("DynamicRange", "Dynamic Range - AES17", Group.Meters | Group.All),
        new("Frequency", "Frequency", Group.Meters | Group.All),
        new("FrequencyResponse", "Frequency Response", Group.Basic | Group.Sweeps | Group.All),
        new("Iec60118ImdFrequencySweep", "IEC 60118-0 IMD Frequency Sweep", Group.Sweeps | Group.Distortion | Group.Acoustic | Group.All),
        new("Imd", "IMD (DFD/MOD/SMPTE/CCIF)", Group.Meters | Group.Distortion | Group.All),
        new("ImdFrequencySweep", "IMD (DFD/MOD/SMPTE/CCIF) Frequency Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("ImdLevelSweep", "IMD (DFD/MOD/SMPTE/CCIF) Level Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("ImpedanceThieleSmall", "Impedance/Thiele-Small", Group.Acoustic | Group.All),
        new("SampleRate", "Input Sample Rate", Group.Meters | Group.All),
        new("InterchannelPhase", "Interchannel Phase", Group.Basic | Group.Meters | Group.All),
        new("JitterFrequencySweep", "Jitter Frequency Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("JitterLevelSweep", "Jitter Level Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("LevelAndGain", "Level and Gain", Group.Basic | Group.Meters | Group.All),
        new("LevelRatio", "Level Ratio", Group.Meters | Group.All),
        new("LoudspeakerProductionTest", "Loudspeaker Production Test", Group.Acoustic | Group.All),
        new("MaxOutput", "Maximum Output", Group.Meters | Group.All),
        new("MaxOutputBurst", "Maximum Output (CEA-2006)", Group.Meters | Group.All),
        new("MeasurementRecorder", "Measurement Recorder", Group.Sweeps | Group.Distortion | Group.All),
        new("MetadataRecorder", "Metadata Recorder", Group.Sweeps | Group.All),
        new("ModulatedNoise", "Modulated Noise", Group.Meters | Group.Distortion | Group.Acoustic | Group.All),
        new("MultitoneAnalyzer", "Multitone Analyzer", Group.Distortion | Group.All),
        new("QpeakNoise", "Noise (Q-peak per ITU-R BS.468-4)", Group.Meters | Group.All),
        new("Noise", "Noise (RMS)", Group.Meters | Group.All),
        new("NoiseRecorder", "Noise Recorder (RMS)", Group.Sweeps | Group.All),
        new("PassFail", "Pass/Fail", Group.All),
        new("Pesq", "PESQ", Group.Perceptual | Group.All),
        new("PesqAverage", "PESQ (Averaged)", Group.Perceptual | Group.All),
        new("Polqa", "POLQA", Group.Perceptual | Group.All),
        new("PolqaAverage", "POLQA (Averaged)", Group.Perceptual | Group.All),
        new("Psr", "PSR", Group.Meters | Group.All),
        new("PsrSweep", "PSR Frequency Sweep", Group.Sweeps | Group.All),
        new("RegulatedFrequencySweep", "Regulated Frequency Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("ResistanceDcx", "Resistance (DCX)", Group.Meters | Group.All),
        new("SignalAcquisition", "Signal Acquisition", Group.All),
        new("SignalAnalyzer", "Signal Analyzer", Group.All),
        new("SignalToNoiseRatio", "Signal to Noise Ratio", Group.Basic | Group.Meters | Group.All),
        new("Sinad", "SINAD", Group.Meters | Group.Distortion | Group.All),
        new("SteppedFrequencySweep", "Stepped Frequency Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("SteppedLevelSweep", "Stepped Level Sweep", Group.Sweeps | Group.Distortion | Group.All),
        new("TedsData", "TEDS Data", Group.Acoustic | Group.All),
        new("ThdN", "THD+N", Group.Basic | Group.Meters | Group.Distortion | Group.All),
        new("TransferFunction", "Transfer Function", Group.Acoustic | Group.All),
        new("VddRamp", "Vdd Ramp", Group.All)
    ]);
}

