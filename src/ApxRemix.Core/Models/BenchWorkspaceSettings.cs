using System.Text.Json.Serialization;

namespace ApxRemix.Core.Models;

// 一条曲线用配对的 X、Y 数组保存。Y 允许 NaN，才能把没有有效读数的点与实测为零的点区分开。
public sealed record BenchTrace(double[] X, [property: JsonNumberHandling(JsonNumberHandling.AllowNamedFloatingPointLiterals)] double[] Y);
// 声学快照留存原始校准、采样率及窗口起止时间，以便后来切换窗口或通道时保持对应的物理尺度。
public sealed record AcousticCaptureSettings(AcousticResponseConfig Config, double InputFullScale, double OtherMicSensitivity, int SampleRate,
    double WindowStartMs, double WindowEndMs);
public sealed record ContinuousPhaseCapture(BenchTrace? Channel1, BenchTrace? Channel2, ContinuousPhaseMode Mode = ContinuousPhaseMode.RelativeToCh1);
// 捕获结果同时保存来源、时间、两路曲线和必要分析条件，使后来切换视图时仍能找回同一次采集。
public sealed record BenchCapture(string Source, DateTimeOffset Time, Dictionary<string, BenchTrace> Traces, Dictionary<string, BenchTrace>? Channel2 = null, AcousticCaptureSettings? Acoustic = null, Dictionary<string,BenchExternalTrace[]>? FileTraces = null, int? InputBitDepth = null, ContinuousPhaseCapture? ContinuousPhase = null);
// 工作区保存结果集编号、可见性和用户备注。测量时间保存在捕获记录，导入时间可另外标记。
public sealed record BenchSavedDataSet(int Number, BenchCapture Capture, string Notes, bool Visible,
    DateTimeOffset? ImportedAt = null, int DisplayNumber = 0);
public sealed record BenchSavedRow(string Name, string Right, string XUnit, string LeftUnit, BenchGraphLimitSettings? Limits = null, BenchExportSpec? Export = null, BenchGraphOptions? Graph = null, string? RightUnit = null, BenchChirpResultSettings? Chirp = null);
public sealed record BenchChirpResultSettings(int Order = 2, bool Sum = false, string Orders = "2,3", bool Iec = false,
    bool Normalized = false, double MinHz = 20, double MaxHz = 20000, double OctaveWidth = 0);
// Set 0 follows the most recent acquisition. Specific measured and imported sets use their project numbers.
// 派生曲线保存所依赖的原始数据集、通道和参数；重新加载工程时依此复原数学关系。
public sealed record BenchEqTarget(string Name, int Set, bool Ch2, string CurveName, BenchTrace Curve);
public sealed record BenchDerivation(string Name,string Source,string Function,double Parameter, double[]? Points = null, int ReferenceSet = 0, bool ReferenceCh2 = false, BenchEqTarget[]? EqTargets = null);
public sealed record BenchWorkspaceSettings(BenchSavedRow[] Rows, BenchSavedDataSet[] Sets, string? Selected,
    bool Append, bool Repeat, bool ShowCh1, bool ShowCh2, bool TableView,
    bool NestGeneratorLevel = false, double NestStart = -30, double NestStop = -10, int NestPoints = 3,
    BenchDerivation[]? Derivations = null, BenchExternalResult[]? Files = null,
    BenchSweepSource NestSource = BenchSweepSource.GeneratorLevel, SweepSpacingMode NestSpacing = SweepSpacingMode.Linear, string NestTable = "",
    bool DataSetsVisible = true);

public sealed record BenchExternalTrace(string Label, BenchTrace Trace);
public sealed record BenchExternalResult(string Name, string Path, string XUnit, string Unit, bool Bar);

// 导出的采点策略与图的显示范围分开保存，便于保留完整原始数据，同时按指定密度输出文件。
public sealed record BenchExportSpec(bool Included = true, string Mode = "Number of points", int Points = 1200,
    double From = 20, double To = 20000, string Coordinates = "", int SetNumber = 0, int Channel = 0);

// 坐标范围与曲线外观只影响显示。左右轴可采用各自的自动范围或固定极值，实际单位从结果定义读取。
public sealed record BenchGraphOptions(string Title = "", string XScale = "Auto", string YScale = "Auto",
    bool AutoX = true, double XMin = 20, double XMax = 20000, bool AutoY = true, double YMin = -100,
    double YMax = 10, bool Legend = true, bool Grid = true, BenchTraceAppearance[]? Traces=null,
    bool ScopeInterpolation = true, bool AutoRightY = true, double RightYMin = 0,
    double RightYMax = 1, string RightYScale = "Auto");
public sealed record BenchTraceAppearance(bool Right,int Channel,bool Visible,string Color);
