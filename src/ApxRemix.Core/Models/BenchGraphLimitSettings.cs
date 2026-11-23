namespace ApxRemix.Core.Models;

// 上、下限各有独立开关并绑定轴和通道；保存文本可保留限值表达方式，比较时还要使用匹配的物理单位。
public sealed record BenchLimitPair(string Axis, int Channel, bool UpperEnabled, string Upper, bool LowerEnabled, string Lower);
// 跟踪第一路或同时使用两根轴都影响限值应用范围，所以这些开关要和实际的上下限对一起保存。
public sealed record BenchGraphLimitSettings(bool TrackCh1, bool BothAxes, BenchLimitPair[] Limits);
