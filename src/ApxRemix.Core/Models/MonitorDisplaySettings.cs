namespace ApxRemix.Core.Models;

// 监视器是从同一批数据选取显示条件的消费者。示波器按周期数或点数截取，FFT 另用自己的窗口、长度和平均次数。
public sealed record MonitorDisplaySettings(
    bool ShowMeters, bool ShowPlots, bool AutomaticTime, int Cycles, int ManualSamples,
    bool Interpolate, bool ScopeCh2, ResidualDisplayGain ScopeResidual,
    int FftLength, FftWindowType Window, int Averages, bool FftCh2, bool FftResidual, ApxRemix.Core.Audio.AudibleMix? Audible = null, string? AudibleOutputId = null);
