using ApxRemix.Core.Models;
using ApxRemix.Core.Storage;

namespace ApxRemix.Core.Dsp;

public static class BenchFastSweep
{
    public static ContinuousSweepConfig WithTableBand(ContinuousSweepConfig cfg, int rate)
    {
        // 自定义频点表决定实际激励范围，分析频带也须按表的两端更新；否则会在没有发声的频率要求增益。
        if (cfg.ChirpType != ContinuousChirpType.FastSweep || cfg.FastSweep.Spacing != SweepSpacingMode.CustomTable)
            return cfg;
        var frequencies = Plan(cfg, rate).FrequenciesHz;
        var effective = cfg.Clone();
        effective.StartFrequencyHz = frequencies.Min();
        effective.StopFrequencyHz = frequencies.Max();
        return effective;
    }

    public static FastSweepSettings Plan(ContinuousSweepConfig cfg, int rate)
    {
        var options = cfg.FastSweep;
        // 将用户指定的间距、幅度和停留规则展开为具体频率序列。所需总时间来自逐点计划，点数和总时长要在分配波形前验证。
        if (options.Points is < 2 or > 512) throw new ArgumentException("Choose 2–512 frequency points.");
        var hz = options.Spacing switch
        {
            SweepSpacingMode.CustomTable => ProjectFileService.ParseCustomSweepPoints(options.TableHz),
            SweepSpacingMode.Linear => FrequencyResponseSweeper.GenerateLinearPoints(cfg.StartFrequencyHz, cfg.StopFrequencyHz, options.Points),
            >= SweepSpacingMode.R10 => FrequencyResponseSweeper.PreferredFrequencies(cfg.StartFrequencyHz, cfg.StopFrequencyHz, options.Spacing),
            _ => FrequencyResponseSweeper.GenerateLogFrequencies(cfg.StartFrequencyHz, cfg.StopFrequencyHz, options.Points)
        };
        if (hz.Length is < 2 or > 512) throw new ArgumentException("The Fast Sweep frequency table must contain 2–512 points.");
        if (options.Spacing is SweepSpacingMode.Linear or SweepSpacingMode.Logarithmic)
            hz[^1] = cfg.StopFrequencyHz;
        ContinuousSweepAnalyzer.CheckStopFrequency(hz.Max(), rate);
        var amplitude = Enumerable.Repeat(SignalGeneratorDsp.DbfsToLinearPeak(cfg.LevelDbfs), hz.Length).ToArray();
        var plan = new FastSweepSettings(rate, hz, amplitude, options.AutoTime ? 0 : cfg.DurationSeconds,
            options.TransitionCycles, options.DiscardCycles, options.MeasureCycles, options.TransitionMs / 1000,
            options.DiscardMs / 1000, options.MeasureMs / 1000);
        if (FastSweepGenerator.Duration(plan) > 15)
            throw new ArgumentException("Fast Sweep exceeds 15 seconds. Reduce the point count or the minimum cycles and times.");
        return plan;
    }

    internal static ContinuousSweepResult Analyze(ReadOnlySpan<double> recording, ReadOnlySpan<double> reference,
        int rate, ContinuousSweepConfig config, int delay, ContinuousSweepResult impulse)
    {
        var plan = Plan(config, rate);
        var excitation = FastSweepGenerator.Generate(plan);
        // The reference file can include leading silence or have a different full-scale amplitude.
        static int Onset(ReadOnlySpan<double> samples)
        {
            // 对每段参考各自用峰值的千分之一找起始点，因此同一波形整体缩放后可得到相同起点。
            double peak = 0;
            foreach (double sample in samples) peak = Math.Max(peak, Math.Abs(sample));
            for (int i = 0; i < samples.Length; i++)
                if (Math.Abs(samples[i]) > peak * 0.001) return i;
            return 0;
        }
        // 参考文件与新生成的激励起点之差修正前导静音；录到的输出还需另外加上反卷积得到的通道延时。
        int origin = Onset(reference) - Onset(excitation.Samples);
        var slices = FastSweepGenerator.AcquisitionSlices(plan, rate, excitation.Samples.Length, 0, false);
        double calibration = AudioMetricsAnalyzer.ComputeCalibrationGainOffsetDb(config.InputFullScaleVoltageVrms, config.OutputFullScaleVoltageVrms);
        List<FrequencyResponsePoint> levels = [], thd = [], thdn = [];
        double previousResidual = double.NaN;
        var orders = config.ExtractHarmonics ? Enumerable.Range(2, 19)
            .ToDictionary(order => order, _ => Enumerable.Repeat(double.NaN, slices.Length).ToArray()) : null;
        foreach (var slice in slices)
        {
            int r = origin + slice.Start, y = r + delay;
            // 只有参考和响应都覆盖完整测量段时才取数，一个越界切片会把别的频点误当成本点。
            if (r < 0 || y < 0 || slice.Count < 64 || slice.Count > reference.Length - r || slice.Count > recording.Length - y)
                throw new InvalidDataException("Fast Sweep requires a complete acquisition for each selected tone.");
            var source = reference.Slice(r, slice.Count);
            var measured = recording.Slice(y, slice.Count);
            double hz = slice.FrequencyHz;
            var input = AudioMetricsAnalyzer.ExtractSingleToneLockIn(source, rate, hz);
            var output = AudioMetricsAnalyzer.ExtractSingleToneLockIn(measured, rate, hz);
            // 比较目标正弦峰值与总交流 RMS，检查该段确由预期音调主导。增益使用响应电平减参考电平，再加两端的电压标定差。
            double sum=0, squares=0;
            foreach (double sample in source) { sum+=sample; squares+=sample*sample; }
            double mean=sum/source.Length;
            double sourceAcPower = Math.Max(0,squares/source.Length-mean*mean);
            if (sourceAcPower <= 0 || input.PeakLinear < 0.8 * Math.Sqrt(2 * sourceAcPower))
                throw new InvalidDataException($"Reference waveform does not match the Fast Sweep schedule at {hz:G5} Hz.");
            double gain = output.LevelDbfs - input.LevelDbfs + calibration;
            // 先求同一段两路相位差，再利用上一频点展开跨圈变化。是否还原传播延时产生的相位斜率由显示模式决定。
            double residual = FrequencyResponseSweeper.NormalizeAngleDeg(output.PhaseDeg - input.PhaseDeg);
            if (double.IsFinite(previousResidual))
                residual = previousResidual + FrequencyResponseSweeper.NormalizeAngleDeg(residual - previousResidual);
            if (output.LevelDbfs > -150) previousResidual = residual;
            double phase = residual;
            if (config.PhaseMode != ContinuousPhaseMode.IrPeakAligned) phase -= 360.0 * hz * delay / rate;
            if (config.PhaseMode is ContinuousPhaseMode.Wrapped or ContinuousPhaseMode.IrPeakAligned)
                phase = FrequencyResponseSweeper.NormalizeAngleDeg(phase);
            double level = output.LevelDbfs;
            var point = new FrequencyResponsePoint(hz, level, level, gain, gain, 0, 0, phase, phase, phase, XValue: hz,
                IsValid: output.LevelDbfs > -150, ExcitationToneDbfs: input.LevelDbfs);
            if (orders is not null && point.IsValid)
                ChirpResultMath.FastProducts(measured, hz, rate, levels.Count, orders);
            levels.Add(point);
            // 固定本段的已知 Hz 拟合基波，THD 取分离出的谐波，启用 THD+N 时另外保存含噪残余的比值和绝对电平。
            if (config.ExtractHarmonics || config.FastSweep.MeasureThdn)
            {
                var d = AudioMetricsAnalyzer.AnalyzeThdPlusN(measured, measured, rate, hz, strictFixedNotch: true).Ch1;
                bool hasHarmonics = d.HasValidSignal && d.Harmonics.Count != 0;
                thd.Add(point with { IsValid = hasHarmonics, Ch1ThdNPercent = hasHarmonics ? d.ThdRatioPercent : double.NaN,
                    Ch1ThdNDb = hasHarmonics ? d.ThdRatioDb : double.NaN });
                if (config.FastSweep.MeasureThdn)
                    thdn.Add(point with { IsValid = d.HasValidSignal, Ch1ThdNPercent = d.ThdNRatioPercent, Ch1ThdNDb = d.ThdNRatioDb,
                        Ch1LevelDbfs = d.ThdNLevelDbfs });
            }
        }
        // 相对曲线以参考在 1 kHz 对应的响应电平为基准，从有效点按对数频率插值；超出扫频范围则沿用最近端值。
        var ordered = levels.Where(p => p.IsValid).OrderBy(p => p.FrequencyHz).ToArray();
        double baseline = ordered.Length == 0 ? double.NaN : FrequencyGainTable.GainAtHz(
            new(ordered.Select(p => p.FrequencyHz).ToArray(), ordered.Select(p => p.Ch1LevelDbfs).ToArray()), 1000);
        return impulse with { ResponsePoints = levels.Select(p => p with { Ch1RelativeDb = p.Ch1LevelDbfs - baseline,
                Ch2RelativeDb = p.Ch2LevelDbfs - baseline }).ToArray(), ThdVsFreqPoints = thd, FastThdnPoints = thdn,
            ProductLevels = orders?.Where(o => o.Value.Any(double.IsFinite)).ToDictionary(o => o.Key,
                o => new BenchTrace(levels.Select(p => p.FrequencyHz).ToArray(), o.Value)) };
    }
}
