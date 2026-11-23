// 设备不断索要下一块输出采样。FillBlock 用持续保存的相位生成相邻波形块，也能播放一次性激励；这样驱动在哪个样本处分成两块都不必重新起振。
using ApxRemix.Core.Models;
using ApxRemix.Core.Storage;

namespace ApxRemix.Core.Dsp;

/// <summary>
/// 相位连续的多波形音频信号发生器（Signal Generator DSP）。
/// <para>
/// 支持波形：Sine、DualTone（Sum/Split + 可调幅度比）、Square、10 音 Schroeder Multitone、
/// 标准 SMPTE IMD（60 Hz + 7 kHz, 4:1）、标准 CCIF IMD（19 kHz + 20 kHz, 1:1）、WhiteNoise、PinkNoise 与本地 WAV 播放。
/// 支持 Ch1/Ch2 独立输出电平、连续可调 Ch2 相位偏移（−180° ~ +180°）及 180° 反相。
/// </para>
/// </summary>
public sealed partial class SignalGeneratorDsp
{
    private static readonly double[] MultitoneFrequencies =
        [31.25, 63.0, 127.0, 251.0, 503.0, 997.0, 2003.0, 4001.0, 8009.0, 16001.0];

    private static readonly double[] MultitoneInitialPhases = BuildSchroederPhases(MultitoneFrequencies.Length);

    // 音频设备每次只请求一小块。相位、噪声滤波器记忆和文件游标必须跨块保留，否则每个块边界都会重复起振。
    private ProgrammeNoise? _programmeNoise;
    private double _burstPosition;
    private int _wavPass;
    private long _requestedStart = -1;
    private bool _burstActive, _wavActive;
    private double _phasePrimary;
    private double _phaseSecondary;
    private DimStimulus? _dimStimulus;
    private readonly double[] _multitonePhases = (double[])MultitoneInitialPhases.Clone();

    private readonly Random _rng = new(20261121);
    private double _b0, _b1, _b2, _b3, _b4, _b5, _b6;

    // WAV 缓存保留原始采样率；播放到不同采样率时以三次 Hermite 插值估计源波形的分数采样位置。
    private string? _loadedWavPath;
    private double[] _wavLeft = Array.Empty<double>();
    private double[] _wavRight = Array.Empty<double>();
    private int _wavSampleRate = 48000;
    private int _wavFileChannels, _cachedFirst, _cachedSecond;
    private double _wavPosition;

    /// <summary>最近一次 WAV 波形加载错误（成功时为 null，便于上层向用户暴露损坏/缺失文件错误）。</summary>
    public string? LastWavPlaybackError { get; private set; }

    // 一次性 Chirp 激励队列（支持通道路由与精确播完检测）
    private readonly object _oneShotLock = new();
    private double[]? _oneShotBuffer;
    private double[]? _oneShotRight;
    private int _oneShotCursor;
    private int _oneShotSecondDelay;
    private bool _oneShotEnableCh1 = true;
    private bool _oneShotEnableCh2 = true;

    /// <summary>当前一次性激励是否仍有未播完的样本（用于确保输出端真正播完尾段后再结束采集）。</summary>
    public bool HasActiveOneShot { get { lock (_oneShotLock) return _oneShotBuffer is not null && _oneShotCursor < _oneShotBuffer.Length; } }

    /// <summary>当前是否处于一次性激励接管期间（含播完后保持静音等待 ClearOneShot 的阶段）。</summary>
    public bool IsOneShotOwned { get { lock (_oneShotLock) return _oneShotBuffer is not null; } }

    public bool OneShotCh1Enabled { get { lock (_oneShotLock) return _oneShotEnableCh1; } }
    public bool OneShotCh2Enabled { get { lock (_oneShotLock) return _oneShotEnableCh2; } }

    /// <summary>排入一段一次性激励波形（含前后静音保护带与独立通道使能），播放结束后保持静音直到调用 <see cref="ClearOneShot"/>。</summary>
    public void EnqueueOneShot(
        double[] monoSamples,
        int preSilenceSamples = 0,
        int postSilenceSamples = 0,
        bool enableCh1 = true,
        bool enableCh2 = true,
        int secondChannelDelaySamples = 0,
        double[]? secondChannelSamples = null)
    {
        // 一次性激励在首尾加静音，并为第二路延时预留长度。准备好整段再交给回调，样本时序就由同一个游标决定。
        int pre = Math.Max(0, preSilenceSamples);
        int post = Math.Max(0, postSilenceSamples);
        int delay = Math.Max(0, secondChannelDelaySamples);
        var buf = new double[checked(pre + monoSamples.Length + post + delay)];
        Array.Copy(monoSamples, 0, buf, pre, monoSamples.Length);
        double[]? right = null;
        // 若两路共享同一源数组，右路可按延时读取公共缓冲。独立右路必须与左路等长，才能沿同一游标输出。
        if (secondChannelSamples != null && !ReferenceEquals(secondChannelSamples, monoSamples))
        {
            if (secondChannelSamples.Length != monoSamples.Length) throw new ArgumentException("Sweep output lengths differ.");
            right = new double[buf.Length];
            Array.Copy(secondChannelSamples, 0, right, pre, secondChannelSamples.Length);
        }
        lock (_oneShotLock)
        {
            _oneShotBuffer = buf;
            _oneShotRight = right;
            _oneShotSecondDelay = delay;
            _oneShotCursor = 0;
            _oneShotEnableCh1 = enableCh1;
            _oneShotEnableCh2 = enableCh2;
        }
    }

    // 播放完成仅表示游标到达末端，本次接管仍保持静音。清除接管后，常规发生器才恢复输出。
    /// <summary>清除一次性激励并恢复常规连续波形输出。</summary>
    public void ClearOneShot()
    {
        lock (_oneShotLock)
        {
            _oneShotBuffer = null;
            _oneShotRight = null;
            _oneShotCursor = 0;
            _oneShotEnableCh1 = true;
            _oneShotEnableCh2 = true;
        }
    }

    /// <summary>生成下一个连续采样的立体声激励块。</summary>
    public int[]? FileStartOffsets { get; private set; }
    public bool StartedOneShotBlock { get; private set; }

    public void FillBlock(GeneratorConfig config, int sampleRate, Span<double> leftOut, Span<double> rightOut)
    {
        FileStartOffsets = null;
        StartedOneShotBlock = false;
        int count = Math.Min(leftOut.Length, rightOut.Length);

        // 若存在已排入的一次性 Chirp 激励，按显式通道使能精确输出（播放完毕后保持静音直到 ClearOneShot）
        lock (_oneShotLock)
        {
            // 若扫频接管了输出，就按共同游标先写两路。缓冲播完后的空余回调样本保持静音，以免连续发生器提前混入测量尾巴。
            if (_oneShotBuffer is not null)
            {
                StartedOneShotBlock = _oneShotCursor == 0 && _oneShotBuffer.Length > 0 && count > 0;
                for (int i = 0; i < count; i++)
                {
                    if (_oneShotCursor >= _oneShotBuffer.Length)
                    {
                        leftOut[i..count].Clear();
                        rightOut[i..count].Clear();
                        break;
                    }
                    double v = _oneShotBuffer[_oneShotCursor];
                    int secondIndex = _oneShotCursor++ - _oneShotSecondDelay;
                    leftOut[i] = _oneShotEnableCh1 ? v : 0.0;
                    rightOut[i] = _oneShotEnableCh2 && secondIndex >= 0 ? (_oneShotRight ?? _oneShotBuffer)[secondIndex] : 0.0;
                }
                return;
            }
        }

        // 关闭或切换波形时清掉相应的起始状态；未切换时维持周期位置，下一音频块便能从前一块的末端继续。
        if (!config.IsEnabled || config.Waveform != GeneratorWaveform.SineBurst) _burstActive = false;
        if (!config.IsEnabled || config.Waveform != GeneratorWaveform.WavPlayback) _wavActive = false;
        if (!config.IsEnabled || config.Waveform != _patternKind) ResetPattern(config.Waveform);
        // 数字字型直接写 PCM 并返回，使后面的电平缩放、直流偏置和模拟波形抖动都不会改变被检查的位。
        if (config.IsEnabled && GeneratorWaveSettings.IsDigital(config.Waveform))
        {
            FillDigitalPattern(config,leftOut[..count],rightOut[..count]);
            return;
        }
        if (!config.IsEnabled)
        {
            leftOut[..count].Clear();
            rightOut[..count].Clear();
            return;
        }

        // dBFS 描述相对数字满量程的幅值，转线性幅值要用 10^(dB/20)。叠加偏置、反相及通道路由之前，各路先有自己的幅度。
        double amp1 = DbfsToLinearPeak(config.LevelDbfs);
        double amp2 = DbfsToLinearPeak(config.EffectiveChannel2LevelDbfs);
        double ch2PhaseRad = config.Channel2PhaseOffsetDeg * (Math.PI / 180.0);
        if (config.InvertChannel2)
            ch2PhaseRad += Math.PI;
        double ch2Sign = config.InvertChannel2 ? -1.0 : 1.0;

        // 每个样本推进 2πf/Fs 弧度。例如 48 kHz 下输出 1 kHz，48 点恰好绕一圈。
        double phaseInc1 = 2.0 * Math.PI * Math.Clamp(config.FrequencyHz, 1.0, sampleRate * 0.49) / sampleRate;
        double phaseInc2 = 2.0 * Math.PI * Math.Clamp(config.SecondaryFrequencyHz, 1.0, sampleRate * 0.49) / sampleRate;

        switch (config.Waveform)
        {
            case GeneratorWaveform.SineVarPhase:
                // 把两路的相位设置施加在共同振荡时钟上，才能改变相对相位而不让两路频率随块边界漂移。
                var phase = config.WaveSettings;
                double offset = config.Channel2PhaseOffsetDeg*Math.PI/180;
                for (int i=0;i<count;i++)
                {
                    leftOut[i]=config.Channel1Enabled ? amp1*Math.Sin(_phasePrimary+(phase.PhaseCh1==GeneratedSignal.B ? offset : 0)) : 0;
                    rightOut[i]=config.Channel2Enabled ? amp2*ch2Sign*Math.Sin(_phasePrimary+(phase.PhaseCh2==GeneratedSignal.B ? offset : 0)) : 0;
                    _phasePrimary=WrapPhase(_phasePrimary+phaseInc1);
                }
                break;
            case GeneratorWaveform.Sine:
                // 普通双路正弦共用同一振荡器，右路另加相位偏移；每采样只推进一次相位，保持频率同步。
                for (int i = 0; i < count; i++)
                {
                    double s1 = amp1 * Math.Sin(_phasePrimary);
                    double s2 = amp2 * Math.Sin(_phasePrimary + ch2PhaseRad);
                    leftOut[i] = config.Channel1Enabled ? s1 : 0.0;
                    rightOut[i] = config.Channel2Enabled ? s2 : 0.0;
                    _phasePrimary = WrapPhase(_phasePrimary + phaseInc1);
                }
                break;

            case GeneratorWaveform.SineBurst:
                // 短音在高低电平间交替。包络位置决定当前档，载波相位继续前进，二者分别控制响度和瞬时波形。
                if (!_burstActive) { _burstPosition = 0; _phasePrimary = 0; _burstActive = true; }
                var burst = config.Burst ?? new SineBurstSettings();
                var (high, interval) = burst.SampleLengths(sampleRate,phaseInc1);
                double lowRatio = burst.LowMuted ? 0 : Math.Pow(10,Math.Clamp(burst.LowDb,-80,0)/20);
                for (int i=0; i<count; i++)
                {
                    double gain = _burstPosition < high ? 1 : lowRatio;
                    leftOut[i] = config.Channel1Enabled ? amp1 * gain * Math.Sin(_phasePrimary) : 0;
                    rightOut[i] = config.Channel2Enabled ? amp2 * gain * Math.Sin(_phasePrimary + ch2PhaseRad) : 0;
                    _phasePrimary = WrapPhase(_phasePrimary + phaseInc1);
                    _burstPosition++;
                    if (_burstPosition >= interval) _burstPosition %= interval;
                }
                break;

            case GeneratorWaveform.DualTone:
                // B 音相对 A 音的差值用 10^(差值/20) 换成线性比，实际分路或求和由双音通路决定。
                FillTwoTones(config,phaseInc1,phaseInc2,Math.Pow(10.0,config.DualToneSecondaryRatioDb/20.0),
                    config.DualToneMode,config.WaveSettings.FreqCh1,config.WaveSettings.FreqCh2,amp1,amp2,ch2PhaseRad,
                    leftOut[..count],rightOut[..count]);
                break;

            case GeneratorWaveform.Dim30:
            case GeneratorWaveform.Dim100:
            case GeneratorWaveform.DimB:
            case GeneratorWaveform.DimB8:
                // DIM 有方波与高频正弦两部分，路由可以把它们相加或分配到不同输出。采样率或波形类型改变时重建对应刺激。
                var dim=_dimStimulus;
                if (dim==null || dim.Waveform!=config.Waveform || dim.Rate!=sampleRate)
                    _dimStimulus=dim=new DimStimulus(config.Waveform,sampleRate);
                double DimSample(GeneratedSignal assignment,double phaseOffset)
                {
                    if (config.WaveSettings.ImdMode==DualToneRoutingMode.Sum)
                        return dim.Sample(_phasePrimary+phaseOffset,_phaseSecondary+phaseOffset);
                    return assignment==GeneratedSignal.A ? dim.SquareSample(_phasePrimary+phaseOffset) :
                        .25*Math.Sin(_phaseSecondary+phaseOffset);
                }
                for (int i = 0; i < count; i++)
                {
                    leftOut[i] = config.Channel1Enabled ? amp1 * DimSample(config.WaveSettings.ImdCh1,0) : 0;
                    rightOut[i] = config.Channel2Enabled ? amp2 * DimSample(config.WaveSettings.ImdCh2,ch2PhaseRad) : 0;
                    _phasePrimary = WrapPhase(_phasePrimary + Math.Tau * dim.SquareHz / sampleRate);
                    _phaseSecondary = WrapPhase(_phaseSecondary + Math.Tau * dim.SineHz / sampleRate);
                }
                break;

            case GeneratorWaveform.ImdSmpte:
            case GeneratorWaveform.ImdCcif:
                var imd=config.WaveSettings;
                // 互调测试需要两个实际可表示的频率，先核对当前采样率能否产生设定组合，再按幅度比合成。
                if (!imd.CanGenerateTones(config.Waveform,sampleRate))
                {
                    leftOut[..count].Clear();
                    rightOut[..count].Clear();
                    return;
                }
                var (f1,f2,ratio)=imd.Tones(config.Waveform);
                FillTwoTones(config,Math.Tau*f1/sampleRate,Math.Tau*f2/sampleRate,ratio,imd.ImdMode,
                    imd.ImdCh1,imd.ImdCh2,amp1,amp2,ch2PhaseRad,leftOut[..count],rightOut[..count]);
                break;

            case GeneratorWaveform.Square:
                // 用正弦的符号决定方波的正负电平，经过零点便翻转；这个离散生成方式会包含基波以外的谐波。
                for (int i = 0; i < count; i++)
                {
                    double s1 = Math.Sin(_phasePrimary) >= 0.0 ? amp1 : -amp1;
                    double s2 = Math.Sin(_phasePrimary + ch2PhaseRad) >= 0.0 ? amp2 : -amp2;
                    leftOut[i] = config.Channel1Enabled ? s1 : 0.0;
                    rightOut[i] = config.Channel2Enabled ? s2 : 0.0;
                    _phasePrimary = WrapPhase(_phasePrimary + phaseInc1);
                }
                break;

            case GeneratorWaveform.Multitone:
                // 每个音单独维护相位并相加。多音相长时峰值会超过单音，因此组合后按系数缩放并限幅。
                double toneNorm = 1.0 / (MultitoneFrequencies.Length * 0.55);
                Span<double> toneIncs = stackalloc double[MultitoneFrequencies.Length];
                for (int t = 0; t < MultitoneFrequencies.Length; t++)
                    toneIncs[t] = 2.0 * Math.PI * Math.Min(MultitoneFrequencies[t], sampleRate * 0.46) / sampleRate;

                for (int i = 0; i < count; i++)
                {
                    double sum = 0.0;
                    for (int t = 0; t < MultitoneFrequencies.Length; t++)
                    {
                        sum += Math.Sin(_multitonePhases[t]);
                        _multitonePhases[t] = WrapPhase(_multitonePhases[t] + toneIncs[t]);
                    }
                    double nVal = Math.Clamp(sum * toneNorm, -1.0, 1.0);
                    leftOut[i] = config.Channel1Enabled ? Math.Clamp(amp1 * nVal, -1.0, 1.0) : 0.0;
                    rightOut[i] = config.Channel2Enabled ? Math.Clamp(amp2 * nVal * ch2Sign, -1.0, 1.0) : 0.0;
                }
                break;

            case GeneratorWaveform.WhiteNoise:
                // 每个样本用一个均匀白噪声值同时送往左右路，因此两路在独立增益及极性之外高度相关；白噪声的长期功率谱密度近似平坦。
                for (int i = 0; i < count; i++)
                {
                    double w = 2.0 * _rng.NextDouble() - 1.0;
                    leftOut[i] = config.Channel1Enabled ? amp1 * w : 0.0;
                    rightOut[i] = config.Channel2Enabled ? amp2 * w * ch2Sign : 0.0;
                }
                break;

            case GeneratorWaveform.IecProgrammeNoise:
            case GeneratorWaveform.EnProgrammeNoise:
                // 节目噪声先构造有规定幅度和随机相位的频谱，再把连续噪声块重叠输出。Fs 改变时频格对应的 Hz 改变，需要重建生成器。
                if (_programmeNoise?.Rate != sampleRate) _programmeNoise = new(sampleRate,_rng);
                for (int i=0;i<count;i++)
                {
                    double v = _programmeNoise.Next(config.Waveform==GeneratorWaveform.EnProgrammeNoise);
                    leftOut[i] = config.Channel1Enabled ? amp1*v : 0;
                    rightOut[i] = config.Channel2Enabled ? amp2*v*ch2Sign : 0;
                }
                break;
            case GeneratorWaveform.PinkNoise:
                for (int i = 0; i < count; i++)
                {
                    // 白噪声通过多个不同衰减时间的递推分量后相加，使低频能量相对提高；跨块保留状态才能维持粉红噪声的谱形。
                    double white = 2.0 * _rng.NextDouble() - 1.0;
                    _b0 = 0.99886 * _b0 + white * 0.0555179;
                    _b1 = 0.99332 * _b1 + white * 0.0750759;
                    _b2 = 0.96900 * _b2 + white * 0.1538520;
                    _b3 = 0.86650 * _b3 + white * 0.3104856;
                    _b4 = 0.55000 * _b4 + white * 0.5329522;
                    _b5 = -0.7616 * _b5 - white * 0.0168980;
                    double pink = Math.Clamp((_b0 + _b1 + _b2 + _b3 + _b4 + _b5 + _b6 + white * 0.5362) * 0.11, -1.0, 1.0);
                    _b6 = white * 0.115926;

                    leftOut[i] = config.Channel1Enabled ? amp1 * pink : 0.0;
                    rightOut[i] = config.Channel2Enabled ? amp2 * pink * ch2Sign : 0.0;
                }
                break;

            case GeneratorWaveform.WavPlayback:
                count = FillFromWavPlayback(config, sampleRate, amp1, amp2, ch2Sign, leftOut, rightOut, count);
                break;
        }

        // 可选直流偏置与 24-bit TPDF 抖动（对齐 APx500 Generator DC Offset & Output Enable Dither）
        // 最后叠加每路直流偏移和可选微小抖动。两个独立均匀随机数之差得到三角形分布；启用通道才会收到这些附加量。
        double dc = Math.Clamp(config.DcOffsetFs, -0.9, 0.9);
        double dc2 = config.LinkChannelLevels ? dc : Math.Clamp(config.Channel2DcOffsetFs ?? dc,-0.9,0.9);
        bool addDc = Math.Abs(dc) > 1e-12 || Math.Abs(dc2) > 1e-12;
        bool addDither = config.EnableDither;
        if (addDc || addDither)
        {
            // 归一化的 24 位有符号 PCM 一个量化步长为 2⁻²³。两个独立均匀随机数作差，得到以零为中心的三角分布抖动。
            const double lsb24 = 1.0 / 8388608.0;
            for (int i = 0; i < count; i++)
            {
                if (config.Channel1Enabled)
                {
                    double d1 = addDither ? (_rng.NextDouble() - _rng.NextDouble()) * lsb24 : 0.0;
                    leftOut[i] = Math.Clamp(leftOut[i] + dc + d1, -1.0, 1.0);
                }
                if (config.Channel2Enabled)
                {
                    double d2 = addDither ? (_rng.NextDouble() - _rng.NextDouble()) * lsb24 : 0.0;
                    rightOut[i] = Math.Clamp(rightOut[i] + dc2 + d2, -1.0, 1.0);
                }
            }
        }
    }

    private int FillFromWavPlayback(
        GeneratorConfig config,
        int targetSampleRate,
        double amp1,
        double amp2,
        double ch2Sign,
        Span<double> leftOut,
        Span<double> rightOut,
        int count)
    {
        // 切换文件或多通道映射时刷新缓存；同一文件同一映射重复回调时可直接从内存取样。
        bool newFile=!string.Equals(_loadedWavPath, config.PlaybackWavFilePath,StringComparison.OrdinalIgnoreCase);
        int first=config.WaveSettings.FileCh1-1, second=config.WaveSettings.FileCh2-1;
        if (newFile || (_wavFileChannels>2 && (_cachedFirst!=first || _cachedSecond!=second)))
        {
            _loadedWavPath=config.PlaybackWavFilePath;
            if (newFile) { _wavActive=false; _wavFileChannels=0; }
            LastWavPlaybackError = null;

            if (string.IsNullOrWhiteSpace(_loadedWavPath) || !File.Exists(_loadedWavPath))
            {
                _wavLeft = Array.Empty<double>();
                _wavRight = Array.Empty<double>();
                LastWavPlaybackError = $"找不到发生器 WAV 文件：{_loadedWavPath ?? "(未设置)"}";
            }
            else
            {
                try
                {
                    if (newFile)
                    {
                        _wavFileChannels=ProjectFileService.AudioFileInfo(_loadedWavPath).Channels;
                    }
                    _cachedFirst=first; _cachedSecond=second;
                    (_wavLeft,_wavRight,_wavSampleRate,_)=_wavFileChannels<=2
                        ? ProjectFileService.ReadStereoWavWithMeta(_loadedWavPath)
                        : ProjectFileService.ReadStereoWavWithMeta(_loadedWavPath,Math.Clamp(first,0,_wavFileChannels-1),Math.Clamp(second,0,_wavFileChannels-1));
                    if (_wavLeft.Length == 0 || _wavSampleRate <= 0)
                    {
                        _wavLeft = Array.Empty<double>();
                        _wavRight = Array.Empty<double>();
                        LastWavPlaybackError = $"发生器 WAV 文件无有效音频样本：{_loadedWavPath}";
                    }
                }
                catch (Exception ex)
                {
                    _wavLeft = Array.Empty<double>();
                    _wavRight = Array.Empty<double>();
                    LastWavPlaybackError = $"读取发生器 WAV 文件失败：{ex.Message}";
                }
            }
        }

        // 读取失败或文件为空时输出静音，防止上一文件的最后一块数据被当作本次播放。
        int srcLen = _wavLeft.Length;
        if (srcLen == 0)
        {
            leftOut[..count].Clear();
            rightOut[..count].Clear();
            return 0;
        }
        // Mono goes to both outputs. For stereo, remapping uses the cached tracks and
        // leaves the playback cursor intact. Larger files cache just the two chosen tracks.
        var wavLeft=_wavFileChannels==2 && first>=1 ? _wavRight : _wavLeft;
        var wavRight=_wavFileChannels==2 && second==0 ? _wavLeft : _wavRight;
        bool firstFrame = !_wavActive || _requestedStart != config.WaveformStartSample;
        // 第一个回调或用户指定起点改变时才重设文件位置与播放遍数，正常流式回调沿用上一个块末端的小数游标。
        if (firstFrame)
        {
            _wavActive = true;
            _requestedStart = config.WaveformStartSample;
            _wavPosition = Math.Clamp(_requestedStart,0,srcLen-1);
            _wavPass = 1;
        }

        // 每个输出样本让文件位置前移 Fs文件/Fs设备。相同采样率直接按整数索引读取，不同时在小数位置插值。
        double step = (double)_wavSampleRate / Math.Max(targetSampleRate, 1);
        bool sameRate = _wavSampleRate == targetSampleRate;
        List<int>? starts = null;

        for (int i = 0; i < count; i++)
        {
            bool atFileStart = firstFrame && i == 0;
            // 越过文件末尾时计算跨过几遍，按循环模式与指定次数决定回绕还是补静音，时间轴上另行标记实际开头。
            if (_wavPosition >= srcLen)
            {
                int wraps = (int)(_wavPosition / srcLen);
                if (!config.LoopPlaybackWav && (long)_wavPass + wraps > Math.Max(1,config.WaveformPlayCount))
                {
                    leftOut[i..count].Clear();
                    rightOut[i..count].Clear();
                    FileStartOffsets = starts?.ToArray();
                    return i;
                }
                if (!config.LoopPlaybackWav) _wavPass += wraps;
                _wavPosition %= srcLen;
                atFileStart = true;
            }

            if (atFileStart) (starts ??= []).Add(i);
            double sampleL, sampleR;
            if (sameRate)
            {
                int idx = Math.Clamp((int)_wavPosition, 0, srcLen - 1);
                sampleL = wavLeft[idx];
                sampleR = wavRight[idx];
            }
            else
            {
                sampleL = SampleCubicHermite(wavLeft, _wavPosition, config.LoopPlaybackWav || _wavPass < config.WaveformPlayCount);
                sampleR = SampleCubicHermite(wavRight, _wavPosition, config.LoopPlaybackWav || _wavPass < config.WaveformPlayCount);
            }

            leftOut[i] = config.Channel1Enabled ? Math.Clamp(sampleL * amp1, -1.0, 1.0) : 0.0;
            rightOut[i] = config.Channel2Enabled ? Math.Clamp(sampleR * amp2 * ch2Sign, -1.0, 1.0) : 0.0;
            _wavPosition += step;
        }
        FileStartOffsets = starts?.ToArray();
        return count;
    }

    /// <summary>4 点 Catmull-Rom 三次 Hermite 插值重采样（保持频率、相位与时长精确，用于异速 WAV 播放与异速 Chirp 输出）。</summary>
    public static double[] ResampleCubicHermite(ReadOnlySpan<double> input, int sourceRate, int targetRate)
    {
        if (input.Length == 0 || sourceRate <= 0 || targetRate <= 0) return Array.Empty<double>();
        if (sourceRate == targetRate) return input.ToArray();

        // 目标长度按两种采样率的比例改变，使音频时长保持近似相同；对每个目标时刻反查源采样坐标。
        int outLen = Math.Max(1, (int)Math.Round(input.Length * (double)targetRate / sourceRate));
        var output = new double[outLen];
        double step = (double)sourceRate / targetRate;
        double pos = 0.0;

        for (int i = 0; i < outLen; i++)
        {
            output[i] = SampleCubicHermite(input, pos, loop: false);
            pos += step;
        }
        return output;
    }

    private static double SampleCubicHermite(ReadOnlySpan<double> data, double pos, bool loop)
    {
        int len = data.Length;
        int i1 = (int)Math.Floor(pos);
        double frac = pos - i1;

        // 文件循环时两侧取样可越过首尾接缝，不循环时重复端点，避免在 Hermite 插值所需的邻点上越界。
        int WrapIdx(int idx)
        {
            if (loop) return ((idx % len) + len) % len;
            return Math.Clamp(idx, 0, len - 1);
        }

        // 用中心两点确定区间端值，左右邻点估计端点斜率。四个样本即可在小数位置平滑插值；文件边界由前面的回绕或夹取规则处理。
        double y0 = data[WrapIdx(i1 - 1)];
        double y1 = data[WrapIdx(i1)];
        double y2 = data[WrapIdx(i1 + 1)];
        double y3 = data[WrapIdx(i1 + 2)];

        double c0 = y1;
        double c1 = 0.5 * (y2 - y0);
        double c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
        double c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
        return ((c3 * frac + c2) * frac + c1) * frac + c0;
    }

    /// <summary>重置所有振荡器相位、噪声滤波器与 WAV 播放游标。</summary>
    public void Reset()
    {
        // 重置振荡相位、粉红噪声递推和 WAV/短音起播标识。随机数生成器及节目噪声块仍属于当前对象，并未重新播种。
        ResetPattern(GeneratorWaveform.Sine);
        _phasePrimary = 0.0;
        _phaseSecondary = 0.0;
        Array.Copy(MultitoneInitialPhases, _multitonePhases, MultitoneInitialPhases.Length);
        _b0 = _b1 = _b2 = _b3 = _b4 = _b5 = _b6 = 0.0;
        _wavPosition = 0.0;
        _wavActive = _burstActive = false;
    }

    // 已有相位小于 2π 且每采样增量也小于 2π 时，一次减圈即可维持相位范围。
    private static double WrapPhase(double phase) => phase >= 2.0 * Math.PI ? phase - 2.0 * Math.PI : phase;

    private static double[] BuildSchroederPhases(int count)
    {
        // 给多音分配不同的初始相位，以减少所有频率同时相长的机会。改变这些相位会改变瞬时峰值，即使各音幅值相同。
        var phases = new double[count];
        for (int k = 0; k < count; k++)
            phases[k] = -Math.PI * k * (k - 1) / count;
        return phases;
    }

    // 输入是幅度相对数字满幅的 dBFS，例如 −6.02 dBFS 约为半幅，输出为线性峰值比例。
    public static double DbfsToLinearPeak(double dbfs) => Math.Pow(10.0, dbfs / 20.0);
    // 线性幅度用 20log10 转成 dBFS，低于可显示范围时取 −180 dBFS。
    public static double LinearToDbfs(double linear) => linear > 1e-9 ? 20.0 * Math.Log10(linear) : -180.0;
}
