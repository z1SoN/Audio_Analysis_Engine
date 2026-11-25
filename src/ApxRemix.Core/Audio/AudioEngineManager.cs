// 从这里追踪真实数据流。Start 根据 SignalPathConfig 选择一个 IAudioEngine，OnEngineBlockCaptured 接过连续样本，一份进入环形缓冲供界面取最近窗口，一份送入有历史的实时表。
// 长扫频另有 LinearCaptureCollector 保存整次采集。环形缓冲会覆盖旧数据，适合刷新屏幕，完整扫频还要用到开头的响应。
using ApxRemix.Core.Models;
using ApxRemix.Core.Dsp;
using NAudio.Wave;

namespace ApxRemix.Core.Audio;

public readonly record struct AsioSweepStart(int Frame, int? Latency)
{
    // 预留静音至少覆盖驱动保护区。所有长度以样本计，直到解释为时间时才除以采样率。
    public static int QuietLeadIn(int requestedSamples, int guardSamples) => Math.Max(requestedSamples, guardSamples);

    public Range ResponseWindow(int preSilence, int responseLength, int retainedLeadIn = 0)
    {
        // Frame 记录输出首块对应的采集下标。加往返延迟得到返回首块的位置，再加前置静音得到响应起点。
        // 可回取一段静音作上下文，回取量最多等于计划中的前置静音，终点为响应起点加响应长度。
        int first = checked(Frame + (Latency ?? 0) + preSilence);
        return new Range(first - Math.Min(preSilence, retainedLeadIn), checked(first + responseLength));
    }
}

/// <summary>
/// 无损线性连续采集收集器（用于 Continuous Sweep / Acoustic Response 捕获完整长 Chirp 与尾部衰减）。
/// </summary>
public sealed class LinearCaptureCollector : IDisposable
{
    private readonly AudioEngineManager _owner;
    private readonly object _lock = new();
    private readonly List<double> _left = new(65536);
    private readonly List<double> _right = new(65536);
    private readonly List<double> _syncRef = new(65536);
    private bool _hasSyncRef;
    private bool _disposed;

    internal LinearCaptureCollector(AudioEngineManager owner)
    {
        _owner = owner;
        // 订阅连续块事件，把记录逐块追加到线性数组。扫频可能长于显示环形缓冲，保留开头才能配对完整激励和响应。
        _owner.AudioBlockCaptured += OnBlock;
    }

    public int CapturedSampleCount { get { lock (_lock) return _left.Count; } }
    private AsioSweepStart? _asioStart;
    public AsioSweepStart? AsioStart { get { lock (_lock) return _asioStart; } }

    private void OnBlock(StereoAudioBlock block)
    {
        lock (_lock)
        {
            if (_disposed) return;
            // 在追加前记下激励起点块在整段采集中的偏移，同时保留驱动报告的往返延迟。
            if (block.StartsAsioOneShot) _asioStart = new(_left.Count, block.AsioRoundTripLatency);
            _left.AddRange(block.Left);
            _right.AddRange(block.Right);
            if (block.ReferenceLeft is not null && block.ReferenceLeft.Length == block.Left.Length)
            {
                _hasSyncRef = true;
                _syncRef.AddRange(block.ReferenceLeft);
            }
        }
    }

    public (double[] Left, double[] Right, double[]? SyncRef) Snapshot()
    {
        lock (_lock)
        {
            // 快照复制出固定的时间序列。参考必须和采集逐样本等长才一起交付，否则对齐关系无法保证。
            return (_left.ToArray(), _right.ToArray(), _hasSyncRef && _syncRef.Count == _left.Count ? _syncRef.ToArray() : null);
        }
    }

    public void Dispose()
    {
        lock (_lock)
        {
            if (_disposed) return;
            _disposed = true;
        }
        _owner.AudioBlockCaptured -= OnBlock;
    }
}

/// <summary>
/// 音频引擎与实时采样环形缓冲区统一管理器。
/// 保留独立双声道身份、单调样本序号、WAV EOF 状态传播、一次性 Chirp 激励注入与长时长线性采集。
/// </summary>
public sealed class AudioEngineManager(bool useVirtualFixture = false) : IDisposable
{
    // 环形缓冲长度按单通道样本数计。48 kHz 时可保留 262144 / 48000 ≈ 5.46 秒最新波形。
    internal const int RingCapacity = 262144;

    private readonly double[] _ringLeft = new double[RingCapacity];
    private readonly double[] _ringRight = new double[RingCapacity];
    private readonly double[] _ringSyncRef = new double[RingCapacity];
    private readonly object _ringLock = new();
    private readonly object _rawRecordLock = new();

    private int _writeCursor;
    private int _validSampleCount;
    private long _totalSamplesCaptured;
    private bool _hasSyncReference;
    private readonly Queue<long> _generatorFileStarts = new();
    private bool _hasIndependentDualChannels = true;
    private ApxStreamingPhaseMeter? _streamingPhaseMeter;
    private StreamingBenchPipeline? _streamingBenchPipeline;
    private StreamingBenchPipeline? _panelMeters;
    private StreamingBenchPipeline? _recorderMeters;
    private Func<AudioCaptureSnapshot, bool>? _publishRecorderFrame;
    private Action<Exception?>? _completeRecorderFrames;
    private double _recorderReadingRateHz;
    private long _recorderMeterStart, _recorderDeliveredEnd;
    private bool _scanRecorderInput;
    public bool CapturePanelReadings { get; init; }
    private AnalyzerFilterConfig _liveAnalyzerFilter = new();
    private AnalyzerFilterConfig _panelFilter = new();
    private double _liveCh1GainDb;
    private double _liveCh2GainDb;
    private double _liveGeneratorFreqHz = 1000.0;
    private IAudioEngine? _activeEngine;

    private WaveFileWriter? _rawWavWriter;
    private long _rawRecordedSamples;

    public bool IsRunning => _activeEngine?.IsRunning == true;
    public AudioBackendMode? ActiveBackend => _activeEngine?.BackendMode;
    // 分离式路由和一体式引擎的设备 ID 来源不同，统一读取实际打开的端点供界面显示。
    public string? ActiveWasapiOutputDeviceId => _activeEngine switch
    {
        WasapiAudioEngine windows => windows.OpenOutputDeviceId,
        IndependentAudioRoute route => route.OpenWasapiOutputDeviceId,
        _ => null
    };
    public string? ActiveWasapiInputDeviceId => _activeEngine switch
    {
        WasapiAudioEngine windows => windows.OpenInputDeviceId,
        IndependentAudioRoute route => route.OpenWasapiInputDeviceId,
        _ => null
    };
    public int ActualSampleRate => _activeEngine?.ActualSampleRate ?? 48000;
    public int? InputBitDepth => _activeEngine?.InputBitDepth;
    public int OutputSampleRate => ActiveOutput == AudioOutputConnector.ExternalSource ? 0 : _activeEngine?.OutputSampleRate ?? ActualSampleRate;
    public bool HasActiveOneShotExcitation => _activeEngine?.HasActiveOneShotExcitation == true;
    public bool IsRawRecording { get { lock (_rawRecordLock) return _rawWavWriter is not null; } }
    public double RawRecordedSeconds { get { lock (_rawRecordLock) return _rawRecordedSamples / (double)Math.Max(ActualSampleRate, 1); } }
    public string? ActiveRawWavPath { get; private set; }
    public long TotalSamplesCaptured { get { lock (_ringLock) return _totalSamplesCaptured; } }
    public string StatusDescription => _activeEngine?.StatusDescription ?? "Idle";

    public event Action<StereoAudioBlock>? AudioBlockCaptured;
    public event Action<string>? EngineStateChanged;

    public static IReadOnlyList<AudioDeviceInfo> GetWasapiOutputDevices() => WasapiAudioEngine.EnumerateOutputDevices();
    public static IReadOnlyList<AudioDeviceInfo> GetWasapiInputDevices() => WasapiAudioEngine.EnumerateInputDevices();
    public static IReadOnlyList<AudioDeviceInfo> GetAsioDrivers() => AsioAudioEngine.EnumerateDrivers();

    // 数字字计量可单独启停，由接收音频块所用的锁保护，切换时重新开始累计。
    private DigitalWordMeter? _digitalWords;
    public bool MonitorDigitalWords
    {
        set { lock (_ringLock) { if (value) _digitalWords ??= new(); else _digitalWords=null; } }
    }
    public DigitalWordReading DigitalReading { get { lock (_ringLock) return _digitalWords?.Reading ?? default; } }

    private SignalPathEqualizer _inputEqualizer = new();
    private SignalPathConfig? _capturePath;
    public void Start(SignalPathConfig pathConfig, GeneratorConfig generatorConfig) =>
        Start(pathConfig, generatorConfig, null);

    public void Start(SignalPathConfig pathConfig, GeneratorConfig generatorConfig, Action? beforeInput)
    {
        // 虚拟 DUT 需要成对的模拟输入与输出，并由调用方显式启用。
        AudioOutputConnector output = pathConfig.EffectiveOutput;
        if ((pathConfig.BackendMode == AudioBackendMode.VirtualSimulator) != (output == AudioOutputConnector.VirtualLoopback))
            throw new InvalidOperationException("The virtual DUT input and output form one simulated loop.");
        if (!useVirtualFixture && pathConfig.BackendMode == AudioBackendMode.VirtualSimulator)
            throw new InvalidOperationException("The virtual DUT requires an explicitly enabled simulation.");
        ValidateDimOutput(OutputBackend(output), generatorConfig, pathConfig.SampleRate > 0 ? pathConfig.SampleRate : 48000);
        Stop();
        ClearBuffer();
        _capturePath=pathConfig;
        _inputEqualizer=new();

        // 切换数据来源有一个集中入口。从这里任选一个具体类跟到 AudioBlockCaptured，回来时都走同一个 OnEngineBlockCaptured，因此上层无需为每个设备再写一套表头。
        _activeOutput = output;
        _activeEngine = (pathConfig.BackendMode, output) switch
        {
            (AudioBackendMode.VirtualSimulator, AudioOutputConnector.VirtualLoopback) when useVirtualFixture => new VirtualAudioEngine(),
            (AudioBackendMode.Wasapi, AudioOutputConnector.Wasapi) => new WasapiAudioEngine(),
            (AudioBackendMode.Asio, AudioOutputConnector.Asio) => new AsioAudioEngine(capture:true, playback:true),
            (AudioBackendMode.WavFile, AudioOutputConnector.ExternalSource) => new WavFileAudioEngine(),
            _ => new IndependentAudioRoute(pathConfig.BackendMode switch {
                AudioBackendMode.Asio => new AsioAudioEngine(capture:true, playback:false),
                AudioBackendMode.Wasapi => new WasapiAudioEngine(playback:false),
                AudioBackendMode.WavFile => new WavFileAudioEngine(),
                _ => throw new ArgumentOutOfRangeException(nameof(pathConfig.BackendMode))
            }, output switch {
                AudioOutputConnector.Wasapi => new WasapiAudioEngine(capture:false),
                AudioOutputConnector.Asio => new AsioAudioEngine(capture:false, playback:true),
                AudioOutputConnector.ExternalSource => null,
                _ => throw new ArgumentOutOfRangeException(nameof(pathConfig.OutputConnector))
            })
        };
        _activeEngine.AudioBlockCaptured += OnEngineBlockCaptured;
        _activeEngine.EngineStateChanged += OnEngineStateChanged;
        try
        {
            beforeInput?.Invoke();
            _activeEngine.Start(pathConfig, generatorConfig);
        }
        catch { Stop(); throw; }
    }

    private static AudioBackendMode? OutputBackend(AudioOutputConnector output) => output switch
    {
        AudioOutputConnector.Wasapi => AudioBackendMode.Wasapi,
        AudioOutputConnector.Asio => AudioBackendMode.Asio,
        AudioOutputConnector.VirtualLoopback => AudioBackendMode.VirtualSimulator,
        _ => null
    };

    public AudioOutputConnector? ActiveOutput => _activeEngine == null ? null : _activeOutput;
    // 外部激励表示这个进程不向被测输入发送波形。WAV 输入的时间轴也独立于当前发生器。
    public bool CanDriveLiveInput => _activeEngine != null && ActiveOutput != AudioOutputConnector.ExternalSource && ActiveBackend != AudioBackendMode.WavFile;
    public bool IsGeneratorLoopback => _activeEngine is VirtualAudioEngine && CanDriveLiveInput;
    public int AsioSweepGuardSamples => (_activeEngine as AsioAudioEngine)?.SweepQuietLeadInSamples ?? 0;
    public int GeneratorBufferDelaySamples => ActiveOutput == AudioOutputConnector.Asio ? _activeEngine switch
    {
        AsioAudioEngine device => device.GeneratorBufferDelaySamples,
        _ => 0
    } : 0;
    private AudioOutputConnector _activeOutput;

    public void Stop()
    {
        StopRawWavRecording();
        CloseRecorderCapture(new InvalidOperationException("Acquisition stopped."));
        if (_activeEngine is not null)
        {
            // 先解除事件订阅，再停止并释放旧引擎。下次 Start 建立的是新会话，旧回调不应继续推进新会话的读数。
            _activeEngine.AudioBlockCaptured -= OnEngineBlockCaptured;
            _activeEngine.EngineStateChanged -= OnEngineStateChanged;
            _activeEngine.Stop();
            _activeEngine.Dispose();
            _activeEngine = null;
        }
    }

    // DIM 方波与高频正弦对输出有特定要求。虚拟夹具检查采样率的奈奎斯特上限，普通软件声卡输出拒绝这个专用激励。
    private static void ValidateDimOutput(AudioBackendMode? backend, GeneratorConfig generator, int rate)
    {
        if (!generator.IsEnabled || !DimStimulus.IsDim(generator.Waveform)) return;
        if (backend == AudioBackendMode.VirtualSimulator && !DimStimulus.SupportsRate(generator.Waveform, rate))
            throw new InvalidOperationException("The sample rate must exceed twice the DIM sine frequency.");
        if (backend is AudioBackendMode.Asio or AudioBackendMode.Wasapi)
            throw new InvalidOperationException("DIM excitation requires an analog square-wave generator.");
    }

    public void UpdateGenerator(GeneratorConfig config)
    {
        ValidateDimOutput(ActiveOutput is {} connector ? OutputBackend(connector) : null, config, OutputSampleRate);
        lock (_ringLock)
        {
            // 发生器频率同时可供分析侧带通跟踪。先在锁内刷新实时表配置，再把发生器配置交给输出引擎。
            _liveGeneratorFreqHz = config.FrequencyHz;
            _streamingBenchPipeline?.Configure(_liveAnalyzerFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
            _panelMeters?.Configure(_panelFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
            _recorderMeters?.Configure(_panelFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
        }
        _activeEngine?.UpdateGenerator(config);
    }

    public void UpdateVirtualDut(VirtualDutConfig dutConfig) => _activeEngine?.UpdateVirtualDut(dutConfig);

    /// <summary>热更新实时连续 Bench 测量链的信号路径滤波、带通与输入增益微调配置。</summary>
    public void UpdateAnalyzerState(AnalyzerFilterConfig filter, double ch1GainDb, double ch2GainDb, double generatorFreqHz)
        => UpdateAnalyzerState(filter, ch1GainDb, ch2GainDb, (double?)generatorFreqHz);

    public void UpdateAnalyzerState(AnalyzerFilterConfig filter, double ch1GainDb, double ch2GainDb, double? generatorFreqHz)
        => UpdateAnalyzerState(filter, ch1GainDb, ch2GainDb, generatorFreqHz, null);

    public void UpdateAnalyzerState(AnalyzerFilterConfig filter, double ch1GainDb, double ch2GainDb, double? generatorFreqHz,
        double? panelFixedBandpassHz)
    {
        lock (_ringLock)
        {
            _liveAnalyzerFilter = filter.Clone();
            // 面板与实时采集共享多数配置。面板若还需要记录另一个固定中心频率，用单独副本保存这一字段。
            _panelFilter = _liveAnalyzerFilter;
            if (CapturePanelReadings && panelFixedBandpassHz is double fixedHz && fixedHz != filter.BandpassCenterHz)
            {
                // The native Panel compares the stored fixed frequency even while
                // the generator or measured frequency supplies the active center.
                _panelFilter = filter.Clone();
                _panelFilter.BandpassCenterHz = fixedHz;
            }
            _liveCh1GainDb = ch1GainDb;
            _liveCh2GainDb = ch2GainDb;
            if (generatorFreqHz is double frequency) _liveGeneratorFreqHz = frequency;
            _streamingBenchPipeline?.Configure(_liveAnalyzerFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
            _panelMeters?.Configure(_panelFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
            _recorderMeters?.Configure(_panelFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
        }
    }

    /// <summary>向实时连续 Bench 链推送 APxRT 双二阶输入 EQ 级联系数。</summary>
    public void SetRealtimeInputEq(int order, double[] a1, double[] a2,
        double[] b1, double[] b2, double gain, int slot, bool on1, bool on2)
    {
        lock (_ringLock)
        {
            _streamingBenchPipeline?.SetEqFilter(order, a1, a2, b1, b2, gain, slot, on1, on2);
            _panelMeters?.SetEqFilter(order, a1, a2, b1, b2, gain, slot, on1, on2);
            _recorderMeters?.SetEqFilter(order, a1, a2, b1, b2, gain, slot, on1, on2);
        }
    }

    public long BeginRecorderCapture(double readingRateHz, bool scanInput,
        Func<AudioCaptureSnapshot, bool> publish, Action<Exception?> complete)
    {
        lock (_ringLock)
        {
            _recorderMeters = null;
            _recorderReadingRateHz = readingRateHz;
            _recorderDeliveredEnd = _totalSamplesCaptured;
            _scanRecorderInput = scanInput;
            _publishRecorderFrame = publish;
            _completeRecorderFrames = complete;
            if (_activeEngine?.HasReachedEof == true) CloseRecorderCapture(null);
            return _totalSamplesCaptured;
        }
    }

    public void SetRecorderReadingRate(double readingRateHz)
    {
        lock (_ringLock)
        {
            _recorderReadingRateHz = readingRateHz;
            _recorderMeters?.SetRecorderReadingRate(readingRateHz);
        }
    }

    public void StopRecorderTriggerScan()
    {
        lock (_ringLock) _scanRecorderInput = false;
    }

    public void EndRecorderCapture() => CloseRecorderCapture(null);

    private void CloseRecorderCapture(Exception? error)
    {
        lock (_ringLock)
        {
            _completeRecorderFrames?.Invoke(error);
            _completeRecorderFrames = null;
            _publishRecorderFrame = null;
            _recorderMeters = null;
        }
    }

    public void EnqueueOneShotExcitation(
        double[] monoSamples,
        int preSilenceSamples = 0,
        int postSilenceSamples = 0,
        bool enableCh1 = true,
        bool enableCh2 = true,
        int secondChannelDelaySamples = 0,
        double[]? secondChannelSamples = null)
    {
        // 一次性激励必须有本机可驱动的输出。其长度、静音和两路延时均按输出样本计。
        if (ActiveOutput == AudioOutputConnector.ExternalSource)
            throw new InvalidOperationException("External source has no generator output.");
        _activeEngine?.EnqueueOneShotExcitation(monoSamples, preSilenceSamples, postSilenceSamples, enableCh1, enableCh2, secondChannelDelaySamples, secondChannelSamples);
    }

    public void ClearOneShotExcitation() => _activeEngine?.ClearOneShotExcitation();

    /// <summary>创建长时长无截断线性采集收集器（供 Continuous Sweep / Acoustic Response 使用）。</summary>
    public LinearCaptureCollector BeginLinearCapture() => new(this);

    public void StartRawWavRecording(string filePath)
    {
        StopRawWavRecording();
        string? dir = Path.GetDirectoryName(filePath);
        if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
        lock (_rawRecordLock)
        {
            // 用实际输入采样率和两路 IEEE float32 写 WAV。保存的是进入管理器后应用输入均衡的样本，编码会把 double 转成 float。
            _rawWavWriter = new WaveFileWriter(filePath, WaveFormat.CreateIeeeFloatWaveFormat(ActualSampleRate, 2));
            _rawRecordedSamples = 0;
            ActiveRawWavPath = filePath;
        }
    }

    public string? StopRawWavRecording()
    {
        lock (_rawRecordLock)
        {
            string? path = ActiveRawWavPath;
            if (_rawWavWriter is not null)
            {
                // 结束时在写入锁内刷新并关闭文件，让 WAV 头的数据区域长度与已写帧数对应。
                _rawWavWriter.Flush();
                _rawWavWriter.Dispose();
                _rawWavWriter = null;
            }
            ActiveRawWavPath = null;
            return path;
        }
    }

    public (double[] Left, double[] Right) CaptureLatestSnapshot(int sampleCount)
    {
        var snap = CaptureLatestSnapshotWithMeta(sampleCount);
        return (snap.Left, snap.Right);
    }

    public AudioCaptureSnapshot CaptureLatestSnapshotWithMeta(int sampleCount)
    {
        lock (_ringLock)
            return CaptureSnapshotAt(sampleCount, _totalSamplesCaptured, ActualSampleRate);
    }

    private AudioCaptureSnapshot CaptureSnapshotAt(int sampleCount, long endSample, int sampleRate)
    {
        // A Recorder report can land inside the most recent driver callback.
        int trailing = (int)Math.Clamp(_totalSamplesCaptured - endSample, 0, RingCapacity);
        int count = Math.Clamp(sampleCount, 0, Math.Max(0, _validSampleCount - trailing));
        if (count <= 0)
        {
            return new AudioCaptureSnapshot(
                Array.Empty<double>(), Array.Empty<double>(), null,
                endSample, sampleRate, IsRunning, _hasIndependentDualChannels);
        }

        var left = new double[count];
        var right = new double[count];
        double[]? syncRef = _hasSyncReference ? new double[count] : null;

        // 若窗口跨过数组末尾，分两次复制，将样本按时间从早到晚交付。
        int start = (_writeCursor - trailing - count + RingCapacity) % RingCapacity;
        int firstPart = Math.Min(count, RingCapacity - start);
        int secondPart = count - firstPart;

        Array.Copy(_ringLeft, start, left, 0, firstPart);
        Array.Copy(_ringRight, start, right, 0, firstPart);
        if (syncRef is not null) Array.Copy(_ringSyncRef, start, syncRef, 0, firstPart);

        if (secondPart > 0)
        {
            Array.Copy(_ringLeft, 0, left, firstPart, secondPart);
            Array.Copy(_ringRight, 0, right, firstPart, secondPart);
            if (syncRef is not null) Array.Copy(_ringSyncRef, 0, syncRef, firstPart, secondPart);
        }

        return new AudioCaptureSnapshot(
            left, right, syncRef, endSample, sampleRate, IsRunning,
            _hasIndependentDualChannels, _streamingPhaseMeter?.Latest, _streamingBenchPipeline?.Latest,
            _generatorFileStarts.Count == 0 ? null : _generatorFileStarts.Where(i => i >= endSample - count && i < endSample).ToArray(), _digitalWords?.Reading)
            { PanelReading = _panelMeters?.Latest };
    }

    public void ClearBuffer()
    {
        lock (_ringLock)
        {
            // 新会话同时丢弃历史采样和历史检测器。否则即便数组清空，旧滤波状态仍会影响首批表值。
            Array.Clear(_ringLeft);
            Array.Clear(_ringRight);
            Array.Clear(_ringSyncRef);
            _writeCursor = 0;
            _validSampleCount = 0;
            _totalSamplesCaptured = 0;
            if (_digitalWords!=null) _digitalWords=new();
            _hasSyncReference = false;
            _generatorFileStarts.Clear();
            _hasIndependentDualChannels = true;
            _streamingPhaseMeter = null;
            _streamingBenchPipeline = null;
            _panelMeters = null;
        }
    }

    private void OnEngineStateChanged(string status)
    {
        lock (_ringLock)
        {
            if (_activeEngine?.HasReachedEof == true)
            {
                _panelMeters?.CompleteInput();
                _recorderMeters?.CompleteInput(PublishRecorderReading);
                CloseRecorderCapture(null);
            }
            else if (_activeEngine?.IsRunning == false)
                CloseRecorderCapture(new InvalidOperationException("Acquisition stopped."));
        }
        EngineStateChanged?.Invoke(status);
    }

    private void PublishRecorderReading(StreamingBenchReading reading)
    {
        long end = _recorderMeterStart + reading.EndSampleIndex;
        int waveformLength = Math.Clamp(reading.ReportSampleCount, 256, 16384);
        if (_scanRecorderInput)
            waveformLength = (int)Math.Min(RingCapacity, Math.Max(waveformLength, end - _recorderDeliveredEnd));
        var frame = CaptureSnapshotAt(waveformLength, end, reading.SampleRate) with { BenchReading = reading };
        if (_publishRecorderFrame?.Invoke(frame) == true) _recorderDeliveredEnd = end;
    }

    // 从声卡跟读一次测量，可以把断点放在这里。ASIO 路径上，AsioPcmFormat.Decode 已把
    // 每个通道的字节变成随时间排列的满量程比例；此时没有一个样本单独代表“这段声音的 RMS”。
    // 若关心 Bench 表头如何响应电平突变，往下跟 StreamingBenchPipeline.Process，看检波器怎样逐点更新。
    // 若关心 Sequence 怎样从记录算失真，可跟到 MainViewModel.SequenceReadings.cs 的
    // ReadSequenceToneAsync。它用 BenchFrameCapture.ReadAsync 订阅这里发出的 AudioBlockCaptured 事件；
    // 凑齐所需 PCM 后，AnalyzeWorksheet 再对这段记录分析基波和残差。
    private void OnEngineBlockCaptured(StereoAudioBlock block)
    {
        int len = Math.Min(block.Left.Length, block.Right.Length);
        if (len <= 0) return;

        lock (_ringLock)
        {
            // 数字字统计先取原始解码块，保留输入码的特征。随后在浮点样本上应用输入均衡，后续 FFT、实时表与录盘都读取均衡后的数据。
            _digitalWords?.Add(block);
            _inputEqualizer.Process(_capturePath?.InputEq,block.SampleRate,block.Left,block.Right);
            // 单个块超过显示缓冲时只保留最后 RingCapacity 个样本，实时表仍依时间顺序处理整个块。
            int srcOffset = len > RingCapacity ? len - RingCapacity : 0;
            int copyLen = Math.Min(len, RingCapacity);
            int firstPart = Math.Min(copyLen, RingCapacity - _writeCursor);
            int secondPart = copyLen - firstPart;

            Array.Copy(block.Left, srcOffset, _ringLeft, _writeCursor, firstPart);
            Array.Copy(block.Right, srcOffset, _ringRight, _writeCursor, firstPart);

            // 发生器标记原先以块为单位定位。加上已经接收的帧数变为会话内绝对下标，随后清除已被环形缓冲覆盖的标记。
            if (block.FileStartOffsets is {} starts)
                foreach (int offset in starts) _generatorFileStarts.Enqueue(_totalSamplesCaptured + offset);
            while (_generatorFileStarts.TryPeek(out long mark) && mark < _totalSamplesCaptured + len - RingCapacity)
                _generatorFileStarts.Dequeue();
            bool blockHasRef = block.ReferenceLeft is not null && block.ReferenceLeft.Length >= len;
            _hasSyncReference = blockHasRef;
            _hasIndependentDualChannels = block.HasIndependentDualChannels;
            if (blockHasRef) Array.Copy(block.ReferenceLeft!, srcOffset, _ringSyncRef, _writeCursor, firstPart);

            if (secondPart > 0)
            {
                Array.Copy(block.Left, srcOffset + firstPart, _ringLeft, 0, secondPart);
                Array.Copy(block.Right, srcOffset + firstPart, _ringRight, 0, secondPart);
                if (blockHasRef) Array.Copy(block.ReferenceLeft!, srcOffset + firstPart, _ringSyncRef, 0, secondPart);
            }

            // 数组下标到了末端会绕回，不能单靠下标判断快照新旧。下面的累计采样数另行向前增长，消费者可用它区分重画同一帧和收到新帧。
            _writeCursor = (_writeCursor + copyLen) % RingCapacity;
            _validSampleCount = Math.Min(RingCapacity, _validSampleCount + copyLen);
            _totalSamplesCaptured += len;
            if (block.SampleRate >= 8000)
            {
                // 相位表的时间尺度依赖采样率。相同采样率沿用历史，采样率改变后从新的状态计算。
                if (_streamingPhaseMeter?.SampleRate != block.SampleRate)
                    _streamingPhaseMeter = new(block.SampleRate);
                _streamingPhaseMeter.Process(block.Left.AsSpan(0, len), block.Right.AsSpan(0, len));

                // 声卡以实际采样率送来一块数据。滤波系数和每次读数的样本数都依赖这个速率。
                // 同一采样率下复用这一台表，下一次回调才能接着上一次的滤波状态算。
                if (_streamingBenchPipeline?.SampleRate != block.SampleRate)
                {
                    _streamingBenchPipeline = new StreamingBenchPipeline(block.SampleRate, _liveAnalyzerFilter);
                    _streamingBenchPipeline.Configure(_liveAnalyzerFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
                }
                _streamingBenchPipeline.Process(block.Left.AsSpan(0, len), block.Right.AsSpan(0, len));
                // 面板以独立状态消费同一组输入样本，用面板的更新节奏推进检测器。采集仪表的读数刷新不会替它推进。
                if (CapturePanelReadings)
                {
                    // Both consumers receive the same calibrated PCM. Reading the Panel
                    // advances detector state at 4 Hz, independently of acquisition meters.
                    if (_panelMeters?.SampleRate != block.SampleRate)
                    {
                        _panelMeters = new(block.SampleRate, _panelFilter, StreamingBenchPipeline.PanelReadingsPerSecond);
                        _panelMeters.Configure(_panelFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
                    }
                    _panelMeters.Process(block.Left.AsSpan(0, len), block.Right.AsSpan(0, len));
                }
                if (_publishRecorderFrame is not null)
                {
                    if (_recorderMeters?.SampleRate != block.SampleRate)
                    {
                        _recorderMeterStart = _totalSamplesCaptured - len;
                        _recorderMeters = StreamingBenchPipeline.ForRecorder(block.SampleRate, _panelFilter, _recorderReadingRateHz);
                        _recorderMeters.Configure(_panelFilter, _liveCh1GainDb, _liveCh2GainDb, _liveGeneratorFreqHz);
                    }
                    _recorderMeters.Process(block.Left.AsSpan(0, len), block.Right.AsSpan(0, len), PublishRecorderReading);
                    if (_scanRecorderInput && _recorderDeliveredEnd < _totalSamplesCaptured)
                    {
                        int scanLength = (int)Math.Min(RingCapacity, _totalSamplesCaptured - _recorderDeliveredEnd);
                        var scan = CaptureSnapshotAt(scanLength, _totalSamplesCaptured, block.SampleRate) with { BenchReading = null };
                        if (_publishRecorderFrame.Invoke(scan)) _recorderDeliveredEnd = _totalSamplesCaptured;
                    }
                }
            }
        }

        lock (_rawRecordLock)
        {
            if (_rawWavWriter is not null)
            {
                // WAV 写入顺序是 L0、R0、L1、R1。计时只加 len 帧，两声道的两个浮点数仍属于同一采样时刻。
                var interleaved = new float[len * 2];
                for (int i = 0; i < len; i++)
                {
                    interleaved[i * 2] = (float)block.Left[i];
                    interleaved[i * 2 + 1] = (float)block.Right[i];
                }
                _rawWavWriter.WriteSamples(interleaved, 0, interleaved.Length);
                _rawRecordedSamples += len;
            }
        }

        // 转发时释放内部锁，连续采集者可保存完整样本块，并通过会话样本计数定位窗口。
        AudioBlockCaptured?.Invoke(block);
    }

    public void Dispose() => Stop();
}
