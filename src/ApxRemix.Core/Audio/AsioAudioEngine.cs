using System.Buffers.Binary;
using System.Runtime.InteropServices;
using ApxRemix.Core.Dsp;
using ApxRemix.Core.Models;
using NAudio.Wave;
using NAudio.Wave.Asio;

namespace ApxRemix.Core.Audio;

/// <summary>ASIO full-duplex engine. Each physical channel has its own native sample type.</summary>
// ASIO 驱动的一次回调同时提供采集和播放缓冲。每个物理声道有自己的采样格式，
// 所以在各声道的字节边界做编解码，交给测量层的是同一采样时钟下的两条数值序列。
public sealed class AsioAudioEngine(bool capture = true, bool playback = true) : IAudioEngine
{
    private readonly SignalGeneratorDsp _generator = new();
    private readonly object _lifecycle = new();
    private GeneratorConfig _generatorConfig = new();
    private SignalPathEqualizer _outputEq = new();
    private SignalPathConfig? _path;
    private AsioDriver? _driver;
    private AsioCallbacks _callbacks;
    private AsioNativeBuffer[] _buffers = [];
    private GCHandle _pinnedBuffers;
    private bool _outputReady;
    private AsioChannelInfo[] _inputs = [], _outputs = [];
    private string? _openedName;
    private bool _buffersRequested, _startRequested;
    private SynchronizationContext? _ownerContext;
    private static readonly Dictionary<string, AsioAudioEngine> OpenDrivers = new(StringComparer.OrdinalIgnoreCase);
    private int _running;
    private int _input1, _input2, _output1, _output2;
    private AsioPcmFormat _inputFormat1, _inputFormat2;
    private AsioPcmFormat _outputFormat1, _outputFormat2;
    private int _frames;
    private int? _roundTripLatency;
    private byte[] _scratch = [], _silence = [];
    private double[] _drive1 = [], _drive2 = [];

    public AudioBackendMode BackendMode => AudioBackendMode.Asio;
    public bool IsRunning => Volatile.Read(ref _running) != 0;
    public int ActualSampleRate { get; private set; } = 48000;
    public int? InputBitDepth { get; private set; }
    public int OutputSampleRate => playback ? ActualSampleRate : 0;
    // One buffer can already be playing and its successor queued when a generator edit arrives.
    // 按样本数记录驱动链延迟。两块缓冲加驱动报告的输入、输出延迟，用于等待发生器的新设置传播到采集端。
    // 例如 48 kHz 时一块 256 帧耗时 256 / 48000 ≈ 5.33 ms，单是两块就约 10.67 ms。
    internal int GeneratorBufferDelaySamples => capture && playback ? 2 * _frames + (_roundTripLatency ?? 0) : 0;
    internal int SweepQuietLeadInSamples => capture && playback ? _frames : 0;
    public bool HasActiveOneShotExcitation => _generator.HasActiveOneShot;
    public string StatusDescription { get; private set; } = "ASIO (Idle)";

    public event Action<StereoAudioBlock>? AudioBlockCaptured;
    public event Action<string>? EngineStateChanged;

    public static IReadOnlyList<AudioDeviceInfo> EnumerateDrivers()
    {
        return AsioOut.GetDriverNames()
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .Select(name => new AudioDeviceInfo(name, name, AudioBackendMode.Asio, ChannelCount: 0)).ToArray();
    }

    /// <summary>Reads channel names without preparing buffers, or reuses the open driver's capabilities.</summary>
    public static (string[] Inputs, string[] Outputs) GetChannelNames(string driverName)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(driverName);
        lock (OpenDrivers)
        {
            // 通道列表也可能在正在测量时被查询，先复用活动驱动已缓存的硬件通道信息。
            // 仅查询离线驱动时临时初始化 COM 对象，读完输入和输出名称就释放。
            if (OpenDrivers.TryGetValue(driverName, out var open))
                return (open._inputs.Select(ch => ch.name).ToArray(),
                    open._outputs.Select(ch => ch.name).ToArray());

            var native = AsioDriver.GetAsioDriverByName(driverName);
            try
            {
                if (!native.Init(IntPtr.Zero))
                    throw new InvalidOperationException($"Failed to initialize {driverName}: {native.GetErrorMessage()}");
                native.GetChannels(out int inputs, out int outputs);
                return (Enumerable.Range(0, inputs).Select(i => native.GetChannelInfo(i, true).name).ToArray(),
                    Enumerable.Range(0, outputs).Select(i => native.GetChannelInfo(i, false).name).ToArray());
            }
            finally { native.ReleaseComAsioDriver(); }
        }
    }

    public void Start(SignalPathConfig pathConfig, GeneratorConfig initialGenerator)
    {
        lock (_lifecycle) StartDriver(pathConfig, initialGenerator);
    }

    private void StartDriver(SignalPathConfig pathConfig, GeneratorConfig initialGenerator)
    {
        Stop();
        // 保留创建驱动的线程上下文，供回调发现故障时调度关闭。新会话的发生器和均衡器由初始状态开始。
        _ownerContext = SynchronizationContext.Current;
        _path = pathConfig;
        _outputEq = new();
        _generator.Reset();
        UpdateGenerator(initialGenerator);
        string? name = pathConfig.AsioDriverName;
        if (string.IsNullOrWhiteSpace(name))
            throw new InvalidOperationException("请选择要使用的 ASIO 驱动。");

        int rate = pathConfig.SampleRate > 0 ? pathConfig.SampleRate : 48000;
        try
        {
            AsioDriver driver;
            lock (OpenDrivers)
            {
                if (OpenDrivers.ContainsKey(name))
                    throw new InvalidOperationException($"{name} already has an open ASIO stream.");
                driver = _driver = AsioDriver.GetAsioDriverByName(name);
                if (!driver.Init(IntPtr.Zero))
                    throw new InvalidOperationException($"Failed to initialize {name}: {driver.GetErrorMessage()}");
                // 激励相位、滤波系数和记录时长都依赖每秒样本数。
                // 先用驱动查询有效速率，再回读实际设置，防止按请求速率给另一时钟产生的数据标频。
                if (!driver.CanSampleRate(rate))
                {
                    int[] standardRates = [32000, 44100, 48000, 88200, 96000, 176400, 192000];
                    var supported = standardRates.Where(r => driver.CanSampleRate(r)).ToArray();
                    string alternatives = supported.Length > 0 ? $"Available sample rates: {string.Join(", ", supported)} Hz."
                        : "The driver reports no available standard sample rate. Check whether its device is connected and available.";
                    throw new InvalidOperationException($"{name} does not support {rate} Hz. {alternatives}");
                }
                if (driver.GetSampleRate() != rate) driver.SetSampleRate(rate);
                double reportedRate = driver.GetSampleRate();
                if (reportedRate != rate)
                    throw new InvalidOperationException($"{name} reports {reportedRate} Hz after requesting {rate} Hz.");
                ActualSampleRate = rate;
                driver.GetChannels(out int inputs, out int outputs);
                _inputs = Enumerable.Range(0, inputs).Select(i => driver.GetChannelInfo(i, true)).ToArray();
                _outputs = Enumerable.Range(0, outputs).Select(i => driver.GetChannelInfo(i, false)).ToArray();
                OpenDrivers.Add(name, this);
                _openedName = name;
            }

            // 每个逻辑通道映射到一个物理通道，再按该物理通道的原生格式换算。
            // 双输入位宽相同时可以报一个共同值，位宽不同时单值无法描述两路。
            var endpoints = new List<string>();
            if (capture)
            {
                (_input1, _input2) = pathConfig.ResolveAsioChannels(true, _inputs.Length);
                _inputFormat1 = AsioPcmFormat.From(_inputs[_input1].type);
                _inputFormat2 = AsioPcmFormat.From(_inputs[_input2].type);
                InputBitDepth = _inputFormat1.BitDepth == _inputFormat2.BitDepth ? _inputFormat1.BitDepth : null;
                endpoints.Add($"In {ChannelLabel(_inputs, _input1)} / {ChannelLabel(_inputs, _input2)}");
            }
            if (playback)
            {
                (_output1, _output2) = pathConfig.ResolveAsioChannels(false, _outputs.Length);
                _outputFormat1 = AsioPcmFormat.From(_outputs[_output1].type);
                _outputFormat2 = AsioPcmFormat.From(_outputs[_output2].type);
                endpoints.Add($"Out {ChannelLabel(_outputs, _output1)} / {ChannelLabel(_outputs, _output2)}");
            }

            // 驱动决定一块原生缓冲包含多少帧。所有已选通道共享这个时间长度，但单帧字节数可以不同。
            _buffers = AsioNativeBuffer.Build(capture, _input1, _input2, playback, _output1, _output2);
            driver.GetBufferSize(out _, out _, out _frames, out _);
            if (_frames <= 0) throw new InvalidOperationException($"{name} reported an invalid ASIO buffer size.");
            string? pendingFault = null;
            // 驱动可在启动过程中同步触发回调。这里同时记录第一个故障并通知运行态，供 Start 返回前再次核对。
            void OnFault(string reason)
            {
                Interlocked.CompareExchange(ref pendingFault, reason, null);
                StreamFailed(driver, reason);
            }
            _callbacks = new AsioCallbacks
            {
                pbufferSwitch = (bank, direct) => ProcessBuffers(driver, bank),
                pbufferSwitchTimeInfo = (time, bank, direct) => { ProcessBuffers(driver, bank); return IntPtr.Zero; },
                psampleRateDidChange = newRate =>
                {
                    if (newRate != rate) OnFault($"driver clock changed to {newRate} Hz; reopen the signal path");
                },
                pasioMessage = (selector, value, message, opt) => HandleMessage(selector, value, OnFault)
            };
            // CreateBuffers 会把两组原生指针写入固定地址的托管结构数组，固定期间 GC 可以安全移动其他对象。
            _pinnedBuffers = GCHandle.Alloc(_buffers, GCHandleType.Pinned);
            _buffersRequested = true;
            driver.CreateBuffers(_pinnedBuffers.AddrOfPinnedObject(), _buffers.Length, _frames, ref _callbacks);
            foreach (var buffer in _buffers)
                if (buffer.First == IntPtr.Zero || buffer.Second == IntPtr.Zero)
                    throw new InvalidOperationException($"{name} did not supply both buffers for channel {buffer.Channel + 1}.");
            // 只有驱动确认支持 OutputReady 才在回调结束通知它。临时空间按最宽的 float64 样本准备，
            // 每次仅复制对应通道有效字节。启动前两面输出缓冲都先置静音，避免播放初始化内容。
            _outputReady = playback && driver.OutputReady() == AsioError.ASE_OK;
            _scratch = new byte[checked(_frames * sizeof(double))];
            _silence = new byte[_scratch.Length];
            _drive1 = playback ? new double[_frames] : [];
            _drive2 = playback ? new double[_frames] : [];
            SilenceOutputs(0);
            SilenceOutputs(1);
            // 设备给出的延迟单位为帧或单通道样本数。只有两项都是有效非负值，才给上层提供往返延迟。
            string latency = driver.GetLatencies(out int inputLatency, out int outputLatency) == AsioError.ASE_OK
                ? $", latency {inputLatency} in / {outputLatency} out samples" : "";
            _roundTripLatency = latency.Length > 0 && inputLatency >= 0 && outputLatency >= 0
                ? checked(inputLatency + outputLatency) : null;
            StatusDescription = $"ASIO | {name} [{string.Join(", ", endpoints)} @ {rate} Hz, {_frames} frames{latency}]";
            Volatile.Write(ref _running, 1);
            if (Volatile.Read(ref pendingFault) is { } startError) throw new InvalidOperationException(startError);
            _startRequested = true;
            driver.Start();
            if (Volatile.Read(ref pendingFault) is { } driverError) throw new InvalidOperationException(driverError);
        }
        catch { Stop(); throw; }
    }

    // 内部索引从 0 起算，错误显示沿用用户看到的从 1 起算。无符号比较同时覆盖负索引和超出通道数的索引。
    internal static int CheckChannel(int index, int count, string direction)
    {
        if ((uint)index >= (uint)count)
            throw new InvalidOperationException($"Selected ASIO {direction} channel {index + 1} is unavailable (driver has {count}).");
        return index;
    }

    private static string ChannelLabel(AsioChannelInfo[] infos, int index) =>
        $"{index + 1} ({infos[index].name})";

    internal static int HandleMessage(AsioMessageSelector selector, int value, Action<string> fault)
    {
        // ASIO 可先询问客户端是否识别某条消息。重置、缓冲大小变化及重新同步会破坏当前流的时间或缓存假定，
        // 收到这些事件时结束本次测量路径，重开后才按新的通道和时钟参数解释数据。
        if (selector == AsioMessageSelector.kAsioSelectorSupported)
            return (AsioMessageSelector)value is AsioMessageSelector.kAsioEngineVersion or AsioMessageSelector.kAsioResetRequest
                or AsioMessageSelector.kAsioBufferSizeChange or AsioMessageSelector.kAsioResyncRequest ? 1 : 0;
        if (selector == AsioMessageSelector.kAsioEngineVersion) return 2;
        string? reason = selector switch
        {
            AsioMessageSelector.kAsioResetRequest => "driver requested a reset",
            AsioMessageSelector.kAsioBufferSizeChange => "driver buffer size changed",
            AsioMessageSelector.kAsioResyncRequest => "driver reported a stream discontinuity",
            _ => null
        };
        if (reason == null) return 0;
        fault($"{reason}; reopen the signal path");
        return 1;
    }

    private void ProcessBuffers(AsioDriver driver, int bank)
    {
        // ASIO 双缓冲轮换只有两面。回调给出的 bank 决定本次读写哪组指针，超出这个范围就终止本次流。
        if (bank is not (0 or 1))
        {
            StreamFailed(driver, $"invalid ASIO double-buffer index {bank}");
            return;
        }
        try
        {
            // 这一块输出由一个发生器配置快照生成。均衡在编码前作用于两路数值，随后才写入驱动指定的有效位和字节序。
            bool drive = playback && IsRunning;
            if (drive)
            {
                var config = Volatile.Read(ref _generatorConfig);
                _generator.FillBlock(config, ActualSampleRate, _drive1, _drive2);
                _outputEq.Process(_path?.OutputEq, ActualSampleRate, _drive1, _drive2);
            }

            // 原生缓冲按声道分开。每次读出一整条声道，或把同一时刻的发生器序列写入对应声道。
            // 输入和输出复用暂存区，单次编码或拷贝完成后才处理下一声道。
            double[]? left = null, right = null;
            foreach (var buffer in _buffers)
            {
                var pointer = buffer.Pointer(bank);
                if (buffer.IsInput == 1)
                {
                    if (!IsRunning) continue;
                    if (buffer.Channel == _input1) left = ReadChannel(pointer, _inputFormat1);
                    else right = ReadChannel(pointer, _inputFormat2);
                }
                else
                {
                    bool first = buffer.Channel == _output1;
                    var format = first ? _outputFormat1 : _outputFormat2;
                    if (drive)
                    {
                        var samples = first ? _drive1 : _drive2;
                        format.Encode(samples, _scratch);
                    }
                    Marshal.Copy(drive ? _scratch : _silence, 0, pointer, _frames * format.Bytes);
                }
            }
            // 两个逻辑输入映射到同一物理输入时，复制其数值帮助双通道接口继续工作，并把独立性标记设为 false。
            // 首次输出一次性激励时同时发出起点标记和延迟，供采集端把激励与返回波形放进同一样本时间轴。
            if (left != null && IsRunning)
                AudioBlockCaptured?.Invoke(new StereoAudioBlock(left, right ?? (double[])left.Clone(), ActualSampleRate, DateTime.UtcNow,
                    HasIndependentDualChannels: _input1 != _input2,
                    StartsAsioOneShot: drive && capture && _generator.StartedOneShotBlock,
                    AsioRoundTripLatency: _roundTripLatency));
        }
        catch (Exception ex)
        {
            // 回调失败后把当前面输出置静音，再把错误传出，驱动仍可在支持时收到当前面的完成通知。
            SilenceOutputs(bank);
            StreamFailed(driver, ex.Message);
        }
        finally
        {
            if (_outputReady) driver.OutputReady();
        }
    }

    // 这里支持的有符号 PCM 和 IEEE 浮点中，全零字节都表示静音。仅写输出，避免篡改驱动送来的输入。
    private void SilenceOutputs(int bank)
    {
        foreach (var buffer in _buffers)
        {
            if (buffer.IsInput == 1) continue;
            var format = buffer.Channel == _output1 ? _outputFormat1 : _outputFormat2;
            Marshal.Copy(_silence, 0, buffer.Pointer(bank), _frames * format.Bytes);
        }
    }

    // 复制的字节数等于帧数乘单样本容器宽度。解码后为每个时间点得到一个 double，返回的新数组可由接收方保存。
    private double[] ReadChannel(IntPtr pointer, AsioPcmFormat format)
    {
        Marshal.Copy(pointer, _scratch, 0, _frames * format.Bytes);
        var samples = new double[_frames];
        format.Decode(_scratch, samples);
        return samples;
    }

    private void StreamFailed(AsioDriver driver, string reason)
    {
        // 异步回调可能来自已关闭的驱动。先核对所有者，再用一次原子写只接收当前会话的首个停流原因。
        if (_driver != driver || Interlocked.Exchange(ref _running, 0) == 0) return;
        string status = $"ASIO endpoint stopped: {reason}";
        StatusDescription = status;
        // End the desktop session on its owning thread, outside the native callback.
        if (_ownerContext is { } context)
            context.Post(_ =>
            {
                lock (_lifecycle)
                {
                    if (_driver != driver) return;
                    CloseDriver();
                }
                EngineStateChanged?.Invoke(status);
            }, null);
        else
            ThreadPool.QueueUserWorkItem(_ => { if (_driver == driver && !IsRunning) EngineStateChanged?.Invoke(status); });
    }

    public void Stop()
    {
        lock (_lifecycle)
        {
            Interlocked.Exchange(ref _running, 0);
            CloseDriver();
            InputBitDepth = null;
            StatusDescription = "ASIO (Stopped)";
        }
        EngineStateChanged?.Invoke(StatusDescription);
    }

    private void CloseDriver()
    {
        if (_driver is { } driver)
        {
            _driver = null;
            // 释放从启动顺序反向进行。先停回调，再让驱动释放缓冲，最后释放 COM、固定数组与委托引用。
            if (_startRequested)
            {
                try { driver.Stop(); } catch { }
                _startRequested = false;
            }
            try
            {
                if (_buffersRequested) driver.DisposeBuffers();
            }
            finally
            {
                driver.ReleaseComAsioDriver();
                _buffersRequested = false;
                _outputReady = false;
                if (_pinnedBuffers.IsAllocated) _pinnedBuffers.Free();
                _callbacks = default;
                _buffers = [];
                if (_openedName != null)
                    lock (OpenDrivers) OpenDrivers.Remove(_openedName);
                _openedName = null;
            }
        }
    }

    // 发布配置副本，减小控制线程改写多个字段时回调读出混合设置的机会。真实输入来自设备，虚拟 DUT 参数对此无效。
    public void UpdateGenerator(GeneratorConfig config) => Volatile.Write(ref _generatorConfig, config.Clone());
    public void UpdateVirtualDut(VirtualDutConfig dutConfig) { }
    // 一次性激励排入发生器，前后静音和第二路时差按输出样本计数，在后续驱动回调里推进。
    public void EnqueueOneShotExcitation(double[] monoSamples, int preSilenceSamples = 0, int postSilenceSamples = 0,
        bool enableCh1 = true, bool enableCh2 = true, int secondChannelDelaySamples = 0, double[]? secondChannelSamples = null)
        => _generator.EnqueueOneShot(monoSamples, preSilenceSamples, postSilenceSamples, enableCh1, enableCh2, secondChannelDelaySamples, secondChannelSamples);
    public void ClearOneShotExcitation() => _generator.ClearOneShot();
    public void Dispose() => Stop();
}

// ASIOBufferInfo is packed on 4-byte boundaries on Windows. ASIOBool is a signed 32-bit value.
[StructLayout(LayoutKind.Sequential, Pack = 4)]
internal struct AsioNativeBuffer
{
    public int IsInput, Channel;
    public IntPtr First, Second;
    // 每个条目描述一个物理通道的两面缓冲。结构按 Windows 原生接口布局传递，标志和通道号是 32 位整数。
    public readonly IntPtr Pointer(int bank) => bank == 0 ? First : Second;

    internal static AsioNativeBuffer[] Build(bool capture, int input1, int input2, bool playback, int output1, int output2)
    {
        // 每个物理方向和通道组合只向驱动请求一次。两个逻辑通道共用物理索引时，复用同一块原生缓冲。
        var buffers = new List<AsioNativeBuffer>(4);
        if (capture)
        {
            buffers.Add(new() { IsInput = 1, Channel = input1 });
            if (input2 != input1) buffers.Add(new() { IsInput = 1, Channel = input2 });
        }
        if (playback)
        {
            buffers.Add(new() { Channel = output1 });
            if (output2 != output1) buffers.Add(new() { Channel = output2 });
        }
        if (buffers.Count == 0) throw new InvalidOperationException("No ASIO input or output selected.");
        return buffers.ToArray();
    }
}

/// <summary>Native planar ASIO PCM. Sub-width integer samples in a four-byte container are right aligned.</summary>
// Bytes 决定原生指针的步长，BitDepth 决定有符号整数的量化尺度。归一化结果是数字满量程的比例。
// 若要把 0.5 换成伏特，还要知道该设备和所选量程下满量程峰值对应的电压。
internal readonly record struct AsioPcmFormat(int Bytes, int BitDepth, bool Floating, bool BigEndian)
{
    public static AsioPcmFormat From(AsioSampleType type)
    {
        // 驱动枚举分别编码 MSB 在前和 LSB 在前的格式。四字节容器可只有 16、18、20 或 24 个有效位，
        // 此实现按低位对齐解读这些 ASIO 整数字，紧凑 24 位则每个样本只占三个字节。
        bool big = type < AsioSampleType.Int16LSB;
        return type switch
        {
            AsioSampleType.Int16MSB or AsioSampleType.Int16LSB => new(2, 16, false, big),
            AsioSampleType.Int24MSB or AsioSampleType.Int24LSB => new(3, 24, false, big),
            AsioSampleType.Int32MSB or AsioSampleType.Int32LSB => new(4, 32, false, big),
            AsioSampleType.Int32MSB16 or AsioSampleType.Int32LSB16 => new(4, 16, false, big),
            AsioSampleType.Int32MSB18 or AsioSampleType.Int32LSB18 => new(4, 18, false, big),
            AsioSampleType.Int32MSB20 or AsioSampleType.Int32LSB20 => new(4, 20, false, big),
            AsioSampleType.Int32MSB24 or AsioSampleType.Int32LSB24 => new(4, 24, false, big),
            AsioSampleType.Float32MSB or AsioSampleType.Float32LSB => new(4, 32, true, big),
            AsioSampleType.Float64MSB or AsioSampleType.Float64LSB => new(8, 64, true, big),
            _ => throw new NotSupportedException($"ASIO sample format {type} cannot carry PCM measurements.")
        };
    }

    public void Decode(ReadOnlySpan<byte> source, Span<double> samples)
    {
        // 令有效位数为 N，二补码整数的范围是 −2^(N−1) 到 2^(N−1)−1。除以 2^(N−1) 后，
        // 可表示的负端为 −1，正端差一个量化步长达到 +1。IEEE 浮点已经按数值存储，直接按字节序还原。
        int shift = 32 - BitDepth;
        double scale = Floating ? 1 : Math.ScaleB(1, BitDepth - 1);
        for (int i = 0; i < samples.Length; i++)
        {
            // 跳过前 i 个完整容器即可取出本声道第 i 个样本。所用跨度不包含其他声道的数据。
            var word = source.Slice(i * Bytes, Bytes);
            if (Floating && Bytes == 8)
                samples[i] = BigEndian ? BinaryPrimitives.ReadDoubleBigEndian(word) : BinaryPrimitives.ReadDoubleLittleEndian(word);
            else if (Floating)
                samples[i] = BigEndian ? BinaryPrimitives.ReadSingleBigEndian(word) : BinaryPrimitives.ReadSingleLittleEndian(word);
            else
            {
                int value = 0;
                // 按高字节到低字节拼成一个整数。左移把第 N 位移到 int 符号位，算术右移将该位复制回高位。
                // 例如 24 位小端字节 00 00 C0 拼成 0xC00000，符号扩展得到 −4194304，再除以 8388608 得到 −0.5。
                for (int j = 0; j < Bytes; j++) value = (value << 8) | word[BigEndian ? j : Bytes - 1 - j];
                samples[i] = ((value << shift) >> shift) / scale;
            }
        }
    }

    public void Encode(ReadOnlySpan<double> samples, Span<byte> destination)
    {
        double scale = Floating ? 1 : Math.ScaleB(1, BitDepth - 1);
        for (int i = 0; i < samples.Length; i++)
        {
            var word = destination.Slice(i * Bytes, Bytes);
            // 输出进入硬件前把非有限值改成静音并限制到数字满量程。浮点分支直接将幅度按对应字宽和字节序写出。
            double sample = double.IsFinite(samples[i]) ? Math.Clamp(samples[i], -1, 1) : 0;
            if (Floating && Bytes == 8)
            {
                if (BigEndian) BinaryPrimitives.WriteDoubleBigEndian(word, sample);
                else BinaryPrimitives.WriteDoubleLittleEndian(word, sample);
            }
            else if (Floating)
            {
                if (BigEndian) BinaryPrimitives.WriteSingleBigEndian(word, (float)sample);
                else BinaryPrimitives.WriteSingleLittleEndian(word, (float)sample);
            }
            else
            {
                // 整数分支乘满量程系数、舍入到最近码（平局取偶），再给正端设置最高可表示码。
                // 16 位时 +1 对应 32767，−1 对应 −32768。算术右移逐次取出二补码低字节，按大小端放进容器。
                int value = (int)Math.Min(scale - 1, Math.Round(sample * scale));
                for (int j = 0; j < Bytes; j++)
                {
                    word[BigEndian ? Bytes - 1 - j : j] = (byte)value;
                    value >>= 8;
                }
            }
        }
    }
}
