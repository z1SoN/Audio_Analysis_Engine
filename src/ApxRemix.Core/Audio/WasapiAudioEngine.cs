// Windows 端点给的是带格式的音频缓冲。此处连接录放设备、解码样本并报告实际采样率；频率计算要用设备实际送来的每秒点数。
using System.Buffers.Binary;
using System.Runtime.InteropServices;
using ApxRemix.Core.Dsp;
using ApxRemix.Core.Models;
using NAudio.CoreAudioApi;
using NAudio.Dmo;
using NAudio.Wave;

namespace ApxRemix.Core.Audio;

/// <summary>Windows WASAPI 真实声卡全双工 I/O 引擎（支持真实声道数上报与一次性 Chirp 播放）。</summary>
public sealed class WasapiAudioEngine(bool capture = true, bool playback = true) : IAudioEngine
{
    // WASAPI 共享模式的端点缓冲按帧交错。录音端的时钟决定帧的时间间隔，数字回环和物理输入共用解码流程。
    private const string LoopbackPrefix = "loopback:";
    private EndpointSamples? _captureSamples;
    private int _running;

    private readonly SignalGeneratorDsp _generator = new();
    private readonly object _configLock = new();
    private GeneratorConfig _generatorConfig = new();

    private WasapiOut? _wasapiOut;
    private WasapiCapture? _wasapiCapture;
    private MMDevice? _outputDevice;
    private MMDevice? _inputDevice;

    public AudioBackendMode BackendMode => AudioBackendMode.Wasapi;
    public bool IsRunning => Volatile.Read(ref _running) != 0;
    public int ActualSampleRate { get; private set; } = 48000;
    public int OutputSampleRate { get; private set; } = 48000;
    internal string? OpenOutputDeviceId => _outputDevice?.ID;
    internal string? OpenInputDeviceId => _inputDevice is null ? null :
        _wasapiCapture is WasapiLoopbackCapture ? LoopbackPrefix + _inputDevice.ID : _inputDevice.ID;
    public int? InputBitDepth { get; private set; }
    public bool HasActiveOneShotExcitation => _generator.HasActiveOneShot;
    public string StatusDescription { get; private set; } = "WASAPI (Idle)";

    public event Action<StereoAudioBlock>? AudioBlockCaptured;
    public event Action<string>? EngineStateChanged;

    public static IReadOnlyList<AudioDeviceInfo> EnumerateOutputDevices() => EnumerateEndpoints(input: false);
    public static IReadOnlyList<AudioDeviceInfo> EnumerateInputDevices() => EnumerateEndpoints(input: true);

    private static IReadOnlyList<AudioDeviceInfo> EnumerateEndpoints(bool input)
    {
        var list = new List<AudioDeviceInfo>();
        try
        {
            using var enumerator = new MMDeviceEnumerator();
            // 输入列表包含真实录音端点和 Windows 播放混音的数字回环入口。保留设备 ID 用于重开，并把同组的默认端点放在前面。
            if (input) AddEndpoints(DataFlow.Capture, loopback: false);
            AddEndpoints(DataFlow.Render, loopback: input);

            void AddEndpoints(DataFlow flow, bool loopback)
            {
                int firstInGroup = list.Count;
                string? defaultId = null;
                try
                {
                    using var preferred = enumerator.GetDefaultAudioEndpoint(flow, Role.Multimedia);
                    defaultId = preferred.ID;
                }
                catch (COMException) { /* A default endpoint is optional. */ }
                foreach (var endpoint in enumerator.EnumerateAudioEndPoints(flow, DeviceState.Active))
                {
                    using var device = endpoint;
                    try
                    {
                        string id = device.ID;
                        string name = device.FriendlyName;
                        int channels;
                        try
                        {
                            using var client = device.AudioClient;
                            // 共享端点的混音格式给出可映射的声道数。端点快速插拔时访问可能失败，因此列表仍可展示可读取的设备。
                            channels = client.MixFormat.Channels;
                        }
                        catch (COMException) { channels = 0; }
                        var info = new AudioDeviceInfo(loopback ? LoopbackPrefix + id : id,
                            loopback ? $"Windows playback loopback · {name}" : name, AudioBackendMode.Wasapi, channels);
                        if (string.Equals(id, defaultId, StringComparison.OrdinalIgnoreCase)) list.Insert(firstInGroup, info);
                        else list.Add(info);
                    }
                    catch (COMException) { /* The device can disappear during enumeration. */ }
                }
            }
        }
        catch (COMException) { /* Windows Audio may be unavailable. */ }
        return list;
    }

    private int _inLeft,_inRight,_outLeft,_outRight;
    private SignalPathEqualizer _outputEq = new();
    private SignalPathConfig? _outputPath;
    public void Start(SignalPathConfig pathConfig, GeneratorConfig initialGenerator)
    {
        Stop();
        // 重开流时生成器相位和输出均衡的历史清空，新的配置副本供播放线程逐块读取。
        _outputEq=new();
        _outputPath=pathConfig;
        lock (_configLock) _generatorConfig = initialGenerator.Clone();
        _generator.Reset();

        try { OpenStreams(pathConfig); }
        catch { Stop(); throw; }
    }

    private void OpenStreams(SignalPathConfig pathConfig)
    {
        using var enumerator = new MMDeviceEnumerator();
        if (capture)
        {
            var inputId = pathConfig.WasapiInputDeviceId;
            // 回环标识的后半段仍然是 Windows 渲染设备 ID，配上回环捕获类才能接收该端点的混音数据。
            bool loopback = inputId?.StartsWith(LoopbackPrefix,StringComparison.Ordinal) == true;
            _inputDevice = string.IsNullOrWhiteSpace(inputId)
                ? enumerator.GetDefaultAudioEndpoint(DataFlow.Capture, Role.Multimedia)
                : enumerator.GetDevice(loopback ? inputId[LoopbackPrefix.Length..] : inputId);
            _wasapiCapture = loopback ? new WasapiLoopbackCapture(_inputDevice)
                : new WasapiCapture(_inputDevice, useEventSync: true, audioBufferMillisecondsLength: 20);
            // WasapiCapture.WaveFormat's getter strips the extensible header (valid bits and mask).
            // 读取 IAudioClient 的原始混音格式，保留有效位数与扬声器掩码。容器同为 32 位时，24 位有效与 32 位有效需要不同解码。
            using var client = _inputDevice.AudioClient;
            var format = client.MixFormat;
            _wasapiCapture.WaveFormat = format;
            _captureSamples = new EndpointSamples(format);
            ActualSampleRate = format.SampleRate;
            InputBitDepth = _captureSamples.ValidBits;
            int channels = format.Channels;
            // 通道映射从零起算。单声道端点把默认第二路也映射到第零路，其余越界设置直接报错。
            _inLeft = pathConfig.AudioInputCh1;
            _inRight = channels == 1 && pathConfig.AudioInputCh2 == 1 ? 0 : pathConfig.AudioInputCh2;
            if (_inLeft < 0 || _inRight < 0 || _inLeft >= channels || _inRight >= channels)
                throw new InvalidOperationException($"Windows input endpoint has {channels} channels. Check the input channel map.");
            _wasapiCapture.DataAvailable += OnCaptureDataAvailable;
            _wasapiCapture.RecordingStopped += OnEndpointStopped;
        }
        if (playback)
        {
            _outputDevice = string.IsNullOrWhiteSpace(pathConfig.WasapiOutputDeviceId)
                ? enumerator.GetDefaultAudioEndpoint(DataFlow.Render, Role.Multimedia)
                : enumerator.GetDevice(pathConfig.WasapiOutputDeviceId);
            bool exclusive = pathConfig.WasapiExclusivePlayback;
            // 播放回环依赖共享混音路径，同一个端点采用独占播放时无法沿这条路径捕获其输出。
            if (exclusive && _wasapiCapture is WasapiLoopbackCapture &&
                string.Equals(_inputDevice?.ID, _outputDevice.ID, StringComparison.OrdinalIgnoreCase))
                throw new InvalidOperationException("Windows playback loopback requires shared playback on this endpoint. Select a recording input or switch playback to Shared.");
            var mode = exclusive ? AudioClientShareMode.Exclusive : AudioClientShareMode.Shared;
            _wasapiOut = new WasapiOut(_outputDevice, mode, useEventSync: true, latency: 30);
            var format = _wasapiOut.OutputWaveFormat;
            // 共享播放沿用混音格式。独占播放直接探测端点可用格式，按实际选中的格式确定发生器的采样率。
            if (exclusive)
            {
                using var client = _outputDevice.AudioClient;
                format = SelectExclusiveFormat(format, pathConfig.SampleRate, f => client.IsFormatSupported(mode, f));
            }
            OutputSampleRate = format.SampleRate;
            int channels = format.Channels;
            _outLeft = pathConfig.AudioOutputCh1;
            _outRight = channels == 1 && pathConfig.AudioOutputCh2 == 1 ? 0 : pathConfig.AudioOutputCh2;
            if (_outLeft < 0 || _outRight < 0 || _outLeft >= channels || _outRight >= channels)
                throw new InvalidOperationException($"Windows output endpoint has {channels} channels. Check the output channel map.");
            var provider = new GeneratorWaveProvider(this, format, _outLeft, _outRight);
            try { _wasapiOut.Init(provider); }
            catch (Exception ex) when (exclusive)
            {
                throw new InvalidOperationException($"Cannot open {_outputDevice.FriendlyName} for exclusive playback at {OutputSampleRate} Hz (0x{ex.HResult:X8}). {ex.Message}", ex);
            }
            // WasapiOut substitutes a different format and resampler if a driver changes capability between probes.
            // 如果设备能力在查询和打开之间变化，NAudio 可能隐式换格式并重采样。独占测量遇到此情况要求重新打开。
            if (exclusive && !ReferenceEquals(_wasapiOut.OutputWaveFormat, format))
                throw new InvalidOperationException("The WASAPI endpoint changed its supported format while opening exclusive playback. Reopen the signal path.");
            _wasapiOut.PlaybackStopped += OnEndpointStopped;
        }
        // 状态分别报告输入和输出采样率与输出模式，端点音量是打开时的记录，用于核对本次设备设置。
        var endpoints = new List<string>();
        if (capture)
        {
            endpoints.Add(_wasapiCapture is WasapiLoopbackCapture
                ? $"Windows playback loopback {_inputDevice!.FriendlyName} @ {ActualSampleRate} Hz"
                : $"In {_inputDevice!.FriendlyName} @ {ActualSampleRate} Hz {MasterVolumeAtOpen(_inputDevice)}");
        }
        if (playback)
        {
            string mode = pathConfig.WasapiExclusivePlayback ? "Exclusive" : "Shared";
            var samples = new EndpointSamples(_wasapiOut!.OutputWaveFormat);
            endpoints.Add($"Out {_outputDevice!.FriendlyName} @ {OutputSampleRate} Hz ({mode}, {samples.Description}) {MasterVolumeAtOpen(_outputDevice)}");
        }
        StatusDescription = "WASAPI | " + string.Join(" / ", endpoints);
        // 先公布完整初始化后的状态再启动回调。若录音启动过程中发生停止事件，随后不再启动播放。
        Volatile.Write(ref _running, 1);
        _wasapiCapture?.StartRecording();
        if (IsRunning) _wasapiOut?.Play();
    }

    internal static WaveFormat SelectExclusiveFormat(WaveFormat mix, int sampleRate, Func<WaveFormat, bool> supports)
    {
        if (sampleRate <= 0) throw new ArgumentOutOfRangeException(nameof(sampleRate));
        int channels = mix.Channels;
        // 独占播放沿用现有扬声器位置映射。普通单声道用前中置位 4，普通立体声用左右位的并集 3。
        int mask = channels switch { 1 => 4, 2 => 3, _ => 0 };
        if (mix is WaveFormatExtensible)
        {
            var header = WaveFormat.MarshalToPtr(mix);
            try { mask = Marshal.ReadInt32(header, 20); }
            finally { Marshal.FreeHGlobal(header); }
        }
        // Older mono/stereo drivers sometimes accept WAVEFORMATEX but reject its extensible equivalent.
        bool standardLayout = channels == 1 && mask is 0 or 4 || channels == 2 && mask is 0 or 3;
        // 候选保持所需采样率、声道数和掩码，只改变容器宽度、有效精度和编码。设备接受第一个候选时停止探测。
        (int Bits, int Valid, bool Float)[] candidates =
        [
            (32, 32, false), (32, 24, false), (24, 24, false), (32, 32, true), (64, 64, true),
            (32, 20, false), (24, 20, false), (32, 16, false), (16, 16, false), (8, 8, false)
        ];
        foreach (var (bits, valid, floating) in candidates)
        {
            var pointer = WaveFormat.MarshalToPtr(new WaveFormatExtensible(sampleRate, bits, channels));
            WaveFormat candidate;
            try
            {
                // 扩展头中偏移 18 放有效位数，20 放声道掩码，24 开始放子格式 GUID。声卡据子格式区分整数码和浮点位图。
                Marshal.WriteInt16(pointer, 18, (short)valid);
                Marshal.WriteInt32(pointer, 20, mask);
                Guid subtype = floating ? AudioMediaSubtypes.MEDIASUBTYPE_IEEE_FLOAT : AudioMediaSubtypes.MEDIASUBTYPE_PCM;
                Marshal.Copy(subtype.ToByteArray(), 0, pointer + 24, 16);
                candidate = WaveFormat.MarshalFromPtr(pointer);
            }
            finally { Marshal.FreeHGlobal(pointer); }
            if (supports(candidate)) return candidate;
            // 基本格式头无法表达独立有效位数和自定义扬声器映射，只有常见布局和满容器位宽才尝试它。
            if (standardLayout && valid == bits && (!floating || bits == 32))
            {
                var basic = floating ? WaveFormat.CreateIeeeFloatWaveFormat(sampleRate, channels)
                    : new WaveFormat(sampleRate, bits, channels);
                if (supports(basic)) return basic;
            }
        }
        throw new InvalidOperationException($"Exclusive playback has no supported PCM or float format at {sampleRate} Hz with {channels} channels. Select a supported sample rate or use Shared playback.");
    }

    private static string MasterVolumeAtOpen(MMDevice device)
    {
        try
        {
            // 这里只读 Windows 音量控制值与静音状态。它们记录端点设置，实际满量程电压仍由设备输出级和量程决定。
            var volume = device.AudioEndpointVolume;
            return $"(Windows master at open {volume.MasterVolumeLevel:0.##} dB, {(volume.Mute ? "muted" : "unmuted")})";
        }
        catch (Exception)
        {
            // Endpoint volume control is optional and independent of the audio stream.
            return "(Windows master at open unavailable)";
        }
    }

    private void OnEndpointStopped(object? sender, StoppedEventArgs args)
    {
        // 确认停止事件属于本次流，通过原子标志只处理一次，随后停止另一个方向。两个方向按同一次测量会话管理。
        if (sender != _wasapiCapture && sender != _wasapiOut || Interlocked.Exchange(ref _running, 0) == 0) return;
        if (sender == _wasapiCapture) _wasapiOut?.Stop();
        else _wasapiCapture?.StopRecording();
        StatusDescription = $"WASAPI endpoint stopped: {args.Exception?.Message ?? "stream ended"}";
        EngineStateChanged?.Invoke(StatusDescription);
    }

    private void OnCaptureDataAvailable(object? sender, WaveInEventArgs e)
    {
        // 捕获事件可能在停止过程中到达。仅处理当前对象的非空数据块，按本次打开时保存的格式解码。
        if (!IsRunning || sender != _wasapiCapture || _captureSamples is not {} samples || e.BytesRecorded == 0) return;
        AudioBlockCaptured?.Invoke(samples.DecodeBlock(e.Buffer, e.BytesRecorded, _inLeft, _inRight));
    }

    // Keep the endpoint format, including speaker mask and valid bits, all the way to IAudioClient.
    internal sealed class EndpointSamples
    {
        public WaveFormat Format { get; }
        public int ValidBits { get; }
        public string Description => $"{(floating ? "float" : "PCM")} {ValidBits}/{Format.BitsPerSample} bit";
        private readonly int bytesPerSample;
        private readonly bool floating;
        private readonly int paddingBits;
        private readonly long fullScale;

        public EndpointSamples(WaveFormat format)
        {
            Format = format;
            int bits = format.BitsPerSample;
            var encoding = format.Encoding;
            // 扩展头的外层编码只说明头布局，子格式决定波形样本用二补码整数还是 IEEE 浮点。
            if (format is WaveFormatExtensible ext)
            {
                if (ext.SubFormat == AudioMediaSubtypes.MEDIASUBTYPE_PCM) encoding = WaveFormatEncoding.Pcm;
                if (ext.SubFormat == AudioMediaSubtypes.MEDIASUBTYPE_IEEE_FLOAT) encoding = WaveFormatEncoding.IeeeFloat;
            }
            floating = encoding == WaveFormatEncoding.IeeeFloat;
            if (!(floating && bits is 32 or 64 || encoding == WaveFormatEncoding.Pcm && bits is 8 or 16 or 24 or 32)
                || format.Channels < 1 || format.BlockAlign != format.Channels * (bits / 8))
                throw new NotSupportedException($"Unsupported WASAPI mix format: {format}");
            bytesPerSample = bits / 8;
            ValidBits = ApxRemix.Core.Storage.ProjectFileService.InputPrecision(format);
            // 整数有效码在容器高位，低位为填充位。移掉填充位后，以 2^(有效位−1) 个码作为数字满量程尺度。
            paddingBits = bits - ValidBits;
            fullScale = floating ? 0 : 1L << (ValidBits - 1);
        }

        public StereoAudioBlock DecodeBlock(byte[] buffer, int count, int first, int second)
        {
            // 每帧包含同一时刻的全部声道。以十六位立体声为例，每帧四字节，第 i 帧左、右样本分别从 4i 和 4i+2 开始。
            int frames = count / Format.BlockAlign;
            var left = new double[frames];
            var right = new double[frames];
            for (int i = 0; i < frames; i++)
            {
                int offset = i * Format.BlockAlign;
                left[i] = Read(buffer, offset + first * bytesPerSample);
                // 映射到同一物理声道时复用已解码值，标记此次只有一个独立输入，避免上层把相同波形当成双通道证据。
                right[i] = first == second ? left[i] : Read(buffer, offset + second * bytesPerSample);
            }
            return new StereoAudioBlock(left, right, Format.SampleRate, DateTime.UtcNow,
                ReferenceLeft: null, HasIndependentDualChannels: first != second);
        }

        private double Read(byte[] buffer, int offset)
        {
            var sample = buffer.AsSpan(offset, bytesPerSample);
            // 浮点直接读取数值。整数先按小端恢复符号，8 位 PCM 是有 128 偏置的无符号码，24 位则借最高字节扩展符号。
            if (floating) return bytesPerSample == 4
                ? BinaryPrimitives.ReadSingleLittleEndian(sample) : BinaryPrimitives.ReadDoubleLittleEndian(sample);
            int value = bytesPerSample switch
            {
                1 => sample[0] - 128,
                2 => BinaryPrimitives.ReadInt16LittleEndian(sample),
                3 => ((sbyte)sample[2] << 16) | (sample[1] << 8) | sample[0],
                _ => BinaryPrimitives.ReadInt32LittleEndian(sample)
            };
            // 例如 32 位容器中的 24 位有效码 0xC0000000，算术右移八位得 −4194304，再除以 8388608 得 −0.5。
            // 这个比例是无量纲的。若该输入量程经校准满量程峰值为 2 V，则该采样点对应 −1 V。
            return (value >> paddingBits) / (double)fullScale;
        }

        public void EncodeBlock(Span<byte> buffer, ReadOnlySpan<double> left, ReadOnlySpan<double> right,
            int first, int second)
        {
            // 按格式先填静音，然后只写映射的发生器通道。未选的声道在 8 位 PCM 中保持 128，在其他支持格式中保持零。
            buffer.Fill(bytesPerSample == 1 ? (byte)128 : (byte)0);
            for (int i = 0; i < left.Length; i++)
            {
                int offset = i * Format.BlockAlign;
                Write(buffer[(offset + first * bytesPerSample)..], left[i]);
                if (second != first) Write(buffer[(offset + second * bytesPerSample)..], right[i]);
            }
        }

        private void Write(Span<byte> sample, double value)
        {
            // 在端点编码处按目标浮点宽度写入原值。浮点在这个函数中保持自己的数值范围，整数分支负责满量程裁剪。
            if (floating)
            {
                if (bytesPerSample == 4) BinaryPrimitives.WriteSingleLittleEndian(sample, (float)value);
                else BinaryPrimitives.WriteDoubleLittleEndian(sample, value);
                return;
            }

            // 把无量纲幅度乘满量程码后舍入到最近整数（平局取偶）。二补码正端少一码，因此输出 +1 时要限制到最大正码，
            // 然后左移留出低位填充。例如 24 有效位、32 位容器中的 +0.5 写成 0x40000000。
            long quantized = (long)Math.Round(Math.Clamp(value, -1, 1) * fullScale);
            int word = (int)(Math.Min(quantized, fullScale - 1) << paddingBits);
            switch (bytesPerSample)
            {
                case 1: sample[0] = (byte)(word + 128); break;
                case 2: BinaryPrimitives.WriteInt16LittleEndian(sample, (short)word); break;
                case 3:
                    sample[0] = (byte)word;
                    sample[1] = (byte)(word >> 8);
                    sample[2] = (byte)(word >> 16);
                    break;
                case 4: BinaryPrimitives.WriteInt32LittleEndian(sample, word); break;
            }
        }
    }

    public void Stop()
    {
        // 先清活动标志并解除设备事件，再停止和释放流。如此停机期间的回调不会继续产生采集样本或触发端点停机联动。
        Interlocked.Exchange(ref _running, 0);
        if (_wasapiCapture is not null)
        {
            _wasapiCapture.DataAvailable -= OnCaptureDataAvailable;
            _wasapiCapture.RecordingStopped -= OnEndpointStopped;
            try { _wasapiCapture.StopRecording(); } catch { }
            _wasapiCapture.Dispose();
            _wasapiCapture = null;
        }
        if (_wasapiOut is not null)
        {
            _wasapiOut.PlaybackStopped -= OnEndpointStopped;
            try { _wasapiOut.Stop(); } catch { }
            _wasapiOut.Dispose();
            _wasapiOut = null;
        }
        _outputDevice?.Dispose(); _outputDevice = null;
        _inputDevice?.Dispose(); _inputDevice = null;
        _captureSamples = null;
        StatusDescription = "WASAPI (Stopped)";
        EngineStateChanged?.Invoke(StatusDescription);
    }

    // 配置与一次性激励队列由同一把锁保护。控制线程提交快照，播放线程在一个完整缓冲内用同一设置生成样本。
    public void UpdateGenerator(GeneratorConfig config) { lock (_configLock) _generatorConfig = config.Clone(); }
    public void UpdateVirtualDut(VirtualDutConfig dutConfig) { }
    public void EnqueueOneShotExcitation(
        double[] monoSamples,
        int preSilenceSamples = 0,
        int postSilenceSamples = 0,
        bool enableCh1 = true,
        bool enableCh2 = true,
        int secondChannelDelaySamples = 0,
        double[]? secondChannelSamples = null)
    {
        lock (_configLock)
            _generator.EnqueueOneShot(monoSamples, preSilenceSamples, postSilenceSamples, enableCh1, enableCh2, secondChannelDelaySamples, secondChannelSamples);
    }
    public void ClearOneShotExcitation() { lock (_configLock) _generator.ClearOneShot(); }
    public void Dispose() => Stop();

    internal sealed class GeneratorWaveProvider(WasapiAudioEngine owner, WaveFormat format, int first, int second) : IWaveProvider
    {
        private readonly EndpointSamples samples = new(format);
        private double[] leftBuffer = [];
        private double[] rightBuffer = [];
        public WaveFormat WaveFormat => samples.Format;

        public int Read(byte[] buffer, int offset, int count)
        {
            // 端点以字节请求数据，发生器以帧推进相位。只按可容纳的整帧数计算和返回，临时数组按最大已用长度复用。
            int frames = count / WaveFormat.BlockAlign;
            if (frames > leftBuffer.Length)
            {
                leftBuffer = new double[frames];
                rightBuffer = new double[frames];
            }
            var left = leftBuffer.AsSpan(0, frames);
            var right = rightBuffer.AsSpan(0, frames);
            bool useSecondOnMono;
            lock (owner._configLock)
            {
                var config = owner._generatorConfig;
                // 两个发生器通道映射到同一物理输出且仅第二路启用时，把第二路送到该端点。一次性激励拥有发生器时采用它自己的启用位。
                useSecondOnMono = first == second && (owner._generator.IsOneShotOwned
                    ? !owner._generator.OneShotCh1Enabled && owner._generator.OneShotCh2Enabled
                    : !config.Channel1Enabled && config.Channel2Enabled);
                owner._generator.FillBlock(config, WaveFormat.SampleRate, left, right);
            }
            // 先在浮点样本上应用输出均衡，再按端点容器格式量化和交错。这样每个启用声道只有一次整数编码。
            owner._outputEq.Process(owner._outputPath?.OutputEq, WaveFormat.SampleRate, left, right);
            int bytes = frames * WaveFormat.BlockAlign;
            samples.EncodeBlock(buffer.AsSpan(offset, bytes), useSecondOnMono ? right : left, right, first, second);
            return bytes;
        }
    }
}
