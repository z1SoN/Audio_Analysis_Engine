using ApxRemix.Core.Models;
using NAudio.CoreAudioApi;
using NAudio.Wave;

namespace ApxRemix.Core.Audio;

public sealed class AudibleMonitorPlayback : IDisposable
{
    // 可听监听驱动独立的 Windows 播放端点。它通过监听流取样，端点停止时把原因转发给调用方。
    private readonly MMDevice _output;
    private readonly WasapiOut _player;
    public AudibleMonitorStream Stream { get; }
    public string OutputName => _output.FriendlyName;
    public event Action<string>? Interrupted;

    public static void ValidateRoute(AudioBackendMode backend, string? input, string? measurementOutput, string monitorOutput,
        AudioOutputConnector? outputConnector = null)
    {
        // 防止监听的声音重新进入测量链。同一个数字回环端点会收回自己的监听声，而与测量共用播放端点可能混入激励。
        if (backend == AudioBackendMode.WavFile) throw new InvalidOperationException("File input is unavailable for audible monitoring.");
        if (backend == AudioBackendMode.Wasapi && string.Equals(input,"loopback:"+monitorOutput,StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException("Select a monitor output different from the loopback input.");
        if ((outputConnector ?? SignalPathConfig.DefaultOutput(backend)) == AudioOutputConnector.Wasapi &&
            string.Equals(measurementOutput,monitorOutput,StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException("Select a separate monitor output so playback does not enter the measurement output.");
    }

    public AudibleMonitorPlayback(string? endpointId, SignalPathConfig path, int rate, AudibleMix mix)
    {
        if (path.BackendMode == AudioBackendMode.WavFile) throw new InvalidOperationException("File input is unavailable for audible monitoring.");
        // 没指定监听端点时使用 Windows 默认播放器。验证前也要解析默认测量出口，才能用实际设备 ID 判断重合。
        using var devices = new MMDeviceEnumerator();
        _output = endpointId == null ? devices.GetDefaultAudioEndpoint(DataFlow.Render, Role.Multimedia) : devices.GetDevice(endpointId);
        try
        {
            string? measurementOut = path.WasapiOutputDeviceId;
            if (path.EffectiveOutput == AudioOutputConnector.Wasapi && string.IsNullOrWhiteSpace(measurementOut))
            {
                using var defaultOut = devices.GetDefaultAudioEndpoint(DataFlow.Render,Role.Multimedia);
                measurementOut = defaultOut.ID;
            }
            ValidateRoute(path.BackendMode,path.WasapiInputDeviceId,measurementOut,_output.ID,path.EffectiveOutput);
            // 以采集的采样率构造监听提供者，由共享播放器负责向监听端点送数据。
            Stream = new AudibleMonitorStream(rate) { Mix=mix };
            _player = new WasapiOut(_output,AudioClientShareMode.Shared,true,40);
            try
            {
                _player.Init(Stream);
                _player.PlaybackStopped += OnStopped;
            }
            catch { _player.Dispose(); throw; }
        }
        catch { _output.Dispose(); throw; }
    }
    public void Start() => _player.Play();
    private void OnStopped(object? sender, StoppedEventArgs e) => Interrupted?.Invoke(e.Exception?.Message ?? "Playback ended.");
    public void Dispose()
    {
        // 手动释放前先断开停止事件，避免正常关闭再次被调用方当作监听中断。
        _player.PlaybackStopped -= OnStopped;
        try { _player.Dispose(); } finally { _output.Dispose(); }
    }
}
