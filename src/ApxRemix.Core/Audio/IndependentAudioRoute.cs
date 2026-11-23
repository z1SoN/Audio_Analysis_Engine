using ApxRemix.Core.Models;

namespace ApxRemix.Core.Audio;

/// <summary>One acquisition clock and an optional, independently clocked playback endpoint.</summary>
public sealed class IndependentAudioRoute(IAudioEngine input, IAudioEngine? output) : IAudioEngine
{
    // 采样率与位宽来自输入引擎，输出速率单独读取。就算两个设备都显示 48 kHz，独立时钟仍可能逐渐错开。
    public AudioBackendMode BackendMode => input.BackendMode;
    public bool IsRunning => input.IsRunning && (output?.IsRunning ?? true);
    public bool HasReachedEof => input.HasReachedEof;
    public int ActualSampleRate => input.ActualSampleRate;
    public int? InputBitDepth => input.InputBitDepth;
    public int OutputSampleRate => output?.OutputSampleRate ?? 0;
    internal string? OpenWasapiOutputDeviceId => (output as WasapiAudioEngine)?.OpenOutputDeviceId;
    internal string? OpenWasapiInputDeviceId => (input as WasapiAudioEngine)?.OpenInputDeviceId;
    public bool HasActiveOneShotExcitation => output?.HasActiveOneShotExcitation ?? false;
    public string StatusDescription => $"{input.StatusDescription} / {output?.StatusDescription ?? "External source (no output)"}";
    public event Action<StereoAudioBlock>? AudioBlockCaptured;
    public event Action<string>? EngineStateChanged;
    private int active;
    private bool subscribed;
    private readonly object playbackLock = new();
    private bool playbackOpened;

    public void Start(SignalPathConfig path, GeneratorConfig generator)
    {
        if (subscribed) Stop();
        // 先以静音启动输出和输入，两端都就绪且已验证运行态才恢复请求的发生器设置，避免采集准备期间发出测试波形。
        var mute = generator.Clone();
        mute.IsEnabled = false;
        input.AudioBlockCaptured += ForwardInput;
        input.EngineStateChanged += InputChanged;
        if (output != null) output.EngineStateChanged += OutputChanged;
        subscribed = true;
        try
        {
            lock (playbackLock)
            {
                playbackOpened = output != null;
                output?.Start(path, mute);
            }
            input.Start(path, mute);
            Interlocked.Exchange(ref active, 1);
            if (!input.IsRunning) InputChanged(input.StatusDescription);
            else if (output?.IsRunning == false) OutputChanged(output.StatusDescription);
            else UpdateGenerator(generator);
        }
        catch
        {
            Stop();
            throw;
        }
    }

    // 独立时钟之间缺少逐样本对齐关系。往上交付真实输入样本时，去除来自内部回环的参考、码型和触发下标。
    private void ForwardInput(StereoAudioBlock block) => AudioBlockCaptured?.Invoke(
        block with { ReferenceLeft = null, FileStartOffsets = null, DigitalPattern = null, GeneratorTriggerOffsets = null });

    private void InputChanged(string status)
    {
        // 输入结束时停止与这次测量配对的输出。原子运行标志让两端同时结束时只进入一次终止流程。
        if (input.IsRunning || Interlocked.Exchange(ref active, 0) == 0) return;
        StopOutput();
        EngineStateChanged?.Invoke(status);
    }

    private void OutputChanged(string status)
    {
        // 实际输出故障也结束配对采集。外部激励模式的空输出不会被视为故障。
        if (output?.IsRunning != false || Interlocked.Exchange(ref active, 0) == 0) return;
        input.Stop();
        EngineStateChanged?.Invoke(status);
    }

    public void Stop()
    {
        Interlocked.Exchange(ref active, 0);
        // 手动停止先撤销转发订阅，再释放输入输出。角色之间的事件不会在此过程发起另一轮停止。
        if (subscribed)
        {
            input.AudioBlockCaptured -= ForwardInput;
            input.EngineStateChanged -= InputChanged;
            if (output != null) output.EngineStateChanged -= OutputChanged;
            subscribed = false;
        }
        try { input.Stop(); }
        finally { StopOutput(); }
    }

    private void StopOutput()
    {
        lock (playbackLock)
        {
            // 输出锁把开流、更新配置和停流串行化。每次打开的输出只执行一次停止。
            if (!playbackOpened) return;
            playbackOpened = false;
            output!.Stop();
        }
    }

    public void UpdateGenerator(GeneratorConfig config)
    {
        lock (playbackLock)
            if (playbackOpened && Volatile.Read(ref active) != 0) output!.UpdateGenerator(config);
    }
    public void UpdateVirtualDut(VirtualDutConfig config) { }
    public void EnqueueOneShotExcitation(double[] monoSamples, int preSilenceSamples = 0, int postSilenceSamples = 0,
        bool enableCh1 = true, bool enableCh2 = true, int secondChannelDelaySamples = 0, double[]? secondChannelSamples = null)
    {
        // 一次性激励只排进当前输出引擎，其静音与延迟沿用输出引擎的采样时钟。
        if (output == null) throw new InvalidOperationException("External source has no generator output.");
        output.EnqueueOneShotExcitation(monoSamples,preSilenceSamples,postSilenceSamples,enableCh1,enableCh2,secondChannelDelaySamples,secondChannelSamples);
    }
    public void ClearOneShotExcitation() => output?.ClearOneShotExcitation();
    public void Dispose()
    {
        Stop();
        try { input.Dispose(); }
        finally { output?.Dispose(); }
    }
}
