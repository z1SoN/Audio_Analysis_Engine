// AudioEngineManager 通过这一组操作启动引擎、更新激励和接收采集事件。硬件驱动、虚拟电路和 WAV 回放都交出 StereoAudioBlock，后面的仪表因此可以沿用同一条路径。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Audio;

/// <summary>音频引擎统一接口（支持连续激励、一次性 Chirp 注入与 EOF 状态通知）。</summary>
public interface IAudioEngine : IDisposable
{
    AudioBackendMode BackendMode { get; }
    bool IsRunning { get; }
    bool HasReachedEof => false;
    // 输入采样率给出了双声道块内相邻两帧的时间尺度。输出采样率单独声明，因为录放端点可能分别运行。
    int ActualSampleRate { get; }
    int? InputBitDepth => null;
    int OutputSampleRate { get; }
    bool HasActiveOneShotExcitation { get; }
    string StatusDescription { get; }

    // 一个块里的 Left[i] 和 Right[i] 对应同一采集时刻。块上标记可用通道的独立性及可选的同步参考。
    event Action<StereoAudioBlock>? AudioBlockCaptured;

    /// <summary>当引擎自然到达文件末尾（WAV EOF）或因异常停止时触发。</summary>
    event Action<string>? EngineStateChanged;

    // 启动时由信号路径决定端点、格式与映射，由发生器设置确定初始激励。虚拟 DUT 配置只由能模拟电路的实现处理。
    void Start(SignalPathConfig pathConfig, GeneratorConfig initialGenerator);
    void Stop();
    void UpdateGenerator(GeneratorConfig config);
    void UpdateVirtualDut(VirtualDutConfig dutConfig);

    /// <summary>
    /// 在当前运行的音频输出管线上排入一段一次性波形（例如 Continuous Sweep / Acoustic Response 的 Log Chirp）。
    /// 支持独立通道路由；播放完毕后自动输出静音直到调用方恢复常规发生器。
    /// </summary>
    // 前后静音长度和第二路延迟按输出样本数给出。例如 48 kHz 下 480 点静音对应 10 ms。
    void EnqueueOneShotExcitation(
        double[] monoSamples,
        int preSilenceSamples = 0,
        int postSilenceSamples = 0,
        bool enableCh1 = true,
        bool enableCh2 = true,
        int secondChannelDelaySamples = 0,
        double[]? secondChannelSamples = null);

    /// <summary>清除尚未播完的一次性激励并立即恢复常规发生器输出。</summary>
    void ClearOneShotExcitation();
}
