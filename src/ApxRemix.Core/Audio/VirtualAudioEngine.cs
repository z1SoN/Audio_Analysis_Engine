// 需要能重复验证的电路时，这个引擎用 SignalGeneratorDsp 产生输入，再经 VirtualDutProcessor 形成模拟输出。它沿设备事件接口交付数据，所以测试也能走正常的采集流程。
using System.Diagnostics;
using ApxRemix.Core.Dsp;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Audio;

/// <summary>内置高精度虚拟环回音频引擎（支持常规发生器与一次性 Chirp 激励注入）。</summary>
public sealed class VirtualAudioEngine : IAudioEngine
{
    private readonly SignalGeneratorDsp _generator = new();
    private readonly VirtualDutProcessor _dutProcessor = new();
    private readonly object _configLock = new();

    private GeneratorConfig _generatorConfig = new();
    private VirtualDutConfig _dutConfig = new();
    private CancellationTokenSource? _cts;
    private Task? _workerTask;

    public AudioBackendMode BackendMode => AudioBackendMode.VirtualSimulator;
    public bool IsRunning { get; private set; }
    public int ActualSampleRate { get; private set; } = 48000;
    public int? InputBitDepth => 64;
    public int OutputSampleRate => ActualSampleRate;
    public bool HasActiveOneShotExcitation => _generator.HasActiveOneShot;
    public string StatusDescription { get; private set; } = "Virtual Loopback Simulator (Idle)";

    public event Action<StereoAudioBlock>? AudioBlockCaptured;
    public event Action<string>? EngineStateChanged;

    private SignalPathEqualizer _outputEq = new();
    private SignalPathConfig? _outputPath;
    public void Start(SignalPathConfig pathConfig, GeneratorConfig initialGenerator)
    {
        Stop();
        _outputEq=new();
        _outputPath=pathConfig;

        // 采样率设置求值的时间间隔，分块只决定向消费者交付数据的粒度。启动时把旧发生器相位和 DUT 的历史状态同时清除。
        ActualSampleRate = pathConfig.SampleRate > 0 ? pathConfig.SampleRate : 48000;
        int blockSize = Math.Clamp(pathConfig.BlockSize, 256, 4096);

        lock (_configLock)
        {
            _generatorConfig = initialGenerator.Clone();
            _dutConfig = CloneDut(pathConfig.VirtualDut);
        }

        _generator.Reset();
        _dutProcessor.Reset();

        _cts = new CancellationTokenSource();
        IsRunning = true;
        StatusDescription = $"Virtual DUT Loopback @ {ActualSampleRate / 1000.0:0.#} kHz (64-bit DSP)";

        var token = _cts.Token;
        _workerTask = Task.Run(() => RunLoop(ActualSampleRate, blockSize, token), token);
    }

    private async Task RunLoop(int sampleRate, int blockSize, CancellationToken token)
    {
        // 理想墙钟节奏为每块 N / Fs 秒。DSP 按这个采样率计算，后台任务尽力把样本以接近实时的速度送出。
        double blockDurationMs = blockSize * 1000.0 / sampleRate;
        var sw = Stopwatch.StartNew();
        long blockIndex = 0;

        while (!token.IsCancellationRequested)
        {
            GeneratorConfig genSnapshot;
            VirtualDutConfig dutSnapshot;
            lock (_configLock)
            {
                // 用户可能恰在某一块生成途中调节参数。先在锁内取配置副本，这一块激励和电路响应就能用一致的设置算完。
                genSnapshot = _generatorConfig.Clone();
                dutSnapshot = CloneDut(_dutConfig);
            }

            // 生成两路数字激励，应用输出均衡后再送入虚拟 DUT。同步参考从这个输入节点分流，DUT 输出则作为采集响应。
            var srcL = new double[blockSize];
            var srcR = new double[blockSize];
            _generator.FillBlock(genSnapshot, sampleRate, srcL, srcR);
        _outputEq.Process(_outputPath?.OutputEq,sampleRate,srcL,srcR);

            var outL = new double[blockSize];
            var outR = new double[blockSize];
            // 模型使用参考峰值描述常规发生器的幅度尺度，取 10^(dBFS/20)。一次性激励直接提供已成形的序列，此处分配单位尺度。
            double refPeak = _generator.IsOneShotOwned
                ? 1.0
                : (genSnapshot.IsEnabled ? SignalGeneratorDsp.DbfsToLinearPeak(genSnapshot.LevelDbfs) : 0.0);

            _dutProcessor.ProcessBlock(dutSnapshot, refPeak, sampleRate, srcL, srcR, outL, outR);
            // 若一次性激励仅使能 Ch2（关闭 Ch1），将 Ch2 激励作为同步参考供单路 Ch2 环回反卷积对齐
            double[] syncRef = (_generator.IsOneShotOwned && !_generator.OneShotCh1Enabled && _generator.OneShotCh2Enabled) ? srcR : srcL;
            var eq = _outputPath?.OutputEq;
            // 仅在输出均衡旁路、使用常规数字波形时带上数字码参考。第二路若有模型延迟，同下标的源样本不再是它的对照值。
            bool bitExact = genSnapshot.IsEnabled && !_generator.IsOneShotOwned && GeneratorWaveSettings.IsDigital(genSnapshot.Waveform)
                && eq?.First.Enabled != true && eq?.Second.Enabled != true;
            var pattern = bitExact ? new DigitalPatternReference(genSnapshot.Channel1Enabled ? srcL : null,
                genSnapshot.Channel2Enabled && dutSnapshot.Channel2DelaySamples == 0 ? srcR : null,
                genSnapshot.WaveSettings.DigitalBits, genSnapshot.Waveform == GeneratorWaveform.ConstantValue) : null;
            AudioBlockCaptured?.Invoke(new StereoAudioBlock(outL, outR, sampleRate, DateTime.UtcNow, ReferenceLeft: syncRef, HasIndependentDualChannels: true, FileStartOffsets: _generator.FileStartOffsets, DigitalPattern: pattern));

            blockIndex++;
            // 将下个块的目标时间与累计墙钟时间比较。轻微落后时追赶，落后超过 200 ms 时重建墙钟基准，已生成的样本不受影响。
            double delayMs = blockIndex * blockDurationMs - sw.Elapsed.TotalMilliseconds;
            if (delayMs > 1.0)
            {
                try { await Task.Delay((int)delayMs, token).ConfigureAwait(false); }
                catch (OperationCanceledException) { break; }
            }
            else if (delayMs < -200.0)
            {
                sw.Restart();
                blockIndex = 0;
            }
        }
    }

    public void Stop()
    {
        if (_cts is not null)
        {
            // 通知模拟线程停发，再释放取消资源并发布停机状态，供使用音频引擎接口的消费者清理状态。
            _cts.Cancel();
            try { _workerTask?.Wait(500); } catch { }
            _cts.Dispose();
            _cts = null;
            _workerTask = null;
        }
        IsRunning = false;
        StatusDescription = "Virtual Loopback Simulator (Stopped)";
        EngineStateChanged?.Invoke(StatusDescription);
    }

    // 控制端更新发生器和 DUT 时交付配置副本，处理线程在下个块读取。一次性激励和常规波形共用发生器的输出时间轴。
    public void UpdateGenerator(GeneratorConfig config) { lock (_configLock) _generatorConfig = config.Clone(); }
    public void UpdateVirtualDut(VirtualDutConfig dutConfig) { lock (_configLock) _dutConfig = CloneDut(dutConfig); }
    public void EnqueueOneShotExcitation(
        double[] monoSamples,
        int preSilenceSamples = 0,
        int postSilenceSamples = 0,
        bool enableCh1 = true,
        bool enableCh2 = true,
        int secondChannelDelaySamples = 0,
        double[]? secondChannelSamples = null)
        => _generator.EnqueueOneShot(monoSamples, preSilenceSamples, postSilenceSamples, enableCh1, enableCh2, secondChannelDelaySamples, secondChannelSamples);
    public void ClearOneShotExcitation() => _generator.ClearOneShot();

    // 复制会影响模型输出的增益、延迟、谐波、噪声和带宽配置，以同一组参数处理完整的一块输入。
    private static VirtualDutConfig CloneDut(VirtualDutConfig src) => new()
    {
        GainDb = src.GainDb,
        Channel2GainOffsetDb = src.Channel2GainOffsetDb,
        Channel2DelaySamples = src.Channel2DelaySamples,
        SecondHarmonicPercent = src.SecondHarmonicPercent,
        ThirdHarmonicPercent = src.ThirdHarmonicPercent,
        NoiseFloorDbfs = src.NoiseFloorDbfs,
        HighPassCutoffHz = src.HighPassCutoffHz,
        LowPassCutoffHz = src.LowPassCutoffHz,
        EnableBandwidthFilter = src.EnableBandwidthFilter
    };

    public void Dispose() => Stop();
}
