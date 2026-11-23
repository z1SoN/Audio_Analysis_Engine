// 已有录音通过这个引擎分块送入，仪表仍按连续采样前进。文件提供了确定的采样率、通道数和末尾位置，这使同一段输入可以多次回放和比较。
using System.Diagnostics;
using ApxRemix.Core.Models;
using ApxRemix.Core.Storage;

namespace ApxRemix.Core.Audio;

/// <summary>
/// WAV 文件离线流输入分析引擎。
/// <para>
/// 严格保证：
/// 1. 保留真实物理声道数（单声道 WAV 显式标注 <c>HasIndependentDualChannels = false</c>，阻止误作双通道 FRF）；
/// 2. 关闭循环（单次分析）到达 EOF 前，完整交付最后一个不足整块的真实尾段样本（不丢样、不补零计入），随后同步 EOF 状态；
/// 3. EOF 后可随时调用 <see cref="Start"/> 重新打开文件分析。
/// </para>
/// </summary>
public sealed class WavFileAudioEngine : IAudioEngine
{
    private sealed record FileData(double[] Left,double[] Right,int Channels,int? Bits,bool ExactPcm);
    private int _sourceChannels = 2;
    private CancellationTokenSource? _cts;
    private Task? _workerTask;

    public AudioBackendMode BackendMode => AudioBackendMode.WavFile;
    public bool IsRunning { get; private set; }
    public bool HasReachedEof { get; private set; }
    public int ActualSampleRate { get; private set; } = 48000;
    public int? InputBitDepth { get; private set; }
    public int OutputSampleRate => ActualSampleRate;
    public bool HasActiveOneShotExcitation => false;
    public int SourceChannelCount => _sourceChannels;
    public string StatusDescription { get; private set; } = "WAV File Input (Idle)";

    public event Action<StereoAudioBlock>? AudioBlockCaptured;
    public event Action<string>? EngineStateChanged;

    /// <inheritdoc />
    public void Start(SignalPathConfig pathConfig, GeneratorConfig initialGenerator)
    {
        Stop();

        // 先验证文件列表并确定分析采样率。块大小决定每次交付的点数，文件或用户选择的采样率决定相邻两点的时间。
        var plan = InputFilePlan.Open(pathConfig.InputFileList ?? [],pathConfig.InputWavFilePath,pathConfig.FileRateMode,pathConfig.FixedFileRate);
        int firstChan=pathConfig.AudioInputCh1, secondChan=pathConfig.AudioInputCh2;
        var firstFile=Load(plan.Paths[0],firstChan,secondChan);
        InputBitDepth=firstFile.Bits;
        _sourceChannels=firstFile.Channels;
        ActualSampleRate=plan.Rate;
        HasReachedEof = false;
        int blockSize = Math.Clamp(pathConfig.BlockSize, 64, 4096);
        bool loop = pathConfig.LoopInputWav;

        // 每次开启分配新的取消源。后台按块发送数据，调用方可以在两块之间取消。
        _cts = new CancellationTokenSource();
        IsRunning = true;
        string fileName = Path.GetFileName(plan.Paths[0]);
        string chTag = _sourceChannels == 1 ? "Mono" : $"{_sourceChannels}ch";
        StatusDescription = $"WAV File | {fileName} ({chTag} @ {ActualSampleRate / 1000.0:0.#} kHz, {firstFile.Left.Length} smp)";

        var token = _cts.Token;
        _workerTask = Task.Run(() => StreamLoop(plan,firstFile,firstChan,secondChan,blockSize,loop,token), token);
    }

    private static FileData Load(string path,int first,int second)
    {
        // 按选择的文件声道读出两条序列，并保留原文件声道数、有效位宽和 PCM 标记。
        // 数字字表可用此标记判断块里是否保留了原始整数码对应的精度。
        var file=ProjectFileService.ReadStereoWavWithMeta(path,first,second);
        if (file.Left.Length==0) throw new InvalidDataException($"Input file contains no samples: {path}");
        var format=ProjectFileService.WavWordFormat(path);
        return new(file.Left,file.Right,file.SourceChannels,format.Depth,format.ExactPcm);
    }

    private async Task StreamLoop(InputFilePlan plan,FileData file,int first,int second,int blockSize,bool loop,CancellationToken token)
    {
        var paths=plan.Paths;
        var clock=Stopwatch.StartNew();
        long samples=0;
        try
        {
            do
            {
                for(int index=0;index<paths.Length;index++)
                {
                    token.ThrowIfCancellationRequested();
                    // 初始文件在 Start 已读，可直接复用。多文件进入下一文件或循环回列表开头时按路径重新加载。
                    if(index!=0 || samples!=0 && paths.Length>1) file=Load(paths[index],first,second);
                    token.ThrowIfCancellationRequested();
                    _sourceChannels=file.Channels;
                    InputBitDepth=file.Bits;
                    var name=Path.GetFileName(paths[index]);
                    StatusDescription=$"WAV File | {index+1}/{paths.Length} · {name} @ {ActualSampleRate} Hz";
                    for(int cursor=0;cursor<file.Left.Length;)
                    {
                        token.ThrowIfCancellationRequested();
                        // 尾部复制剩余点数。比如块长 256，文件还剩 12 点，就交付 12 点，累计时长只增加 12 / Fs 秒。
                        int count=Math.Min(blockSize,file.Left.Length-cursor);
                        int end=cursor+count;
                        AudioBlockCaptured?.Invoke(new(file.Left[cursor..end],file.Right[cursor..end],plan.Rate,DateTime.UtcNow,
                            ReferenceLeft:null,HasIndependentDualChannels:file.Channels>=2,ExactPcm:file.ExactPcm));
                        cursor=end;
                        samples+=count;
                        // Preserve the existing EOF contract: the final sample is visible
                        // immediately, before the worker sleeps for another buffer interval.
                        if(!loop && index==paths.Length-1 && end==file.Left.Length) break;
                        // 用累计样本时间减去墙钟已用时间，抵消上一块执行开销带来的节奏误差。每块仍携带名义采样率供下游分析。
                        double wait=samples*1000d/plan.Rate-clock.Elapsed.TotalMilliseconds;
                        if(wait>1) await Task.Delay((int)wait,token).ConfigureAwait(false);
                    }
                }
            } while(loop);
            token.ThrowIfCancellationRequested();
            // 非循环列表正常交付最后一块后才发布 EOF。主动取消直接退出，读取异常走错误状态。
            HasReachedEof=true;
            StatusDescription=$"WAV File | {Path.GetFileName(paths[^1])} [EOF · {samples} samples]";
        }
        catch(OperationCanceledException) when(token.IsCancellationRequested) { return; }
        catch(Exception e) { if (token.IsCancellationRequested) return; StatusDescription=$"WAV File | {e.Message}"; }
        IsRunning=false;
        EngineStateChanged?.Invoke(StatusDescription);
    }

    public void Stop()
    {
        if (_cts is not null)
        {
            // 通知后台尽快停止，并给它有限时间退出。保留自然 EOF 描述，方便消费者认出已经完整读完。
            _cts.Cancel();
            try { _workerTask?.Wait(500); } catch { }
            _cts.Dispose();
            _cts = null;
            _workerTask = null;
        }
        IsRunning = false;
        if (!HasReachedEof)
            StatusDescription = "WAV File Input (Stopped)";
    }

    // 该引擎只供应文件样本。组合路由若有发生器输出，由另一个引擎负责驱动。
    public void UpdateGenerator(GeneratorConfig config) { }
    public void UpdateVirtualDut(VirtualDutConfig dutConfig) { }
    public void EnqueueOneShotExcitation(
        double[] monoSamples,
        int preSilenceSamples = 0,
        int postSilenceSamples = 0,
        bool enableCh1 = true,
        bool enableCh2 = true,
        int secondChannelDelaySamples = 0,
        double[]? secondChannelSamples = null) { }
    public void ClearOneShotExcitation() { }
    public void Dispose() => Stop();
}
