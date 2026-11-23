# Audio Analysis Engine

APx Remix 的独立音频测量内核，基于 C# 和 .NET 8。包含 Bench 流式仪表、Sequence 记录分析，以及 FFT、THD+N、IMD、扫频和传递函数等 DSP。可直接处理 PCM，也可使用 WAV、WASAPI、通用 ASIO 或虚拟回路。

## 构建

在 Windows 上安装 .NET 8 SDK，然后运行：

```powershell
dotnet build src/ApxRemix.Core/ApxRemix.Core.csproj -c Release
```

采集和 WAV 读写依赖 NuGet 上的 NAudio。数值辅助进程由 Windows 自带的 .NET Framework 编译器构建。

## PCM 入口

- **Bench** 使用 `StreamingBenchPipeline`，传入采样率，再用 `Process(left, right)` 顺序提交同一时间段的左右声道样本。
- **Sequence** 使用 `SequenceToneMeasurement.Analyze(pcm, sampleRate)` 分析一帧单声道 PCM，返回频率（Hz）、相位（弧度）、线性电平及 THD/THD+N。输入满幅正弦的峰值为 1。
- **扫频和传递函数** 见 `ContinuousSweepAnalyzer`、`FrequencyResponseSweeper` 及 `TransferFunctionAnalyzer`。

## 运行依赖

基础计算可直接运行。Sequence 可通过 `APXREMIX_MKL_X86` 和 `APXREMIX_MKL_X64` 显式配置对应架构的独立 Intel MKL 目录。未设置时使用本地计算路径。两条路径在个别 FFT 和陷波边界上的读数可能不同。
