using ApxRemix.Core.Models;
using ApxRemix.Core.Dsp;

namespace ApxRemix.Core.Audio;

// 按触发条件取得一段固定长度的连续记录，用于 Bench FFT。显示环形缓冲独立滚动，记录一旦完成就能交给分析器反复读取。
public sealed class BenchFrameCapture
{
    private readonly double[] left, right;
    private readonly int preTriggerSamples;
    private double[]? referenceBuffer;

    public BenchFrameCapture(int count) : this(count, 0) { }

    public BenchFrameCapture(int count, int preTriggerSamples)
    {
        // count 是每通道所需样本数。触发之前的上下文只能占其中一部分，至少留一个位置给边沿及其后的数据。
        if (count <= 0) throw new ArgumentOutOfRangeException(nameof(count));
        if (preTriggerSamples < 0 || preTriggerSamples >= count) throw new ArgumentOutOfRangeException(nameof(preTriggerSamples));
        if (count > 8 * 1248000) throw new InvalidOperationException(
            $"This measurement needs {count:N0} samples per channel. The in-memory acquisition limit is {8 * 1248000:N0}; reduce the averages or FFT length.");
        left = new double[count];
        right = new double[count];
        this.preTriggerSamples = preTriggerSamples;
    }

    // The returned arrays belong to this capture; reuse it only for rejected match candidates.
    // Subscribe for the lifetime of one acquisition. The display ring stays small.
    public async Task<AudioCaptureSnapshot> ReadAsync(AudioEngineManager engine,
        FftTriggerMode mode, double threshold, InputChannelSelection channel, int delaySamples,
        TimeSpan triggerTimeout, CancellationToken token, TaskCompletionSource<AudioCaptureSnapshot>? matchProbe = null,
        GeneratorWaveform generator = GeneratorWaveform.Sine, Action? onArmed = null)
    {
        int count = left.Length;
        token.ThrowIfCancellationRequested();
        // 只有正向过零路径支持保留边沿前的样本。它把正延迟解释为推迟寻找边沿，其他触发先找到边沿再跳过延迟样本。
        if (preTriggerSamples != 0 && mode != FftTriggerMode.PositiveZeroCrossing)
            throw new ArgumentException("Pre-trigger context is supported for positive zero crossing.", nameof(mode));
        int rate = engine.ActualSampleRate;
        int delay = Math.Max(0, delaySamples);
        int beforeEdge = mode == FftTriggerMode.PositiveZeroCrossing ? delay : 0;
        if (beforeEdge > 0)
        {
            triggerTimeout += TimeSpan.FromSeconds((double)beforeEdge / rate);
            delay = 0;
        }
        triggerTimeout += TimeSpan.FromSeconds((double)preTriggerSamples / rate);
        // 循环历史仅存触发之前要保留的最近若干点，并给它额外预留相应的等待时间。例如 48 kHz 的 480 点对应 10 ms。
        var historyLeft = preTriggerSamples > 0 ? new double[preTriggerSamples] : null;
        var historyRight = preTriggerSamples > 0 ? new double[preTriggerSamples] : null;
        int historyCount = 0, historyCursor = 0;
        var gate = new object();
        bool closed = false;
        double[]? reference = null;
        int written = 0;
        // 上一点初值未知，跨块保留真实的最近一点后，边沿即使夹在两个回调之间也能在下一块识别。
        double previous = double.NaN;
        bool started = false, stereo = true;
        var triggered = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var complete = new TaskCompletionSource<AudioCaptureSnapshot>(TaskCreationOptions.RunContinuationsAsynchronously);

        void Receive(StereoAudioBlock block)
        {
            // 关闭和写入使用同一把锁。取消订阅时已开始派发的回调仍可能到达，通过关闭标志和锁保证其不会写已结束的记录。
            lock (gate)
                if (!closed) CopyBlock(block);
        }

        int FindEdgeWithHistory(StereoAudioBlock block, int size)
        {
            var signal = channel == InputChannelSelection.Channel2 ? block.Right : block.Left;
            stereo &= block.HasIndependentDualChannels;
            // Warm up the pre-trigger buffer before the delay starts. The delay
            // postpones edge search while the history continues to follow the input.
            for (int i = 0; i < size; i++)
            {
                if (historyCount == preTriggerSamples)
                {
                    if (beforeEdge > 0) beforeEdge--;
                    else
                    {
                        double current = signal[i];
                        if (previous < 0 && current >= 0)
                        {
                            // 检测到 previous < 0 且 current ≥ 0 后，以历史写指针为最早位置，把绕回的两段顺次复制。
                            // 触发样本由后续 CopyBlock 紧跟在历史之后写入，保持双声道时间对齐。
                            int wrap = preTriggerSamples - historyCursor;
                            historyLeft!.AsSpan(historyCursor).CopyTo(left);
                            historyRight!.AsSpan(historyCursor).CopyTo(right);
                            historyLeft.AsSpan(0, historyCursor).CopyTo(left.AsSpan(wrap));
                            historyRight.AsSpan(0, historyCursor).CopyTo(right.AsSpan(wrap));
                            written = preTriggerSamples;
                            return i;
                        }
                        previous = current;
                    }
                }
                else historyCount++;
                // 寻找过程中，两路都推进同一个历史下标。即使用一路判断过零，也会留下另一通道同一时刻的波形。
                historyLeft![historyCursor] = block.Left[i];
                historyRight![historyCursor] = block.Right[i];
                if (++historyCursor == preTriggerSamples) historyCursor = 0;
            }
            return -1;
        }

        int FindEdge(StereoAudioBlock block, int size)
        {
            if (preTriggerSamples != 0) return FindEdgeWithHistory(block, size);
            // 触发延迟可以跨过多个回调。每到一块从剩余等待中减去已消耗的样本，耗尽后从当前块的首个合格位置开始。
            int offset = 0;
            if (beforeEdge > 0)
            {
                offset = Math.Min(size, beforeEdge);
                beforeEdge -= offset;
                if (offset == size) return -1;
            }
            // 发生器已有同采样时间轴的触发标记时，直接使用第一个标记。支持正弦、方波和文件播放这三种触发源。
            if (mode == FftTriggerMode.Generator && block.GeneratorTriggerOffsets is {} marks)
            {
                if (generator is not (GeneratorWaveform.Sine or GeneratorWaveform.Square or GeneratorWaveform.WavPlayback) || marks.Length == 0) return -1;
                return marks[0];
            }
            // 缺显式标记时，要求同步参考和输入块样本数相同。参考时钟对不上无法把发生器边沿定位到一个输入样本。
            if (mode == FftTriggerMode.Generator && block.ReferenceLeft?.Length != size)
            {
                complete.TrySetException(new InvalidOperationException("This input has no sample-synchronous generator trigger."));
                triggered.TrySetResult();
                return -1;
            }
            // 文件播放从首样本标记找触发点。自由运行立即采用允许范围的第一个点，其他支持模式再比较相邻输入样本。
            if (mode == FftTriggerMode.Generator && generator == GeneratorWaveform.WavPlayback)
                return block.FileStartOffsets is { Length: > 0 } starts ? starts[0] : -1;
            if (mode == FftTriggerMode.FreeRun) return offset;
            if (mode == FftTriggerMode.Generator && generator is not (GeneratorWaveform.Sine or GeneratorWaveform.Square)) return -1;
            var signal = mode == FftTriggerMode.Generator ? block.ReferenceLeft! :
                channel == InputChannelSelection.Channel2 ? block.Right : block.Left;
            for (; offset < size; offset++)
            {
                double current = signal[offset];
                // 边沿由前一采样点、当前采样点和该模式的阈值共同决定。无命中也保留当前值给下一次比较。
                bool hit = AudioMetricsAnalyzer.IsFftTriggerEdge(mode, previous, current, threshold);
                previous = current;
                if (hit) break;
            }
            return offset == size ? -1 : offset;
        }

        void CopyBlock(StereoAudioBlock block)
        {
            if (complete.Task.IsCompleted || token.IsCancellationRequested) return;
            int size = Math.Min(block.Left.Length, block.Right.Length);
            if (size == 0) return;
            // 触发设置在接收前已换算为样本数，换采样率后继续会改变窗口的实际时间尺度。需要第二输入边沿时也要求第二通道确实独立。
            if (rate != block.SampleRate || (mode is not (FftTriggerMode.FreeRun or FftTriggerMode.Generator) && channel == InputChannelSelection.Channel2 && !block.HasIndependentDualChannels))
            {
                complete.TrySetException(new InvalidOperationException("The sample rate changed or the selected trigger channel is unavailable."));
                triggered.TrySetResult();
                return;
            }
            int offset = 0;
            if (!started)
            {
                offset = FindEdge(block, size);
                if (offset < 0) return;
                started = true;
                triggered.TrySetResult();
            }
            // 触发后的跳过量逐块扣减，等到可写样本再按目标长度裁剪，双通道使用相同的偏移与数量。
            int skip = Math.Min(delay, size - offset);
            offset += skip;
            delay -= skip;
            int take = Math.Min(count - written, size - offset);
            if (take == 0) return;
            // 首段有同步参考才尝试累积，任一后续块缺参考则整段参考失效，避免返回一条由真实参考和默认零拼成的波形。
            if (written == 0 && block.ReferenceLeft?.Length >= size) reference = referenceBuffer ??= new double[count];
            if (block.ReferenceLeft?.Length < size || block.ReferenceLeft == null) reference = null;
            else if (reference != null) block.ReferenceLeft.AsSpan(offset, take).CopyTo(reference.AsSpan(written, take));
            block.Left.AsSpan(offset, take).CopyTo(left.AsSpan(written));
            block.Right.AsSpan(offset, take).CopyTo(right.AsSpan(written));
            stereo &= block.HasIndependentDualChannels;
            written += take;
            // 匹配探针可提早取得最多 8192 点。整段结束时刻的会话坐标由当前块起点加已消费偏移得到，探针终点按同一坐标回推。
            int probeLength = Math.Min(8192, count);
            if (matchProbe != null && !matchProbe.Task.IsCompleted && written >= probeLength)
            {
                long end = engine.TotalSamplesCaptured - size + offset + take;
                matchProbe.TrySetResult(new(left[..probeLength], right[..probeLength], reference?[..probeLength],
                    end - written + probeLength, rate, true, stereo));
            }
            if (written == count)
                complete.TrySetResult(new(left, right, reference, engine.TotalSamplesCaptured - size + offset + take, rate, true, stereo));
        }

        // 等待触发和等待采满有各自的时限。间隔检查取消与设备运行态，输入停流时无需等到完整超时。
        async Task WaitFor(Task phase, TimeSpan limit, string timeoutMessage)
        {
            var timer = System.Diagnostics.Stopwatch.StartNew();
            while (!phase.IsCompleted)
            {
                token.ThrowIfCancellationRequested();
                if (!engine.IsRunning) throw new InvalidOperationException("Audio input stopped before the record was complete.");
                TimeSpan remaining = limit - timer.Elapsed;
                if (remaining <= TimeSpan.Zero) throw new TimeoutException(timeoutMessage);
                var check = TimeSpan.FromMilliseconds(150);
                await Task.WhenAny(phase, Task.Delay(remaining < check ? remaining : check, token)).ConfigureAwait(false);
            }
            await phase.ConfigureAwait(false);
        }

        engine.AudioBlockCaptured += Receive;
        try
        {
            // 采集订阅先就绪，再通知可以启动激励。先等待触发，然后按目标时长估计收齐连续样本所需的超时。
            onArmed?.Invoke();
            await WaitFor(triggered.Task, triggerTimeout, "Trigger timed out waiting for an input edge.").ConfigureAwait(false);
            double duration = (count + (double)delaySamples) / rate;
            await WaitFor(complete.Task, TimeSpan.FromSeconds(2 * duration + 5), "Input stopped delivering samples.").ConfigureAwait(false);
            return await complete.Task.ConfigureAwait(false);
        }
        finally
        {
            engine.AudioBlockCaptured -= Receive;
            lock (gate) closed = true;
        }
    }
}
