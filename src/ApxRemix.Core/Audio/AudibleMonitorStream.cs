using System.Runtime.InteropServices;
using ApxRemix.Core.Dsp;
using ApxRemix.Core.Models;
using NAudio.Wave;

namespace ApxRemix.Core.Audio;

// 监听可输出采集波形或陷掉指定基波后的残余。混合方向与增益只作用于监听支路。
public enum AudibleSource { InputSignal, ThdNResidual }
public enum AudibleGain { Automatic = 0, X1 = 1, X3 = 3, X10 = 10, X30 = 30, X100 = 100, X300 = 300, X1K = 1000, X3K = 3000, X10K = 10000, X30K = 30000, X100K = 100000 }

public sealed record AudibleMix(AudibleSource Source = AudibleSource.InputSignal, AudibleGain Gain = AudibleGain.Automatic,
    bool Ch1ToLeft = true, bool Ch2ToLeft = false, bool Ch1ToRight = false, bool Ch2ToRight = true, double NotchHz = 1000);

/// <summary>Bounded audio monitor tap. Capture writes raw frames while the playback clock drains and mixes them.</summary>
public sealed class AudibleMonitorStream : IWaveProvider
{
    private readonly double[] _left, _right;
    private double[] _workLeft = [], _workRight = [];
    private DigitalResidualMeter _notchL, _notchR;
    private readonly object _gate = new();
    private int _oldest, _available;
    private long _dropped;
    private bool _resetNotch = true;
    private double _automaticGain = 1;
    private AudibleMix _mix = new();
    public WaveFormat WaveFormat { get; }
    public long DroppedFrames => Interlocked.Read(ref _dropped);
    public AudibleMix Mix { get => Volatile.Read(ref _mix); set => Volatile.Write(ref _mix, value); }

    public AudibleMonitorStream(int sampleRate, int bufferMilliseconds = 160)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(sampleRate, 8000);
        ArgumentOutOfRangeException.ThrowIfLessThan(bufferMilliseconds,1);
        WaveFormat = WaveFormat.CreateIeeeFloatWaveFormat(sampleRate, 2);
        // 缓冲以时间预算换算成帧数。例如 48 kHz 和 160 ms 得 7680 帧。两路数组共同描述这段排队时间。
        int capacity = checked((int)Math.Ceiling(sampleRate * (bufferMilliseconds / 1000.0)));
        _left = new double[capacity];
        _right = new double[capacity];
        _notchL = new(sampleRate);
        _notchR = new(sampleRate);
    }

    public void Write(StereoAudioBlock block)
    {
        // 此处无重采样器，采样率相同时按帧加入监听队列。单声道的右路留静音，以便有意识地映射到所需扬声器。
        if (block.SampleRate != WaveFormat.SampleRate) return;
        int frames = block.HasIndependentDualChannels ? Math.Min(block.Left.Length,block.Right.Length) : block.Left.Length;
        int accepted = Math.Min(frames, _left.Length);
        lock (_gate)
        {
            // 播放端若暂时取样慢，丢掉最旧的积压以限制监听延迟。写入的新块超长时也只保留其末尾，因时间不连续需要重启陷波。
            int drop = Math.Max(0, _available + accepted - _left.Length);
            if (drop > 0 || frames > accepted)
            {
                _oldest = (_oldest + drop) % _left.Length;
                _available -= drop;
                Interlocked.Add(ref _dropped,drop + frames - accepted);
                _resetNotch = true;
            }
            // 从已排队数据的末尾写入。要写的样本跨数组尾部时分成两段，保持时间先后。
            int write = (_oldest + _available) % _left.Length;
            int first = Math.Min(accepted, _left.Length - write);
            int start = frames - accepted;
            void Copy(double[] source, double[] target)
            {
                Array.Copy(source,start,target,write,first);
                Array.Copy(source,start+first,target,0,accepted-first);
            }
            Copy(block.Left,_left);
            if (block.HasIndependentDualChannels) Copy(block.Right,_right);
            else { Array.Clear(_right,write,first); Array.Clear(_right,0,accepted-first); }
            _available += accepted;
        }
    }

    public int Read(byte[] buffer, int offset, int count)
    {
        var bytes = buffer.AsSpan(offset,count);
        // 先为端点准备静音。请求长度按整帧解读，队列若欠载，其余帧就保留这段静音。
        bytes.Clear();
        int requested = count / WaveFormat.BlockAlign;
        if (requested == 0) return count;
        if (_workLeft.Length < requested) { _workLeft = new double[requested]; _workRight = new double[requested]; }
        int available;
        bool reset;
        lock (_gate)
        {
            // 在队列锁内只完成复制和读指针更新。随后滤波与混音在临时数组上进行，让采集能尽快继续写入。
            available = Math.Min(requested,_available);
            int first = Math.Min(available,_left.Length-_oldest);
            Array.Copy(_left,_oldest,_workLeft,0,first); Array.Copy(_left,0,_workLeft,first,available-first);
            Array.Copy(_right,_oldest,_workRight,0,first); Array.Copy(_right,0,_workRight,first,available-first);
            _available -= available;
            _oldest = (_oldest + available) % _left.Length;
            reset = _resetNotch;
            _resetNotch = available < requested;
        }
        if (available == 0) return count;
        var mix = Mix;
        // 切到失真残余时，对两路按指定频率陷波，检验中心落在有效频率内。加入间断或切换来源后清掉滤波历史。
        if (mix.Source == AudibleSource.ThdNResidual)
        {
            if (!double.IsFinite(mix.NotchHz) || mix.NotchHz < 5 || mix.NotchHz >= WaveFormat.SampleRate/2) return count;
            if (reset || _previousSource != mix.Source)
            {
                _notchL = new(WaveFormat.SampleRate); _notchR = new(WaveFormat.SampleRate);
            }
            // 陷波器就地修改当前样本窗口。末参数 true 让残余再经过交流耦合，监听随后对抑制了基波和直流的波形做音量与路由处理。
            _notchL.Measure(_workLeft.AsSpan(0,available),mix.NotchHz,AnalyzerWeighting.Flat,true);
            _notchR.Measure(_workRight.AsSpan(0,available),mix.NotchHz,AnalyzerWeighting.Flat,true);
        }
        _previousSource = mix.Source;
        // 监听端使用每声道 32 位 IEEE 浮点交错格式。先对每只扬声器选择或叠加两个输入，再依据混合后的最大绝对值调节增益。
        var output = MemoryMarshal.Cast<byte,float>(bytes[..(requested*WaveFormat.BlockAlign)]);
        double peak = 0;
        for (int i=0; i<available; i++)
        {
            double l = double.IsFinite(_workLeft[i]) ? _workLeft[i] : 0;
            double r = double.IsFinite(_workRight[i]) ? _workRight[i] : 0;
            _workLeft[i] = (mix.Ch1ToLeft ? l : 0) + (mix.Ch2ToLeft ? r : 0);
            _workRight[i] = (mix.Ch1ToRight ? l : 0) + (mix.Ch2ToRight ? r : 0);
            peak = Math.Max(peak,Math.Max(Math.Abs(_workLeft[i]),Math.Abs(_workRight[i])));
        }
        double gain = (int)mix.Gain;
        if (mix.Gain == AudibleGain.Automatic)
        {
            // Fast attack prevents a loud block following a quiet one from overloading the output.
            // 自动增益把块峰值瞄准 0.25 满量程，并限制极弱输入的放大倍数。必要时立即降增益，升增益则按 0.4 秒的指数时间常数逐渐恢复。
            double target = peak > 1e-7 ? Math.Min(100000,0.25/peak) : 1;
            double release = 1-Math.Exp(-available/(0.4*WaveFormat.SampleRate));
            _automaticGain = target < _automaticGain ? target : _automaticGain + release*(target-_automaticGain);
            gain = _automaticGain;
        }
        for (int i=0; i<available; i++)
        {
            // 顺序写 L0、R0、L1、R1，并在输出前把样本夹到数字满量程。此限制只影响实际听到的波形。
            output[2*i] = (float)Math.Clamp(_workLeft[i]*gain,-1,1);
            output[2*i+1] = (float)Math.Clamp(_workRight[i]*gain,-1,1);
        }
        return count;
    }
    private AudibleSource _previousSource;
}
