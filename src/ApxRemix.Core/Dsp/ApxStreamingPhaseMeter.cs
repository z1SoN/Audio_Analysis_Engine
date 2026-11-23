// 相位表有自己的报告节奏，驱动回调的长度却由设备决定。这个外层凑齐报告区间再调用 ApxPhaseMeter，便于给实时采集与逐帧对照使用。
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

/// <summary>Reblocks arbitrary device callbacks into continuous 20 readings/s AP acquisitions.</summary>
public sealed class ApxStreamingPhaseMeter
{
    private readonly ApxPhaseMeter _meter;
    private readonly double[] _left, _right;
    private int _pending;
    private long _processed;
    public int SampleRate { get; }
    public ApxRealtimeReading? Latest { get; private set; }

    // 以约每秒 20 次的节拍建表，缓冲长度按采样率整数除以 20 取样本数。
    public ApxStreamingPhaseMeter(int sampleRate)
    {
        SampleRate = sampleRate;
        _meter = new(sampleRate, 20);
        _left = new double[sampleRate / 20]; _right = new double[_left.Length];
    }
    // 音频回调可以任意分块。左右同步地填满一个报告缓冲，未填满的尾部留给下一次调用。
    public void Process(ReadOnlySpan<double> left, ReadOnlySpan<double> right)
    {
        int count = Math.Min(left.Length, right.Length);
        int offset = 0;
        while (offset < count)
        {
            int take = Math.Min(_left.Length - _pending, count - offset);
            left.Slice(offset, take).CopyTo(_left.AsSpan(_pending));
            right.Slice(offset, take).CopyTo(_right.AsSpan(_pending));
            _pending += take; offset += take; _processed += take;
            // 每满一窗才读取并更新 Latest。_processed 标出该读数结束在整个流中的样本位置。
            if (_pending == _left.Length)
            {
                _meter.Process(_left, _right);
                Latest = new(_meter.Read(), _processed, SampleRate, 20);
                _pending = 0;
            }
        }
    }
}
