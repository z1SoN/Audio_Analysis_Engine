namespace ApxRemix.Core.Dsp;

/// <param name="Left">Left-channel digital RMS carried by the completion packet.</param>
/// <param name="Right">Right-channel digital RMS carried by the completion packet.</param>
/// <param name="CompletedPacketEnd">Exclusive end of the PCM consumed by that packet.</param>
internal readonly record struct SequenceCaptureRmsSnapshot(
    double Left, double Right, int CompletedPacketEnd);

/// <summary>Digital File acquisition RMS as carried by the packet that completes a Sequence record.</summary>
internal sealed class SequenceCaptureRms
{
    // 按每秒 1000 包推进采样时钟，累计 250 包更新一次电平。两声道各保存自己的滤波和检波历史。
    private const int PacketsPerSecond = 1000;
    private const int ReadingsPerSecond = 4;
    private const int PacketsPerReading = PacketsPerSecond / ReadingsPerSecond;

    private readonly int _basePacketSize;
    private readonly double _fractionalPacketSize;
    private readonly double[] _work;
    private readonly ApxAcCouplingFilter _leftAc, _rightAc;
    private readonly ApxRmsDetector _leftRms, _rightRms;
    private double _fractionalSamples;
    private double _leftEnvelope, _rightEnvelope;
    private int _packets;
    private int _lastPacketStart;
    private SequenceCaptureRmsSnapshot _snapshot;

    // 由采样率换算 1 ms 包长。以 44.1 kHz 为例，基本包有 44 点，小数部分积攒成额外的一点。
    internal SequenceCaptureRms(int sampleRate)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(sampleRate, PacketsPerSecond);
        double samplesPerPacket = sampleRate * .001;
        _basePacketSize = (int)samplesPerPacket;
        _fractionalPacketSize = samplesPerPacket - _basePacketSize;
        _work = new double[_basePacketSize + 1];
        // 采集过程中重复使用这四个有状态对象，使交流耦合和检波能跨包延续。
        _leftAc = new(sampleRate);
        _rightAc = new(sampleRate);
        _leftRms = new(sampleRate, ReadingsPerSecond);
        _rightRms = new(sampleRate, ReadingsPerSecond);
    }

    /// <summary>
    /// Use the same complete source PCM with each call. <paramref name="recordEnd"/>
    /// is the exclusive absolute end of the record. Calls can reuse the last completed
    /// packet and otherwise advance forward, processing the full completion packet.
    /// </summary>
    internal SequenceCaptureRmsSnapshot CompleteRecord(ReadOnlySpan<double> left, ReadOnlySpan<double> right, int recordEnd)
    {
        // recordEnd 是整段 PCM 的右开索引；左右等长才能沿用同一包时钟。
        if (left.Length != right.Length) throw new ArgumentException("The PCM channels must have the same length.", nameof(right));
        if (recordEnd <= _lastPacketStart || recordEnd > left.Length)
            throw new ArgumentOutOfRangeException(nameof(recordEnd));

        // 从上次完整包的末尾继续，直到包右边界覆盖请求的记录终点。
        while (_snapshot.CompletedPacketEnd < recordEnd)
        {
            int start = _snapshot.CompletedPacketEnd;
            int count = _basePacketSize;
            // 小数余量跨包留存；总数满一点就让当前包增长一点，采集中不会逐包丢掉这部分时间。
            _fractionalSamples += _fractionalPacketSize;
            if (_fractionalSamples >= 1)
            {
                count++;
                _fractionalSamples -= 1;
            }
            // 文件结尾只处理实际存在的样本。左、右依次借用工作缓冲，各自的检测器保存自己的状态。
            count = Math.Min(count, left.Length - start);
            var work = _work.AsSpan(0, count);
            _leftAc.Process(left.Slice(start, count), work);
            _leftRms.Process(work);
            _rightAc.Process(right.Slice(start, count), work);
            _rightRms.Process(work);

            // 先给这包的终点贴上此前已发布的电平；恰在本包末尾算出的新包络从下一包的快照才可见。
            int end = start + count;
            _lastPacketStart = start;
            _packets++;
            _snapshot = new(_leftEnvelope, _rightEnvelope, end);

            // GetRmsLevels snapshots the cached envelope before this packet publishes.
            // 每满四分之一秒读取检波器，乘 √2 把正弦的 RMS 换到数字正弦满幅刻度。
            if (_packets % PacketsPerReading == 0)
            {
                _leftEnvelope = _leftRms.Read() * Math.Sqrt(2.0);
                _rightEnvelope = _rightRms.Read() * Math.Sqrt(2.0);
            }
        }
        return _snapshot;
    }
}
