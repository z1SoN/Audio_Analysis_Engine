namespace ApxRemix.Core.Dsp;

/// <summary>
/// APx500 10.0.0.17 数字接口误码率（Bit Test）分析器独立复刻（对齐 <c>AudioPrecision.Processing.Measurements.BitTest.BitErrorMeasurement</c>）。
/// 支持 32 位 LFSR（多项式 0x8C76EF00 = -1938362624）、Walking Ones/Zeros、5 种等差计数斜坡、全 0/全 1 卡死检出及多码流帧边界偏移。
/// </summary>
public sealed class ApxBitErrorAnalyzer
{
    public const int MaxChannels = 16;
    private const int LfsrPolynomial = unchecked((int)0x8C76EF00); // -1938362624

    private readonly int[] _maskedRemainder = new int[MaxChannels];
    private readonly int[] _zeroStreak = new int[MaxChannels];
    private readonly int[] _prevSample = new int[MaxChannels];
    private readonly int[] _totalSamples = new int[MaxChannels];
    private readonly int[] _totalErrors = new int[MaxChannels];
    private readonly int[] _periodSamples = new int[MaxChannels];
    private readonly int[] _periodErrors = new int[MaxChannels];
    private readonly int[] _activeBits = new int[MaxChannels];
    private readonly object _sync = new();

    public ApxBitErrorAnalyzer()
    {
        Reset();
    }

    public void Reset()
    {
        lock (_sync)
        {
            Array.Clear(_maskedRemainder);
            Array.Clear(_zeroStreak);
            Array.Clear(_prevSample);
            Array.Clear(_totalSamples);
            Array.Clear(_totalErrors);
            Array.Clear(_periodErrors);
            Array.Clear(_activeBits);
            for (int i = 0; i < MaxChannels; i++) _periodSamples[i] = -20;
        }
    }

    public void ProcessSamples(ReadOnlySpan<int> samples, ReadOnlySpan<int> bitMask, ReadOnlySpan<int> bitDepth,
        int numChannels, int streamType = 0, int pc = 0, int pd = 0)
    {
        if (samples.IsEmpty || numChannels <= 0) return;
        int start, end;
        switch (streamType)
        {
            case 1:
                start = 147;
                end = 258;
                break;
            case 11:
            case 12:
            case 13:
                start = 46;
                end = (pd >> 4) - 1;
                break;
            case 17:
                start = 1028;
                end = 2819;
                break;
            default:
                start = 0;
                end = samples.Length - 1;
                break;
        }
        end = Math.Min(end, samples.Length - 1);
        if (start > end) start = 0;

        lock (_sync)
        {
            for (int i = start; i <= end; i++)
            {
                int ch = i % numChannels;
                ProcessSampleUnlocked(samples[i], ch, bitMask[ch], bitDepth[ch]);
            }
        }
    }

    public void ProcessSample(int sample, int channel, int bitMask, int bitDepth)
    {
        lock (_sync)
        {
            ProcessSampleUnlocked(sample, channel, bitMask, bitDepth);
        }
    }

    private void ProcessSampleUnlocked(int curr, int ch, int bitMask, int bitDepth)
    {
        curr &= bitMask;
        _periodSamples[ch]++;
        int prev = _prevSample[ch];
        _prevSample[ch] = curr;
        _activeBits[ch] |= curr ^ prev;

        int fullPrev = prev | _maskedRemainder[ch];
        bool msb = (unchecked((uint)fullPrev) & 0x80000000u) == 0x80000000u;
        int predicted = fullPrev << 1;
        if (msb) predicted ^= LfsrPolynomial;
        _maskedRemainder[ch] = predicted & ~bitMask;
        predicted &= bitMask;

        _zeroStreak[ch] = curr == 0 ? _zeroStreak[ch] + 1 : 0;
        if (_zeroStreak[ch] >= 10)
        {
            RecordError(ch);
            return;
        }
        if (curr == prev)
        {
            if (curr == bitMask) RecordError(ch);
            return;
        }

        if ((predicted ^ curr) == 0
            || NextRamp(prev, 1020067840, 33488896, 24576000) == curr
            || NextRamp(prev, 1019805696, 0, 65536) == curr
            || NextRamp(prev, -1020067840, 16711680, 16646144) == curr
            || NextRamp(prev, -1019936768, 4128768, 1572864) == curr
            || NextRamp(prev, 1019936768, 0, 65536) == curr)
        {
            return;
        }

        if (IsSingleBit(prev) && IsSingleBit(curr))
        {
            if (IsAdjacentWalkingBit(prev, curr, bitDepth)) return;
        }
        else if (IsSingleZero(prev, bitMask) && IsSingleZero(curr, bitMask)
            && IsAdjacentWalkingZero(prev, curr, bitDepth, bitMask))
        {
            return;
        }

        RecordError(ch);
    }

    private void RecordError(int ch)
    {
        if (_periodSamples[ch] > 0) _periodErrors[ch]++;
    }

    public void GetReading(int numValidChannels, out int[] activeBits, out int[] dataBits,
        out int[] bitErrors, out int[] bitsReceived, out int[] totalBitErrors, out int[] totalBitsReceived)
    {
        lock (_sync)
        {
            for (int i = 0; i < numValidChannels; i++)
            {
                if (_periodSamples[i] > 0)
                {
                    _totalSamples[i] += _periodSamples[i];
                    _totalErrors[i] += _periodErrors[i];
                }
            }
            activeBits = new int[numValidChannels];
            dataBits = new int[numValidChannels];
            bitErrors = new int[numValidChannels];
            bitsReceived = new int[numValidChannels];
            totalBitErrors = new int[numValidChannels];
            totalBitsReceived = new int[numValidChannels];

            Array.Copy(_periodErrors, bitErrors, numValidChannels);
            if (numValidChannels > 0 && _periodSamples[0] > 0)
                Array.Copy(_periodSamples, bitsReceived, numValidChannels);
            Array.Copy(_totalSamples, totalBitsReceived, numValidChannels);
            Array.Copy(_totalErrors, totalBitErrors, numValidChannels);
            Array.Copy(_activeBits, activeBits, numValidChannels);
            Array.Copy(_prevSample, dataBits, numValidChannels);

            Array.Clear(_periodErrors);
            Array.Clear(_periodSamples);
            Array.Clear(_activeBits);
        }
    }

    /// <summary>生成与 APx BitTest 校验器完全匹配的 32 位 LFSR 伪随机码流（高位对齐 bitMask）。</summary>
    public static int[] GenerateLfsrStream(int sampleCount, int seed = 0x12345600, int bitMask = unchecked((int)0xFFFFFF00))
    {
        var output = new int[sampleCount];
        int state = seed == 0 ? 0x12345600 : seed;
        for (int i = 0; i < sampleCount; i++)
        {
            output[i] = state & bitMask;
            bool msb = (unchecked((uint)state) & 0x80000000u) == 0x80000000u;
            state <<= 1;
            if (msb) state ^= LfsrPolynomial;
        }
        return output;
    }

    public static int CountBits(int sample)
    {
        uint v = unchecked((uint)((sample & 0x55555555) + ((sample & -1431655766) >>> 1)));
        v = (v & 0x33333333u) + ((v & 0xCCCCCCCCu) >> 2);
        v = (v & 0x0F0F0F0Fu) + ((v & 0xF0F0F0F0u) >> 4);
        v = (v & 0x00FF00FFu) + ((v & 0xFF00FF00u) >> 8);
        return (int)((v & 0xFFFFu) + ((v & 0xFFFF0000u) >> 16));
    }

    private static int BitPosition(int singleBit) => CountBits(singleBit - 1);
    private static bool IsSingleBit(int value) => (value & (value - 1)) == 0;
    private static bool IsSingleZero(int value, int bitMask) => IsSingleBit(~value & bitMask);

    private static bool IsAdjacentWalkingBit(int prev, int curr, int bitDepth)
    {
        int p = BitPosition(prev);
        int c = BitPosition(curr);
        int lsbPos = 32 - bitDepth;
        return c == p - 1 || c == p + 1 || (c == 31 && p == lsbPos) || (c == lsbPos && p == 31);
    }

    private static bool IsAdjacentWalkingZero(int prev, int curr, int bitDepth, int bitMask) =>
        IsAdjacentWalkingBit(~prev & bitMask, ~curr & bitMask, bitDepth);

    private static int NextRamp(int prev, int step, int mask, int skipVal)
    {
        int val = prev;
        do
        {
            val = unchecked(val + step);
        }
        while ((val & mask) == skipVal);
        return val;
    }
}
