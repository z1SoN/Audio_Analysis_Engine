namespace ApxRemix.Core.Dsp;

/// <summary>
/// APxRT 10.0.0.17 原生输入均衡器（SetEqFilter，APxRT.c 10004e70 / 1000e3b0）。
/// 维护两个独立槽位（index = 0、1），每个槽位为任意阶实系数 Biquad 级联，支持逐声道使能，
/// 在过零频相检测与主带宽滤波之前原地作用于输入波形。
/// </summary>
public sealed class ApxInputEqFilter
{
    private sealed class Slot
    {
        public bool On1;
        public bool On2;
        public StreamingIirCascade Left = StreamingIirCascade.Bypass();
        public StreamingIirCascade Right = StreamingIirCascade.Bypass();
    }

    private readonly Slot[] _slots = [new(), new()];

    /// <summary>
    /// 配置指定槽位（0 或 1）的 IIR Biquad 均衡器系数，对齐 <c>TwoChannelMeasurer.SetEqFilter</c>。
    /// 每节传递函数为 <c>(1 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)</c>，整体乘 <paramref name="overallGain"/>。
    /// </summary>
    public void SetEqFilter(int order, double[] b1, double[] b2, double[] a1, double[] a2,
        double overallGain, int index, bool on1, bool on2)
    {
        if (index is < 0 or > 1)
            throw new ArgumentOutOfRangeException(nameof(index), "APxRT SetEqFilter supports slot 0 or 1.");
        int sectionCount = order / 2;
        var slot = _slots[index];
        slot.On1 = on1;
        slot.On2 = on2;
        if (sectionCount <= 0)
        {
            slot.Left = StreamingIirCascade.Bypass();
            slot.Right = StreamingIirCascade.Bypass();
            return;
        }
        var sections = new StreamingIirCascade.BiquadCoeffs[sectionCount];
        for (int i = 0; i < sectionCount; i++)
            sections[i] = new(1.0, b1[i], b2[i], a1[i], a2[i]);

        bool sameOrder = slot.Left.SectionCount == sectionCount;
        var nextLeft = new StreamingIirCascade(sections, overallGain, useApxStateOrder: true);
        var nextRight = new StreamingIirCascade(sections, overallGain, useApxStateOrder: true);
        // APxRT 1000e3b0: 同阶更新系数时保留已有延迟历史，仅在阶数改变时重建滤波节。
        if (sameOrder)
        {
            nextLeft.CopyHistoryFrom(slot.Left);
            nextRight.CopyHistoryFrom(slot.Right);
        }
        slot.Left = nextLeft;
        slot.Right = nextRight;
    }

    public void ProcessInPlace(Span<double> left, Span<double> right)
    {
        for (int i = 0; i < _slots.Length; i++)
        {
            var slot = _slots[i];
            if (slot.On1 && !slot.Left.IsBypass) slot.Left.ProcessInPlace(left);
            if (slot.On2 && !slot.Right.IsBypass) slot.Right.ProcessInPlace(right);
        }
    }

    public void Reset()
    {
        foreach (var slot in _slots)
        {
            slot.Left.Reset();
            slot.Right.Reset();
        }
    }
}
