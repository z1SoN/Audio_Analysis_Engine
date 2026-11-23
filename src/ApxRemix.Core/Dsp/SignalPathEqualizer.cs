using System.Numerics;
using ApxRemix.Core.Models;

namespace ApxRemix.Core.Dsp;

// Independent minimum-phase realization of a frequency/gain table. Overlap-add keeps the
// response continuous across device callbacks, including callbacks smaller than the kernel.
public sealed class SignalPathEqualizer
{
    private sealed class Channel
    {
        private const int Size=4096, Taps=Size/2;
        private readonly Complex[] _spectrum=new Complex[Size];
        private readonly Complex[] _work=new Complex[Size];
        private readonly double[] _tail=new double[Taps-1];
        private readonly double? _flat;
        // 将频率（Hz）和增益（dB）表转换成可流式卷积的单声道滤波器。曲线恒定时直接乘线性系数。
        public Channel(BenchTrace curve,int rate)
        {
            if (!FrequencyGainTable.ValidTable(curve) || curve.Y.Any(v=>Math.Abs(v)>100))
                throw new ArgumentException("EQ requires a valid frequency table with gains between −100 and 100 dB.");
            if (curve.Y.All(v=>v==curve.Y[0])) { _flat=Math.Pow(10,curve.Y[0]/20); return; }
            // 正负频率使用相同目标幅度，把 dB 换成自然对数幅度，供倒谱法重建最小相位响应。
            for (int i=0;i<Size;i++)
            {
                double hz=Math.Min(i,Size-i)*(double)rate/Size;
                _spectrum[i]=FrequencyGainTable.GainAtHz(curve,Math.Max(hz,1))*Math.Log(10)/20;
            }
            // 对数幅度逆变换得到实倒谱。正倒频项加倍，负倒频项清零，零项与中点保留，使重建响应带有相应相位。
            FftEngine.InverseInPlace(_spectrum);
            for(int i=1;i<Taps;i++) _spectrum[i]*=2;
            Array.Clear(_spectrum,Taps+1,Taps-1);
            FftEngine.ForwardInPlace(_spectrum);
            // 取复指数恢复传递函数，再变回时域并截取前 2048 个抽头。这一截断使实现的频响只近似目标表。
            for(int i=0;i<Size;i++) _spectrum[i]=Complex.Exp(_spectrum[i]);
            FftEngine.InverseInPlace(_spectrum);
            Array.Clear(_spectrum,Taps,Taps);
            FftEngine.ForwardInPlace(_spectrum);
        }
        // 以块卷积处理任意长度音频，同时保存上批样本的卷积尾部。输出保持与输入相同的点数和单位。
        public void Process(Span<double> samples)
        {
            if (_flat is {} factor)
            {
                for(int i=0;i<samples.Length;i++) samples[i]*=factor;
                return;
            }
            for(int offset=0;offset<samples.Length;offset+=Taps)
            {
                // 每段最多 2048 点，补零到 4096 点，因此与 2048 点冲激响应卷积时无需让尾部绕回开头。
                int count=Math.Min(Taps,samples.Length-offset);
                Array.Clear(_work);
                for(int i=0;i<count;i++) _work[i]=samples[offset+i];
                FftEngine.ForwardInPlace(_work);
                for(int i=0;i<Size;i++) _work[i]*=_spectrum[i];
                FftEngine.InverseInPlace(_work);
                // 将上次尚未发出的尾部与当前结果相加，仅输出本次到达的点数，其后的贡献留给下一段，短段也适用。
                for(int i=0;i<_tail.Length;i++) _work[i]+=_tail[i];
                for(int i=0;i<count;i++) samples[offset+i]=_work[i].Real;
                for(int i=0;i<_tail.Length;i++) _tail[i]=_work[i+count].Real;
            }
        }
    }
    private PathEqSettings? _settings;
    private int _rate;
    private Channel? _left, _right;
    // 设置或采样率改变时重新设计滤波器。跟踪第一声道只共用目标曲线，左右卷积尾部仍由独立实例保存。
    public void Process(PathEqSettings? settings,int rate,Span<double> left,Span<double> right)
    {
        if (settings!=_settings || rate!=_rate)
        {
            var other=settings?.TrackFirst==true ? settings.First : settings?.Second;
            _left=settings?.First is {Enabled:true} first ? new(first.Table,rate) : null;
            _right=other is {Enabled:true} second ? new(second.Table,rate) : null;
            _settings=settings; _rate=rate;
        }
        _left?.Process(left); _right?.Process(right);
    }
}
