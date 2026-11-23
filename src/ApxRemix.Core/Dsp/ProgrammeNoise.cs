using System.Numerics;

namespace ApxRemix.Core.Dsp;

// IEC 60268-1 programme-noise profile, interpolated between nominal third-octave
// centres. The underlying pink-noise power density is proportional to 1/f.
internal sealed class ProgrammeNoise
{
    private static readonly double[] Hz = [20,25,31.5,40,50,63,80,100,125,160,200,250,315,400,500,630,800,1000,1250,1600,2000,2500,3150,4000,5000,6300,8000,10000,12500,16000,20000];
    private static readonly double[] Db = [-13.5,-10.2,-7.4,-5.2,-3.5,-2.3,-1.4,-.9,-.5,-.2,-.1,0,0,0,0,0,0,-.1,-.3,-.6,-1,-1.6,-2.5,-3.7,-5.1,-7,-9.4,-11.9,-14.8,-18.2,-21.6];
    private readonly Random _random;
    private readonly double[] _weights, _window;
    private readonly Complex[] _work;
    private double[] _previous, _next;
    private int _position;
    public int Rate { get; }

    public ProgrammeNoise(int rate, Random random)
    {
        Rate=rate;
        _random=random;
        int n=(int)BitOperations.RoundUpToPowerOf2((uint)Math.Clamp(rate/3,4096,65536));
        _weights=new double[n/2];
        _window=new double[n];
        _work=new Complex[n];
        _previous=new double[n];
        _next=new double[n];
        double power=0;
        // 逐格将 FFT 频率映射到目标噪声曲线，以对数频率在相邻标称点之间插值，每个频带因此获得相应的权重。
        for (int k=1;k<_weights.Length;k++)
        {
            double frequency=k*rate/(double)n;
            int j=Array.BinarySearch(Hz,frequency);
            if (j<0) j=~j;
            j=Math.Clamp(j,1,Hz.Length-1);
            double fraction=Math.Log(frequency/Hz[j-1])/Math.Log(Hz[j]/Hz[j-1]);
            double gain=Db[j-1]+fraction*(Db[j]-Db[j-1]);
            // 在幅度域乘 1/√f 后，功率密度随 1/f 变化，相同倍频程的基底功率近似相等。表给定的 dB 再塑造节目噪声频带。
            _weights[k]=Math.Pow(10,gain/20)/Math.Sqrt(frequency);
            power+=2*_weights[k]*_weights[k];
        }
        // 整体归一化保持频谱各处的相对权重，正弦窗用于随后相邻噪声块的重叠输出。
        double normalization=Math.Sqrt(power);
        for(int k=1;k<_weights.Length;k++) _weights[k]/=normalization;
        for(int i=0;i<n;i++) _window[i]=Math.Sin(Math.PI*i/n);
        Generate(_previous);
        Generate(_next);
    }

    private void Generate(double[] data)
    {
        _work[0]=_work[_weights.Length]=Complex.Zero;
        // 随机相位配成共轭正负频率，使变换的实部形成实噪声记录。这里用未归一化正向 FFT 合成，相当于改变时间方向和尺度；初相随机时谱形不变。
        for(int k=1;k<_weights.Length;k++)
        {
            var (sin,cos)=Math.SinCos(_random.NextDouble()*2*Math.PI);
            var bin=new Complex(cos*_weights[k],sin*_weights[k]);
            _work[k]=bin;
            _work[^k]=Complex.Conjugate(bin);
        }
        FftEngine.ForwardInPlace(_work);
        for(int i=0;i<data.Length;i++) data[i]=_work[i].Real*_window[i];
    }

    public double Next(bool reducedCrest)
    {
        // 每输出半个块就更替缓冲。前一块尾部和后一块头部叠加，避免分块边界突跳；峰值处理发生在合成之后。
        if (_position==_weights.Length)
        {
            (_previous,_next)=(_next,_previous);
            Generate(_next);
            _position=0;
        }
        double value=_previous[_position+_weights.Length]+_next[_position++];
        // Equal-power overlap retains the RMS of each independent record.
        // 半块错开的两正弦窗平方和为 1，因此独立记录叠加时的期望功率连续。EN 路径用 tanh 压缩峰值，其余模式缩放后限幅。
        return reducedCrest ? Math.Tanh(value/1.5) : Math.Clamp(value/6,-1,1);
    }
}
