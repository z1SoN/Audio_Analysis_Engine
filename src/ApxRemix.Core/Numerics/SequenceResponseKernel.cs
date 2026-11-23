using System;
using System.Numerics;

namespace ApxRemix.Core.Dsp
{
    // 此文件同时编入托管 Core 和 x86 数值进程，让两条路径复用同一套顺序。
    internal static class SequenceResponseKernel
    {
        // 两个输入半谱分别来自用于反卷积的参考和真正播放的激励。输出第一组去卷积，第二组还原读数。
        internal static void Design(int n, double fs, double startHz, double stopHz, bool digital,
            Complex[] referenceFft, Complex[] driveFft, out Complex[] inverse, out Complex[] correction)
        {
            // 将数字或模拟刻度除以每格参考复谱，得到可同时抵消参考幅度和相位的逆传递函数。
            var scale = new Complex(digital ? -1 : -1 / Math.Sqrt(2), 0);
            inverse = new Complex[n / 2 + 1];
            var magnitude = new double[inverse.Length];
            for (int k = 0; k < inverse.Length; k++)
            {
                inverse[k] = scale / referenceFft[k];
                magnitude[k] = inverse[k].Magnitude;
            }
            // Hz 乘 n/fs 换算为频格；从两个带边的逆增益确定低频和高频各自的上限。
            int low = (int)Math.Round(startHz / fs * n, MidpointRounding.AwayFromZero);
            int high = (int)Math.Round(stopHz / fs * n, MidpointRounding.AwayFromZero);
            double lowLimit = magnitude[low] * 100;
            double highLimit = magnitude[high] * 1.1;
            // 频带外参考较弱时倒数会很大。超过限额的格等比缩小实虚两部，保留原相位；带内没有上限。
            for (int k = 0; k < inverse.Length; k++)
            {
                double limit = k <= low ? lowLimit : k > high ? highLimit : double.PositiveInfinity;
                if (magnitude[k] > limit)
                {
                    double gain = limit / magnitude[k];
                    inverse[k] = new Complex(inverse[k].Real * gain, inverse[k].Imaginary * gain);
                }
            }
            // 输出曲线使用较短的变换，每个新格对应原谱的偶数格。对实际激励与逆响应的乘积求校正。
            correction = new Complex[inverse.Length / 2 + 1];
            for (int k = 0; k < correction.Length; k++)
                correction[k] = scale / (driveFft[2*k] * inverse[2*k]);

            // 校正谱取自未移位的逆谱。随后每隔一格变号，令实数逆 FFT 的零延迟位于帧中央。
            // 最后只将逆谱的偶格反号，交替符号带来的半帧时移会把脉冲响应移向记录中心。
            for (int k = 0; k < inverse.Length; k += 2)
                inverse[k] = new Complex(-inverse[k].Real, -inverse[k].Imaginary);
        }
    }
}
