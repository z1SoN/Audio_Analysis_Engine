using System;
using System.IO;
using System.Numerics;
using System.Runtime.InteropServices;
using ApxRemix.Core.Dsp;

internal static partial class SequenceBinWorker
{
    // 开方和复乘分别作用于实数向量与交错复向量，反变换的 1/N 缩放交给 MKL 的 dscal。
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern void vdSqrt(int count, double[] input, [Out] double[] output);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern void vzMul(int count, double[] a, double[] b, [Out] double[] output);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl, EntryPoint = "cblas_dscal")]
    private static extern void ScaleInverse(int count, double scale, [In, Out] double[] data, int stride);
    // 按 double 数目换算要读取的字节数；短读不能解释为完整数组，直接抛出文件结束。
    private static double[] ReadVector(BinaryReader input, int length)
    {
        byte[] bytes = input.ReadBytes(checked(length * sizeof(double)));
        if (bytes.Length != length * sizeof(double)) throw new EndOfStreamException();
        var values = new double[length];
        Buffer.BlockCopy(bytes, 0, values, 0, bytes.Length);
        return values;
    }

    // 只回传调用方指定的元素数。变换缓冲可能为半谱多留位置，不能无条件写出其全部内容。
    private static void WriteVector(BinaryWriter output, double[] values, int count)
    {
        byte[] bytes = new byte[checked(count * sizeof(double))];
        Buffer.BlockCopy(values, 0, bytes, 0, bytes.Length);
        output.Write(bytes);
    }

    // 原地实 FFT 需要为 N/2+1 个复数预留空间，以两个 double 为一格，由 MKL 提交并执行计划。
    private static double[] ForwardReal(double[] values)
    {
        int n = values.Length;
        Array.Resize(ref values, 2 * (n / 2 + 1));
        IntPtr plan;
        Check(DftiCreateDescriptor(out plan, 36, 33, 1, n));
        try
        {
            Check(DftiCommitDescriptor(plan));
            Check(DftiComputeForward(plan, values));
        }
        // 无论执行成功与否都释放本次变换描述符，返回的数组包含交错复谱。
        finally { Check(DftiFreeDescriptor(ref plan)); }
        return values;
    }

    // 每两个 double 的次序是先实后虚，组合成内存中可直接用于复运算的数组。
    private static Complex[] ReadComplex(BinaryReader input, int count)
    {
        var raw = ReadVector(input, 2 * count);
        var values = new Complex[count];
        for (int k = 0; k < count; k++) values[k] = new Complex(raw[2*k],raw[2*k+1]);
        return values;
    }

    // 写回协议时逐格拆成实、虚两槽，保持和 ReadComplex 对称的顺序。
    private static void WriteComplex(BinaryWriter output, Complex[] values)
    {
        var raw = new double[values.Length * 2];
        for (int k = 0; k < values.Length; k++)
        {
            raw[2*k] = values[k].Real;
            raw[2*k+1] = values[k].Imaginary;
        }
        WriteVector(output,raw,raw.Length);
    }

    // 同一窗口同时服务直流还原、单音幅值与功率。先求其和与自身点积，分别得到相关校正量。
    private static void WindowParameters(int n, string type, int width, BinaryWriter output)
    {
        double[] coefficients = GenerateWindow(n, type);
        double sum = 0;
        foreach (double w in coefficients) sum += w;
        double gain = Math.Sqrt(2 / (n * cblas_ddot(n, coefficients, 1, coefficients, 1)));
        double dcCorrection = 1 / (sum * gain);
        // 应答前两个数是直流和峰值校正，然后是已归一化的时域窗。
        output.Write(dcCorrection);
        output.Write(Math.Sqrt(2) / (sum * gain));
        for (int i = 0; i < n; i++) coefficients[i] *= gain;
        WriteVector(output, coefficients, n);
        // 全一信号加窗恰等于窗本身。变换该数组，再按直流增益缩放低频格，得到可扣除的直流泄漏模板。
        var dcShape = ForwardReal(coefficients);
        for (int i = 0; i < 2 * width; i++) dcShape[i] *= dcCorrection;
        WriteVector(output, dcShape, 2 * width);
    }

    // 操作码决定 length 表示时域点数、复数格数或 double 槽数，各分支按请求端约定解析。
    private static void TransformOperation(int operation, BinaryReader input, BinaryWriter output)
    {
        int length = input.ReadInt32();
        switch (operation)
        {
            // −1 返回完整窗参数，−2 对实样本返回含直流与最高格的复数半谱。
            case -1:
                WindowParameters(length, input.ReadString(), input.ReadInt32(), output);
                break;
            case -2:
                var fft = ForwardReal(ReadVector(input, length));
                WriteVector(output, fft, fft.Length);
                break;
            // −3 将实虚两槽变成一个幅值，再平方回复每格功率。
            case -3:
                var samples = ReadVector(input, length);
                var power = new double[length / 2];
                vzAbs(power.Length, samples, power);
                for (int k = 0; k < power.Length; k++) power[k] *= power[k];
                WriteVector(output, power, power.Length);
                break;
            // −4 批量开方；−5 返回未经归一化的时域窗，−10 则按时间生成实际扫频。
            case -4:
                var roots = new double[length];
                vdSqrt(length, ReadVector(input, length), roots);
                WriteVector(output, roots, length);
                break;
            case -5:
                WriteVector(output, GenerateWindow(length, input.ReadString()), length);
                break;
            case -10:
                GenerateChirp(length, input, output);
                break;
            // −11 对采集块做 FFT，再逐格乘参考逆响应，逆实 FFT 回到时域。
            case -11:
                var acquired = ForwardReal(ReadVector(input,length));
                var inverse = ReadVector(input,acquired.Length);
                vzMul(acquired.Length / 2, acquired, inverse, acquired);
                IntPtr plan;
                Check(DftiCreateDescriptor(out plan, 36, 33, 1, length));
                try
                {
                    // MKL 的该逆变换未归一化，按点数除一次，回传长度与原始采集相同。
                    Check(DftiCommitDescriptor(plan));
                    Check(DftiComputeBackward(plan, acquired));
                    ScaleInverse(length, 1.0 / length, acquired, 1);
                }
                finally { Check(DftiFreeDescriptor(ref plan)); }
                WriteVector(output,acquired,length);
                break;
            // −12 逐格将响应复谱乘频率校正，再取模得到用于绘图的线性电平。
            case -12:
                var bins = ReadVector(input,2 * length);
                var multiplier = ReadVector(input,2 * length);
                vzMul(length,bins,multiplier,bins);
                var magnitude = new double[length];
                vzAbs(length,bins,magnitude);
                WriteVector(output,magnitude,length);
                break;
            // −14 在指定相角范围等间隔采样等波纹窗；右端点不计入这些样本。
            case -14:
                double start = input.ReadDouble(), stop = input.ReadDouble();
                double step = (stop - start) / length;
                var taper = new double[length];
                for (int i = 0; i < length; i++) taper[i] = i * step + start;
                for (int i = 0; i < length; i++) taper[i] = Coefficient("Equiripple",taper[i]);
                WriteVector(output,taper,length);
                break;
            // −15 从两段半谱设计反卷积和校正系数，按该顺序写回两组复数。
            case -15:
                double fs = input.ReadDouble(), firstHz = input.ReadDouble(), lastHz = input.ReadDouble();
                bool digital = input.ReadBoolean();
                int count = length / 2 + 1;
                Complex[] kernel, equalizer;
                SequenceResponseKernel.Design(length,fs,firstHz,lastHz,digital,ReadComplex(input,count),
                    ReadComplex(input,count),out kernel,out equalizer);
                WriteComplex(output,kernel);
                WriteComplex(output,equalizer);
                break;
            // 其余操作交给相位及残差分派，只有处理结束才刷新完整应答。
            default: ToneMathOperation(operation, length, input, output); break;
        }
        output.Flush();
    }
}
