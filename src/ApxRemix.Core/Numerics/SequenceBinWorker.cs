// 此进程仅做数值计算。它与 AP 的对照进程分开编译，按工作表所在进程的位数载入通用 Intel MKL。
using System;
using System.IO;
using System.Runtime.InteropServices;

internal static partial class SequenceBinWorker
{
    // 以下导入由所选架构的 MKL 实际执行，托管工作进程负责安排数据及顺序。ddot 同一窗与自身点乘得到窗能量。
    private const string Mkl = "mkl_rt.dll";
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern double cblas_ddot(int n, double[] x, int incX, double[] y, int incY);
    // DFTI 的描述符保存精度、实数域、长度及已提交的计划；提交后才能正、逆变换，用完释放。
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int DftiCreateDescriptor(out IntPtr descriptor, int precision, int domain, int dimension, int n);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int DftiCommitDescriptor(IntPtr descriptor);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int DftiComputeForward(IntPtr descriptor, [In, Out] double[] buffer);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int DftiComputeBackward(IntPtr descriptor, [In, Out] double[] buffer);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int DftiFreeDescriptor(ref IntPtr descriptor);
    // vzAbs 按每对实、虚分量计算复模，输出是幅值，求功率还须平方。
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern void vzAbs(int count, double[] complex, [Out] double[] magnitude);

    // MKL 的非零返回码表示操作失败。保留状态码，阻止错误的变换结果继续参与测量。
    private static void Check(int status)
    {
        if (status != 0) throw new InvalidOperationException("DFTI status " + status);
    }

    // 将一个弧度相位变成对应窗的时域权重。矩形恒取一，余弦和窗通过选定的谐波权重构造形状。
    private static double Coefficient(string window, double phase)
    {
        switch (window)
        {
            case "Rectangular": return 1;
            // 在 x86 中先将 cos 存入一个 double，再重新载入，会舍掉寄存器的额外有效位。
            case "Hann": return 0.5 + Math.Cos(phase)*0.5;
            case "BlackmanHarris": return 0.42 + Math.Cos(phase)*0.5 + Math.Cos(2*phase)*0.08;
            case "BlackmanHarris4": return 0.35875 + Math.Cos(phase)*0.48829 + Math.Cos(2*phase)*0.14128 + Math.Cos(3*phase)*0.01168;
            // FlatTop 和等波纹分别把 cos(phase) 代入自己的多项式，一次求余弦便可算完整系数。
            case "FlatTop":
                double c=Math.Cos(phase);
                return -0.05828331947022 + c*(0.15550277475633 + c*(0.49819630174675 +
                    c*(0.34465939386184 + c*0.05990373024506)));
            case "Equiripple":
                double u=Math.Cos(phase);
                return 0.06372625603133489 + u*(0.23940452419651895 + u*(0.35000203221155923 +
                    u*(0.24817008904762164 + u*(0.08582735035413477 + u*(0.01242535720360149 + u*0.0004443909552289012)))));
            default: throw new InvalidDataException("Unknown window");
        }
    }

    // 根据窗种类生成 n 个时域系数；Dolph 系列用指定旁瓣衰减的频域设计，其他窗沿等角步长采样。
    private static double[] GenerateWindow(int length, string window)
    {
        switch (window)
        {
            case "Dolph150": return GenerateDolphWindow(length, 150.0);
            case "Dolph200": return GenerateDolphWindow(length, 200.0);
            case "Dolph250": return GenerateDolphWindow(length, 250.0);
        }
        var taper = new double[length];
        double step = 2 * Math.PI / length;
        for (int i = 0; i < length; i++) taper[i] = Coefficient(window, i * step - Math.PI);
        return taper;
    }

    // Dolph 窗先规定频率响应，再逆变换成时间权重；超过设计网格长度时改为在网格中插值。
    private static double[] GenerateDolphWindow(int length, double attenuation)
    {
        // Chebyshev 频域多项式的逆变换生成 Dolph 窗。超过基准频格后在两端点之间作线性插值。
        const int gridLimit = 312000;
        if (length > gridLimit)
        {
            double[] grid = GenerateDolphWindow(gridLimit, attenuation);
            var expanded = new double[length];
            for (int i = 1; i < length - 1; i++)
            // 用目标点位置映回设计网格，整数位置直接取值，其余在两邻点间线性插值，两端另按长度比缩放。
            {
                double position = ((gridLimit - 3.0) * i + length - gridLimit) / (length - 3.0);
                int lower = (int)Math.Floor(position);
                expanded[i] = position == lower ? grid[lower] :
                    (position - lower) * grid[lower + 1] + (lower + 1 - position) * grid[lower];
            }
            expanded[0] = expanded[length - 1] = grid[0] * length / gridLimit;
            return expanded;
        }

        // 将 dB 衰减恢复为线性幅度，由 Chebyshev 阶数求 beta；|x| 超过 1 的部分用双曲函数求值。
        int order = length - 1;
        double beta = Math.Cosh(InverseCosh(Math.Pow(10.0, attenuation / 20.0)) / order);
        int lastHyperbolicBin = (int)Math.Floor(order / Math.PI * Math.Acos(1.0 / beta));
        double step = Math.PI / order;
        var response = new double[2 * (1 + order / 2)];
        double sign = 1.0;
        // 每格交替变号相当于给频谱叠加线性相位，建立生成居中窗所需的频域样本。
        for (int k = 0; k <= order / 2; k++)
        {
            double x = beta * Math.Cos(step * k);
            response[2 * k] = sign * (k <= lastHyperbolicBin ? Math.Cosh(order * InverseCosh(x)) : Math.Cos(order * Math.Acos(x)));
            sign = -sign;
        }
        // 交由 MKL 实逆变换回到时间域；释放描述符后折半第一个权重并复制到末端，凑成要求的窗长。
        IntPtr plan;
        Check(DftiCreateDescriptor(out plan, 36, 33, 1, order));
        try
        {
            Check(DftiCommitDescriptor(plan));
            Check(DftiComputeBackward(plan, response));
        }
        finally { Check(DftiFreeDescriptor(ref plan)); }
        response[0] *= 0.5;
        response[order] = response[0];
        if (response.Length != length) Array.Resize(ref response, length);
        return response;
    }

    // 对 x≥1，用 log 和平方根构造反双曲余弦，供上面的 Chebyshev 设计使用。
    private static double InverseCosh(double x)
    {
        return Math.Log(x + Math.Sqrt(x * x - 1.0));
    }

    // 返回同一帧的整数峰位和功率重心。输入是原始 PCM，因此工作进程内部还需完成窗和 FFT。
    private static void Estimate(double[] pcm, string window, int count, int width, int hint, BinaryWriter output)
    {
        int n = pcm.Length;
        // 窗的和决定直流修正，窗自身点积决定能量归一化；加权样本与直流模板并排准备。
        var taper = GenerateWindow(n, window);
        double sum = 0;
        for (int i = 0; i < n; i++) sum += taper[i];
        // 先标定窗，再变换。若把比例移到 FFT 之后，理论重心相同，但每个频格的末位可能已经改变。
        double gain = Math.Sqrt(2/(n*cblas_ddot(n,taper,1,taper,1)));
        var signal = new double[n+2];
        var dcShape = new double[n+2];
        for (int i=0; i<n; i++)
        {
            dcShape[i] = taper[i]*gain;
            signal[i] = dcShape[i]*pcm[i];
        }
        // 复用同一实变换计划，分别计算模板与信号的半谱，使去直流能按每个复分量相减。
        IntPtr plan;
        Check(DftiCreateDescriptor(out plan, 36, 33, 1, n)); // DFTI_DOUBLE, DFTI_REAL
        try
        {
            Check(DftiCommitDescriptor(plan));
            Check(DftiComputeForward(plan,dcShape));
            Check(DftiComputeForward(plan,signal));
        }
        finally { Check(DftiFreeDescriptor(ref plan)); }
        // 常量乘上窗也有主瓣。仅在直流主瓣内减去这部分，保留其余频格原有的舍入结果。
        // 取零频实部并按窗增益反推直流，再减去它泄漏进主瓣邻格的部分。
        double dcBin = signal[0], inverseDc = 1/(sum*gain);
        signal[0] = 0;
        for (int i=1;i<2*Math.Min(width,n/2+1);i++)
            signal[i] -= dcBin*(dcShape[i]*inverseDc);
        // MKL 先求模再逐格平方，实谱直流与奈奎斯特的权重各折半。
        var power = new double[count];
        vzAbs(count,signal,power);
        for (int i=0;i<count;i++) power[i] *= power[i];
        power[0] *= 0.5;
        if (count==n/2+1) power[count-1] *= 0.5;
        // 提示小于零表示自行找功率最大的格，存在有效提示则围绕指定格计算。
        int peak = hint;
        if (peak<0)
        {
            peak=0;
            for (int k=1;k<count;k++) if (power[k]>power[peak]) peak=k;
        }
        // 峰带遇边界时对称缩窄；sum(k·P)/sum(P) 是以功率为权的分数格，乘 Fs/N 才变成 Hz。
        int radius = Math.Min(width, Math.Min(peak,count-1-peak));
        double mass=0, moment=0;
        for (int k=peak-radius;k<=peak+radius;k++)
        {
            mass += power[k];
            moment += power[k]*k;
        }
        // 应答先送整数峰位再送 double 重心，刷新标准输出以便主进程立即读取。
        output.Write(peak);
        output.Write(moment/mass);
        output.Flush();
    }

#if DESKTOP_WORKSHEET
    // 桌面版本可指定 MKL 的线程数，让本工作进程按确定的线程设置执行。
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern void MKL_Set_Num_Threads(int count);
#endif
    // 该程序以二进制标准输入输出常驻服务；桌面构建先将 MKL 线程数固定为一。
    public static void Main()
    {
#if DESKTOP_WORKSHEET
        // APx500 initializes MKL on the desktop UI host before starting the worksheet.
        MKL_Set_Num_Threads(1);
#endif
        using (var input=new BinaryReader(Console.OpenStandardInput()))
        using (var output=new BinaryWriter(Console.OpenStandardOutput()))
        {
            while (true)
            {
                // 文件结束表示客户端关闭。负数字段是操作码，非负字段则为本条测频消息的采样点数。
                int n;
                try { n=input.ReadInt32(); } catch (EndOfStreamException) { return; }
                if (n < 0)
                {
                    TransformOperation(n, input, output);
                    continue;
                }
                // 按约定顺序读取窗型、频带点数、半宽、提示和 PCM；数据不完整直接报错，不交付半帧的估计。
                string window=input.ReadString();
                int count=input.ReadInt32(), width=input.ReadInt32(), hint=input.ReadInt32();
                var pcm=new double[n];
                byte[] bytes=input.ReadBytes(checked(n*sizeof(double)));
                if (bytes.Length!=n*sizeof(double)) throw new EndOfStreamException();
                Buffer.BlockCopy(bytes,0,pcm,0,bytes.Length);
                Estimate(pcm,window,count,width,hint,output);
            }
        }
    }
}
