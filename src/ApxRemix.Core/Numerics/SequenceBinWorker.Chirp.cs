using System;
using System.IO;
using System.Runtime.InteropServices;

internal static partial class SequenceBinWorker
{
    // vdLinearFrac 对整段做有理线性变换，此处借它批量加常数或缩放；指针重载只处理指定窗段。
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int vdLinearFrac(int count, double[] a, double[] b, double scaleA, double shiftA, double scaleB, double shiftB, double[] result);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static unsafe extern int vdLinearFrac(int count, double* a, double* b, double scaleA, double shiftA, double scaleB, double shiftB, double* result);
    // 幂、正弦、余弦、平方由 MKL 向量函数计算，后两项用于两端的渐弱权重。
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int vdPow(int count, double[] a, double[] b, double[] result);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int vdSin(int count, double[] a, double[] result);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int vdCos(int count, double[] a, double[] result);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern int vdSqr(int count, double[] a, double[] result);
    // vdMul 逐元素相乘，dscal 将整段乘常数；相角与窗系数由这两类操作施加到载波。
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static unsafe extern int vdMul(int count, double* a, double* b, double* result);
    [DllImport(Mkl, CallingConvention = CallingConvention.Cdecl)]
    private static extern void cblas_dscal(int count, double alpha, double[] vector, int increment);

    // 从协议取得采样时刻、端点频率与时间步长，先算对数扫频载波，再处理两端包络。
    private static void GenerateChirp(int length, BinaryReader input, BinaryWriter output)
    {
        double t0 = input.ReadDouble(), t3 = input.ReadDouble();
        double f1 = input.ReadDouble(), f2 = input.ReadDouble(), f3 = input.ReadDouble();
        double step = input.ReadDouble();
        double signalEndTime = input.ReadDouble();
        // 有显式右侧渐弱起点就采用它，否则由 f2 在扫频中的位置反推出秒数。
        double t2 = input.ReadBoolean() ? input.ReadDouble() : t3 / Math.Log(f3 / f1) * Math.Log(f2 / f1);
        var times = new double[length];
        for (int i = 0; i < length; i++) times[i] = t0 + i * step;

        double[] signal = ChirpCarrier(t0, t3, f1, f3, times);
        // 以 step 换算左右窗段各需多少整点，有实际点数才对该段施加包络。
        int left = (int)Math.Floor((0.0 - t0) / step);
        if (left > 0) ApplyChirpTaper(times, signal, 0, left, 0.0, t0);
        double rightStart = signalEndTime - (signalEndTime - t2);
        int right = (int)Math.Floor((signalEndTime - rightStart) / step);
        if (right > 0) ApplyChirpTaper(times, signal, length - right, right, rightStart, signalEndTime);
        WriteVector(output, signal, length);
    }

    // 对指数变化的瞬时频率积分得到相位，用该相位的正弦形成扫频采样。
    private static double[] ChirpCarrier(double t0, double t3, double f1, double f3, double[] times)
    {
        // ratio 是首尾频率比，basis 是一秒的频率倍率；factor 把积分换算成弧度。
        double ratio = f3 / f1;
        double factor = Math.PI * 2.0 * f1 * t3 / Math.Log(ratio);
        double basis = Math.Pow(ratio, 1.0 / t3);
        double initial = Math.PI * 2.0 * ChirpInitialPhase(t0, t3, f1, f3);
        int n = times.Length;
        var signal = new double[n];
        var phaseTimes = (double[])times.Clone();

        // Reproduce the original vector stages with the same MKL. Keep the affine stages separate so each output rounds at the same point.
        // 先算 basis 的时间次方，减一、乘积分常数再加初相，最后由 MKL 一次求出全段正弦。
        vdLinearFrac(n, signal, signal, 1, basis, 0, 1, signal);
        vdLinearFrac(n, phaseTimes, phaseTimes, 1, 0, 0, 1, phaseTimes);
        vdPow(n, signal, phaseTimes, signal);
        vdLinearFrac(n, signal, signal, 1, -1, 0, 1, signal);
        cblas_dscal(n, factor, signal, 1);
        vdLinearFrac(n, signal, signal, 1, initial, 0, 1, signal);
        vdSin(n, signal, signal);
        return signal;
    }

    // 从物理时间 t0 开始播放，积分相位需要一个常量让首样本对应零相位。这里返回该补偿的周数。
    private static double ChirpInitialPhase(double t0, double t3, double f1, double f3)
    {
        double ratio = f3 / f1;
        return f1 * t3 / Math.Log(ratio) * (1.0 - Math.Pow(ratio, t0 / t3));
    }

    // 只对 [start,start+count) 的样本乘包络；固定托管数组后将该区间地址交给向量函数。
    private static unsafe void ApplyChirpTaper(double[] times, double[] signal, int start, int count, double from, double to)
    {
        var weights = new double[count];
        fixed (double* t = &times[start], values = &signal[start], w = weights)
        {
            // 将相对时间换成 0 到 π 的角度，计算 ((1+cosθ)/2)² 并乘载波。from 和 to 的次序决定渐入或渐出。
            vdLinearFrac(count, t, t, 1, 0 - from, 0, 1, w);
            cblas_dscal(count, Math.PI, weights, 1);
            cblas_dscal(count, 1.0 / (to - from), weights, 1);
            vdCos(count, weights, weights);
            vdLinearFrac(count, weights, weights, 0.5, 0.5, 0, 1, weights);
            vdSqr(count, weights, weights);
            vdMul(count, values, w, values);
        }
    }
}
