using System.Numerics;

namespace ApxRemix.Core.Dsp;

public enum ThieleSmallModelFit { Standard, Lr2, Wright }

public enum ThieleSmallDerivedMethod { AddedMass, KnownVolume, KnownMmd }

public sealed record ThieleSmallParameters(
    double FsHz,
    double Qms,
    double Qes,
    double Qts,
    double ReOhms,
    double RmsPeakOhms,
    double LeHenries,
    double L2Henries,
    double R2Ohms,
    double WrightKr,
    double WrightXr,
    double WrightKi,
    double WrightXi,
    double FitResidual,
    bool Converged);

/// <summary>
/// 派生 Thiele-Small 机械/声学参数（对齐 <c>AudioPrecision.Om.ThieleSmallMetaData.ComputeTsParams</c>）。
/// 单位与 APx500 一致：Sd (cm²)、Mms (g)、Cms (mm/N)、Vas (L)、Bl (N/A = T·m)、Rms (kg/s)、Eta0 (%)、Spl1W1m (dB SPL @ 1W/1m)。
/// </summary>
public sealed record ThieleSmallDerivedParameters(
    double SdCm2,
    double MmsGrams,
    double CmsMmPerN,
    double VasLiters,
    double BlTm,
    double RmsKgPerS,
    double Eta0Percent,
    double Spl1W1mDb,
    bool IsValid,
    string? ErrorReason = null);

/// <summary>
/// APx500 10.0.0.17 扬声器 Thiele-Small 复阻抗拟合器独立复刻（对齐 <c>AudioPrecision.Processing.Measurements.Chirp.ThieleSmall</c>
/// 与 <c>AudioPrecision.Om.ThieleSmallMetaData</c>）。
/// 支持自动阻抗极性翻转、1.5*Fs 分段对数重采样与 1/24 倍频程平滑、Standard / LR2 / Wright 音圈模型，
/// 采用 3 轮重启 Nelder-Mead 复阻抗相对误差最小化求解自由空气参数，并支持 AddedMass / KnownVolume / KnownMmd 派生参数计算。
/// </summary>
public static class ApxThieleSmallAnalyzer
{
    private static readonly double Octave24Q =
        Math.Sqrt(Math.Pow(2.0, 1.0 / 24.0)) / (Math.Pow(2.0, 1.0 / 24.0) - 1.0);

    public static ThieleSmallParameters ComputeFreeAirTsParams(
        double[] frequenciesHz,
        double[] zReal,
        double[] zImag,
        double fMinHz,
        double fMaxHz,
        double? knownReOhms = null,
        ThieleSmallModelFit modelFit = ThieleSmallModelFit.Standard)
    {
        if (frequenciesHz.Length < 5 || zReal.Length != frequenciesHz.Length || zImag.Length != frequenciesHz.Length)
            throw new ArgumentException("Impedance arrays must have matching length >= 5.");

        var fFull = (double[])frequenciesHz.Clone();
        var reFull = (double[])zReal.Clone();
        var imFull = (double[])zImag.Clone();

        // 1. APx 自动阻抗极性翻转（Om.decompiled.cs:29896-29900）
        if (reFull[0] < 0.0 && reFull[^1] < 0.0)
        {
            for (int i = 0; i < reFull.Length; i++)
            {
                reFull[i] = -reFull[i];
                imFull[i] = -imFull[i];
            }
        }

        // 2. 谐振峰与机械参数初值估计（对齐 XSk8tacFGb / xVO84s9MyV / MCD8P4HWKc）
        var (peakFullIdx, re0, rPeak0, qms0) = EstimateMechanicalInitialGuess(fFull, knownReOhms, reFull, imFull, fMinHz, fMaxHz);

        // 若 APx 策略在极端短序列上未命中有效 Qms，回退至模值峰/几何均值半高宽估计
        if (peakFullIdx <= 0 || !double.IsFinite(qms0) || qms0 <= 0.0)
        {
            (peakFullIdx, re0, rPeak0, qms0) = FallbackMechanicalInitialGuess(fFull, knownReOhms, reFull, imFull, fMinHz, fMaxHz);
        }

        double fs0 = fFull[peakFullIdx];
        var init = new List<double> { fs0, rPeak0, qms0 };
        var fullInit = new List<double> { fs0, rPeak0, qms0, re0, qms0 * re0 / rPeak0, qms0 * re0 / (rPeak0 + re0) };
        if (!knownReOhms.HasValue)
            init.Add(re0);

        // 3. 音圈电感模型初值估计（对齐 gSK89GIgPE，Om.decompiled.cs:30070-30114）
        EstimateVoiceCoilInitialGuess(modelFit, fFull, reFull, imFull, init, fullInit);

        // 4. 1.5*Fs 分段 + 高频段对数重采样 + 1/24 倍频程功率平滑（对齐 qkF8nAO5iH / RJ18hABWHZ / InterpolatePointsSmoothed）
        bool isLinear = IsLinearSpaced(fFull);
        var (lowF, lowRe, lowIm, splitIdx) = SplitLowSegment(fFull, peakFullIdx, reFull, imFull, isLinear);

        double[] fFit, reFitArr, imFitArr;
        if (splitIdx > 0 && splitIdx < fFull.Length - 1)
        {
            var (hiF, hiRe, hiIm) = ResampleHighSegment(fFull, reFull, imFull, splitIdx, lowF.Length, fs0, isLinear);
            double[] smF, smRe, smIm;
            if (hiF.Length > 1)
            {
                SmoothOctave24(hiF, hiRe, hiIm, out smF, out smRe, out smIm);
            }
            else
            {
                smF = hiF;
                smRe = hiRe;
                smIm = hiIm;
            }

            fFit = ConcatArrays(lowF, smF);
            reFitArr = ConcatArrays(lowRe, smRe);
            imFitArr = ConcatArrays(lowIm, smIm);
        }
        else
        {
            fFit = (double[])fFull.Clone();
            reFitArr = (double[])reFull.Clone();
            imFitArr = (double[])imFull.Clone();
        }

        // 裁剪到 [fMinHz, fMaxHz]（对齐 sf98Eu8gbT，Om.decompiled.cs:30218-30251）
        TrimToFrequencyRange(ref fFit, ref reFitArr, ref imFitArr, fMinHz, fMaxHz);
        if (fFit.Length < 5)
            throw new ArgumentException("Insufficient impedance samples in [fMinHz, fMaxHz].");

        // 5. 目标函数（对齐 PB58yCtb4W，Om.decompiled.cs:30624-30664）
        double Objective(double[] x)
        {
            double fs = x[0], rp = x[1], qms = x[2];
            double reVal = knownReOhms ?? x[3];
            int idx = knownReOhms.HasValue ? 3 : 4;
            var model = EvaluateTotalModelZ(fFit, fs, rp, qms, reVal, x.AsSpan(idx), modelFit);
            double sumSq = 0.0;
            for (int i = 0; i < fFit.Length; i++)
            {
                var meas = new Complex(reFitArr[i], imFitArr[i]);
                double mag = meas.Magnitude;
                if (mag <= 1e-15) continue;
                var diff = (model[i] - meas) / mag;
                sumSq += diff.Real * diff.Real + diff.Imaginary * diff.Imaginary;
            }
            return double.IsFinite(sumSq) ? sumSq : 1e12;
        }

        // 6. APx ThieleSmall.ST68fdZgem: 3 轮重启 Nelder-Mead（tol = 1e-9, maxIter = 3000）
        double[] current = init.ToArray();
        double finalVal = double.NaN;
        bool converged = false;
        for (int restart = 0; restart < 3; restart++)
            current = ApxOptimumRegulator.NelderMead(Objective, current, 1e-9, 3000, out finalVal, out _, out converged);

        double fsFit = current[0];
        double rpFit = current[1];
        double qmsFit = current[2];
        double reFit = knownReOhms ?? current[3];
        double qesFit = qmsFit * reFit / rpFit;
        double qtsFit = qmsFit * reFit / (rpFit + reFit);

        int tail = knownReOhms.HasValue ? 3 : 4;
        double le = double.NaN, r2 = double.NaN, l2 = double.NaN;
        double kr = double.NaN, xr = double.NaN, ki = double.NaN, xi = double.NaN;
        switch (modelFit)
        {
            case ThieleSmallModelFit.Standard:
                le = current[tail];
                break;
            case ThieleSmallModelFit.Lr2:
                le = current[tail];
                r2 = current[tail + 1];
                l2 = current[tail + 2];
                break;
            case ThieleSmallModelFit.Wright:
                xr = current[tail];
                kr = current[tail + 1];
                xi = current[tail + 2];
                ki = current[tail + 3];
                break;
        }

        return new ThieleSmallParameters(
            fsFit, qmsFit, qesFit, qtsFit, reFit, rpFit, le, l2, r2, kr, xr, ki, xi, finalVal, converged);
    }

    /// <summary>
    /// 计算派生 Thiele-Small 参数（Sd / Mms / Cms / Vas / Bl / Rms / Eta0 / Spl1W1m），
    /// 精确对齐 <c>AudioPrecision.Om.ThieleSmallMetaData.ComputeTsParams</c>（Om.decompiled.cs:228551-228611）。
    /// <para>
    /// 单位：<paramref name="speakerDiameterCm"/> 为厘米 (cm)，<paramref name="addedMassGrams"/> 为克 (g，KnownMmd 时为振膜质量 Mmd)，
    /// <paramref name="knownVolumeLiters"/> 为升 (L)。
    /// </para>
    /// </summary>
    public static ThieleSmallDerivedParameters ComputeDerivedTsParams(
        ThieleSmallParameters freeAir,
        double speakerDiameterCm,
        ThieleSmallDerivedMethod method,
        ThieleSmallParameters? loaded = null,
        double addedMassGrams = 0.0,
        double knownVolumeLiters = 0.0)
    {
        if (speakerDiameterCm <= 0.0 || !double.IsFinite(speakerDiameterCm))
            throw new ArgumentOutOfRangeException(nameof(speakerDiameterCm), "Speaker diameter (cm) must be positive.");

        var p = loaded ?? freeAir;
        double radiusM = 0.005 * speakerDiameterCm;
        double sdM2 = Math.PI * radiusM * radiusM;
        double sdCm2 = sdM2 * 100.0 * 100.0;

        double fsFree = freeAir.FsHz;
        double fsLoaded = p.FsHz;
        double mmsKg, cmsMPerN, vasM3;

        // ρ₀·c² = 141825.68864 Pa, c³ = 41457661.18100001 m³/s³ (APx500 25°C 标准空气常数)
        const double RhoC2 = 141825.68864;
        const double C3 = 41457661.18100001;
        const double AirLoadCoeff = 3.1612799999999996;

        switch (method)
        {
            case ThieleSmallDerivedMethod.AddedMass:
                if (fsLoaded / fsFree > 0.95)
                {
                    return new ThieleSmallDerivedParameters(
                        sdCm2, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                        IsValid: false, ErrorReason: "AddedMassTooLow: loaded Fs must be <= 0.95 * free-air Fs.");
                }
                mmsKg = addedMassGrams * 0.001 / (Math.Pow(fsFree / fsLoaded, 2.0) - 1.0);
                cmsMPerN = 1.0 / (Math.Pow(Math.Tau * fsFree, 2.0) * mmsKg);
                vasM3 = RhoC2 * sdM2 * sdM2 * cmsMPerN;
                break;

            case ThieleSmallDerivedMethod.KnownVolume:
                if (fsLoaded / fsFree < 1.05)
                {
                    return new ThieleSmallDerivedParameters(
                        sdCm2, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                        IsValid: false, ErrorReason: "KnownVolumeTooLarge: loaded Fs must be >= 1.05 * free-air Fs.");
                }
                vasM3 = knownVolumeLiters * 0.001 * (Math.Pow(fsLoaded / fsFree, 2.0) - 1.0);
                cmsMPerN = vasM3 / (RhoC2 * sdM2 * sdM2);
                mmsKg = 1.0 / (Math.Pow(Math.Tau * fsFree, 2.0) * cmsMPerN);
                break;

            case ThieleSmallDerivedMethod.KnownMmd:
                mmsKg = addedMassGrams * 0.001 + AirLoadCoeff * Math.Pow(speakerDiameterCm * 0.01 * 0.5, 3.0);
                cmsMPerN = 1.0 / (Math.Pow(Math.Tau * fsFree, 2.0) * mmsKg);
                vasM3 = RhoC2 * sdM2 * sdM2 * cmsMPerN;
                break;

            default:
                throw new ArgumentOutOfRangeException(nameof(method));
        }

        double mmsGrams = mmsKg * 1000.0;
        double cmsMmPerN = cmsMPerN * 1000.0;
        double vasLiters = vasM3 * 1000.0;
        double bl = Math.Sqrt(Math.Tau * fsFree * mmsKg * p.ReOhms / p.Qes);
        double rms = bl * bl / p.RmsPeakOhms;
        double eta0Percent = 400.0 * Math.PI * Math.PI * Math.Pow(fsFree, 3.0) * vasM3 / (C3 * p.Qes);
        double spl1W1m = eta0Percent > 0.0 ? 112.1 + 10.0 * Math.Log10(eta0Percent / 100.0) : double.NaN;

        return new ThieleSmallDerivedParameters(
            sdCm2, mmsGrams, cmsMmPerN, vasLiters, bl, rms, eta0Percent, spl1W1m, IsValid: true);
    }

    /// <summary>对齐 <c>ThieleSmall.TotalModelZ</c>（机械谐振支路 + 音圈电感支路）。</summary>
    public static Complex[] EvaluateTotalModelZ(double[] frequenciesHz, double fsHz, double rPeakOhms, double qms,
        double reOhms, ReadOnlySpan<double> voiceCoilParams, ThieleSmallModelFit modelFit)
    {
        double ws = Math.Tau * fsHz;
        var z = new Complex[frequenciesHz.Length];
        for (int i = 0; i < frequenciesHz.Length; i++)
        {
            double w = Math.Tau * frequenciesHz[i];
            var s = new Complex(0.0, w);
            Complex zMech = reOhms + (s * ws * rPeakOhms) / (qms * (ws * ws + s * s) + s * ws);
            Complex zVc = modelFit switch
            {
                ThieleSmallModelFit.Standard => s * voiceCoilParams[0],
                ThieleSmallModelFit.Lr2 => s * (voiceCoilParams[0] + (voiceCoilParams[2] * voiceCoilParams[1]) / (voiceCoilParams[1] + s * voiceCoilParams[2])),
                ThieleSmallModelFit.Wright => new Complex(
                    voiceCoilParams[1] * Math.Pow(w, voiceCoilParams[0]),
                    voiceCoilParams[3] * Math.Pow(w, voiceCoilParams[2])),
                _ => Complex.Zero
            };
            z[i] = zMech + zVc;
        }
        return z;
    }

    private static (int peakFullIdx, double re0, double rPeak0, double qms0) EstimateMechanicalInitialGuess(
        double[] fFull, double? knownRe, double[] reFull, double[] imFull, double fMin, double fMax)
    {
        var fTrim = (double[])fFull.Clone();
        var reTrim = (double[])reFull.Clone();
        var imTrim = (double[])imFull.Clone();
        TrimToFrequencyRange(ref fTrim, ref reTrim, ref imTrim, fMin, fMax);

        int attempt = 0;
        double re0 = double.NaN, rPeak0 = double.NaN, qms0 = double.NaN;
        int trimIdx = 0;
        do
        {
            trimIdx = attempt switch
            {
                0 => FindImagZeroCrossing(imTrim),
                1 => FindRealDerivativeZeroCrossing(reTrim),
                _ => FindMaxIndex(reTrim)
            };
            attempt++;
            if (trimIdx != 0 && trimIdx < fTrim.Length)
            {
                int fullIdx = Array.IndexOf(fFull, fTrim[trimIdx]);
                if (fullIdx >= 0)
                    (re0, rPeak0, qms0) = EstimateReRpeakQms(knownRe, reFull, imFull, fullIdx, fFull);
            }
        }
        while (attempt < 3 && double.IsNaN(qms0));

        int finalFullIdx = trimIdx >= 0 && trimIdx < fTrim.Length ? Array.IndexOf(fFull, fTrim[trimIdx]) : 0;
        return (finalFullIdx, re0, rPeak0, qms0);
    }

    private static (int peakFullIdx, double re0, double rPeak0, double qms0) FallbackMechanicalInitialGuess(
        double[] fFull, double? knownRe, double[] reFull, double[] imFull, double fMin, double fMax)
    {
        int peakIdx = 0;
        double maxZ = 0.0;
        double minRe = double.PositiveInfinity;
        for (int i = 0; i < fFull.Length; i++)
        {
            if (fFull[i] < fMin || fFull[i] > fMax) continue;
            double mag = Math.Sqrt(reFull[i] * reFull[i] + imFull[i] * imFull[i]);
            if (mag > maxZ) { maxZ = mag; peakIdx = i; }
            if (reFull[i] > 0.0 && reFull[i] < minRe) minRe = reFull[i];
        }
        double fs0 = fFull[peakIdx];
        double re0 = knownRe ?? Math.Max(double.IsFinite(minRe) ? minRe : 0.1, 0.1);
        double rPeak0 = Math.Max(maxZ - re0, 0.5);
        double targetMag = Math.Sqrt(re0 * (re0 + rPeak0));
        double fLo = fs0 * 0.7, fHi = fs0 * 1.4;
        for (int i = peakIdx; i >= 0; i--)
            if (Math.Sqrt(reFull[i] * reFull[i] + imFull[i] * imFull[i]) <= targetMag) { fLo = fFull[i]; break; }
        for (int i = peakIdx; i < fFull.Length; i++)
            if (Math.Sqrt(reFull[i] * reFull[i] + imFull[i] * imFull[i]) <= targetMag) { fHi = fFull[i]; break; }
        double r0 = Math.Sqrt((re0 + rPeak0) / Math.Max(re0, 1e-6));
        double bw = Math.Max(fHi - fLo, fs0 * 0.05);
        double qms0 = Math.Clamp(fs0 * r0 / bw, 0.2, 50.0);
        return (peakIdx, re0, rPeak0, qms0);
    }

    private static (double re0, double rPeak0, double qms0) EstimateReRpeakQms(
        double? knownRe, double[] zRe, double[] zIm, int peakIdx, double[] f)
    {
        var zMag = new double[zRe.Length];
        double minMag = double.PositiveInfinity;
        for (int i = 0; i < zRe.Length; i++)
        {
            zMag[i] = Math.Sqrt(zRe[i] * zRe[i] + zIm[i] * zIm[i]);
            if (zMag[i] < minMag) minMag = zMag[i];
        }

        double re0 = knownRe ?? Math.Max(minMag, 0.1);
        double rPeak0 = zMag[peakIdx] - re0;
        double target = rPeak0 / Math.Sqrt(2.0) + re0;

        int? hiIdx = null;
        for (int j = peakIdx + 1; j < zRe.Length; j++)
        {
            if (zRe[j] < target)
            {
                hiIdx = j;
                break;
            }
        }
        if (!hiIdx.HasValue)
            return (re0, rPeak0, double.NaN);

        int idx = hiIdx.Value;
        double magPrev = zMag[idx - 1];
        double fPrev = f[idx - 1];
        double fCross = fPrev + (target - magPrev) * (f[idx] - fPrev) / (zMag[idx] - magPrev);
        double qms0 = f[peakIdx] / (fCross - f[peakIdx] * f[peakIdx] / fCross);
        return (re0, rPeak0, qms0);
    }

    private static void EstimateVoiceCoilInitialGuess(
        ThieleSmallModelFit modelFit, double[] f, double[] zRe, double[] zIm,
        List<double> optInit, List<double> fullInit)
    {
        double ws = Math.Tau * fullInit[0];
        double rPeak = fullInit[1];
        double qms = fullInit[2];
        double re0 = fullInit[3];
        int n = f.Length;
        var resid = new Complex[n];
        for (int i = 0; i < n; i++)
        {
            var s = new Complex(0.0, Math.Tau * f[i]);
            Complex zMech = re0 + (s * ws * rPeak) / (qms * (ws * ws + s * s) + s * ws);
            resid[i] = new Complex(zRe[i], zIm[i]) - zMech;
        }

        double le0 = resid[n - 1].Imaginary / (Math.Tau * f[n - 1]);
        switch (modelFit)
        {
            case ThieleSmallModelFit.Standard:
                optInit.Add(le0);
                fullInit.Add(le0);
                break;

            case ThieleSmallModelFit.Lr2:
            {
                double r2_0 = Math.Max(resid[n - 1].Real, 0.01);
                optInit.AddRange([le0, r2_0, 0.001]);
                fullInit.AddRange([le0, r2_0, 0.001]);
                break;
            }

            case ThieleSmallModelFit.Wright:
            {
                int midIdx = (n - 1) >> 1;
                double wMid = Math.Tau * f[midIdx];
                Complex zMid = resid[midIdx];
                int hiIdx = n - 1;
                double wHi = Math.Tau * f[hiIdx];
                Complex zHi = resid[hiIdx];
                double invLogW = 1.0 / Math.Log(wHi / wMid);
                double reMid = Math.Max(zMid.Real, 0.1);
                double imMid = Math.Max(zMid.Imaginary, 0.1);
                double xr0 = Math.Max(0.0, Math.Log(Math.Max(zHi.Real, 0.1) / reMid) * invLogW);
                double kr0 = reMid / Math.Pow(wMid, xr0);
                double xi0 = Math.Max(0.0, Math.Log(Math.Max(zHi.Imaginary, 0.1) / imMid) * invLogW);
                double ki0 = imMid / Math.Pow(wMid, xi0);
                optInit.AddRange([xr0, kr0, xi0, ki0]);
                fullInit.AddRange([xr0, kr0, xi0, ki0]);
                break;
            }
        }
    }

    private static int FindImagZeroCrossing(double[] zIm)
    {
        var kernel = new double[11];
        double scale = 1.0 / Math.Sqrt(Math.Tau);
        for (int i = 0; i < 11; i++)
        {
            double d = i - 5;
            kernel[i] = scale * Math.Exp(-d * d / 9.0);
        }
        return FindDownwardZeroCrossing(zIm, kernel);
    }

    private static int FindRealDerivativeZeroCrossing(double[] zRe)
    {
        var kernel = new double[11];
        double scale = (2.0 / 9.0) / Math.Sqrt(Math.Tau);
        for (int i = 0; i < 11; i++)
        {
            double d = i - 5;
            kernel[i] = scale * Math.Exp(-d * d / 9.0) * d;
        }
        return FindDownwardZeroCrossing(zRe, kernel);
    }

    private static int FindDownwardZeroCrossing(double[] data, double[] kernel)
    {
        int n = data.Length;
        int kLen = kernel.Length;
        int half = (kLen - 1) >> 1;
        if (n < kLen) return 0;

        double prev = 0.0;
        for (int i = half; i < n - half; i++)
        {
            double val = 0.0;
            for (int k = 0; k < kLen; k++)
                val += data[i - half + k] * kernel[k];

            if (val < 0.0 && prev > 0.0 && (val < -1e-5 || prev > 1e-5) && (prev - val) / prev > 0.5)
                return (-val < prev) ? i : i - 1;
            prev = val;
        }
        return 0;
    }

    private static int FindMaxIndex(double[] arr)
    {
        int best = 0;
        for (int i = 1; i < arr.Length; i++)
            if (arr[i] > arr[best]) best = i;
        return best;
    }

    private static bool IsLinearSpaced(double[] f)
    {
        if (f.Length < 3) return true;
        double d0 = f[1] - f[0];
        for (int i = 2; i < f.Length; i++)
            if (Math.Abs((f[i] - f[i - 1]) - d0) > 0.001) return false;
        return true;
    }

    private static (double[] lowF, double[] lowRe, double[] lowIm, int splitIdx) SplitLowSegment(
        double[] f, int peakIdx, double[] zRe, double[] zIm, bool isLinear)
    {
        int n = f.Length;
        int splitIdx = 0;
        if (isLinear)
        {
            splitIdx = (int)(peakIdx * 1.5);
        }
        else
        {
            double target = f[peakIdx] * 1.5;
            for (int i = peakIdx; i < n - 1; i++)
            {
                if (f[i] <= target && target < f[i + 1])
                {
                    splitIdx = i;
                    break;
                }
            }
        }
        splitIdx = Math.Min(splitIdx, n - 1);
        if (splitIdx <= 0)
            return ([], [], [], splitIdx);

        int count = splitIdx + 1;
        var lowF = new double[count];
        var lowRe = new double[count];
        var lowIm = new double[count];
        Array.Copy(f, 0, lowF, 0, count);
        Array.Copy(zRe, 0, lowRe, 0, count);
        Array.Copy(zIm, 0, lowIm, 0, count);
        return (lowF, lowRe, lowIm, splitIdx);
    }

    private static (double[] hiF, double[] hiRe, double[] hiIm) ResampleHighSegment(
        double[] f, double[] zRe, double[] zIm, int splitIdx, int lowLen, double fs0, bool isLinear)
    {
        double octLow = Math.Ceiling(Math.Log(fs0 / 10.0, 2.0)) + Math.Ceiling(Math.Log(f[splitIdx] / fs0, 2.0));
        double ptsPerOct = lowLen / Math.Max(octLow, 1e-9);
        int n = f.Length;
        int hiStart = (splitIdx == n - 1) ? n - 1 : splitIdx + 1;
        int hiEnd = n - 1;
        double fStart = f[hiStart];
        double fEnd = f[hiEnd];
        int hiCount = (int)(Math.Log(fEnd / fStart) / Math.Log(2.0) * ptsPerOct);

        bool fallback = hiCount < 2;
        var hiF = fallback ? [] : new double[hiCount];
        var hiRe = fallback ? [] : new double[hiCount];
        var hiIm = fallback ? [] : new double[hiCount];

        if (!fallback)
        {
            var seen = new HashSet<double>();
            double ratio = Math.Pow(fEnd / fStart, 1.0 / (hiCount - 1));
            for (int i = 0; i < hiCount; i++)
            {
                double target = fStart * Math.Pow(ratio, i);
                if (isLinear)
                {
                    int idx = hiStart + (int)Math.Round((target - fStart) / (f[1] - f[0]));
                    idx = Math.Clamp(idx, hiStart, hiEnd);
                    hiF[i] = f[idx];
                    hiRe[i] = zRe[idx];
                    hiIm[i] = zIm[idx];
                    continue;
                }

                int found = -1;
                for (int j = hiStart + 1; j < n; j++)
                {
                    if (f[j - 1] == target || (f[j - 1] < target && target < f[j]))
                    {
                        found = j - 1;
                        break;
                    }
                }
                if (found == -1 && target >= f[n - 1]) found = n - 1;
                if (found == -1 || !seen.Add(f[found]))
                {
                    fallback = true;
                    break;
                }
                hiF[i] = f[found];
                hiRe[i] = zRe[found];
                hiIm[i] = zIm[found];
            }
        }

        if (fallback)
        {
            int rawCount = Math.Max(1, hiEnd - hiStart);
            hiF = new double[rawCount];
            hiRe = new double[rawCount];
            hiIm = new double[rawCount];
            Array.Copy(f, hiStart, hiF, 0, rawCount);
            Array.Copy(zRe, hiStart, hiRe, 0, rawCount);
            Array.Copy(zIm, hiStart, hiIm, 0, rawCount);
        }

        return (hiF, hiRe, hiIm);
    }

    private static void SmoothOctave24(
        double[] rawX, double[] rawRe, double[] rawIm,
        out double[] finalX, out double[] finalRe, out double[] finalIm)
    {
        int n = rawX.Length;
        double xMin = rawX[0], xMax = rawX[n - 1];
        var signedSqRe = new double[n];
        var signedSqIm = new double[n];
        for (int i = 0; i < n; i++)
        {
            signedSqRe[i] = double.IsNaN(rawRe[i]) ? 0.0 : rawRe[i] * rawRe[i] * SignedUnit(rawRe[i]);
            signedSqIm[i] = double.IsNaN(rawIm[i]) ? 0.0 : rawIm[i] * rawIm[i] * SignedUnit(rawIm[i]);
        }

        double q = Octave24Q;
        double lowThresh = (rawX[1] - rawX[0]) * q * 1.5;

        var outX = new List<double>(n);
        var outRe = new List<double>(n);
        var outIm = new List<double>(n);

        int pt = 0;
        double? lastSnapped = null;
        double xVal = GraphIdx2XValueLog(pt, xMin, xMax, n);
        while (xVal < lowThresh && xVal <= xMax && pt < n)
        {
            if (xVal < xMin || xVal > xMax)
            {
                outX.Add(xVal);
                outRe.Add(double.NaN);
                outIm.Add(double.NaN);
                lastSnapped = xVal;
            }
            else
            {
                int pos = Array.BinarySearch(rawX, xVal);
                if (pos < 0) pos = ~pos;
                double fracIdx = pos == 0
                    ? 0.0
                    : (pos - 1) + (xVal - rawX[pos - 1]) / (rawX[pos] - rawX[pos - 1]);
                double snapped = rawX[Math.Clamp((int)Math.Round(fracIdx), 0, n - 1)];
                if (lastSnapped != snapped)
                {
                    var (sRe, sIm) = ApplyNthOctavePowerSmooth(q, snapped, rawX, signedSqRe, signedSqIm);
                    outX.Add(snapped);
                    outRe.Add(sRe);
                    outIm.Add(sIm);
                }
                lastSnapped = snapped;
            }
            pt++;
            if (pt < n) xVal = GraphIdx2XValueLog(pt, xMin, xMax, n);
        }

        for (int i = pt; i < n; i++)
        {
            double xi = GraphIdx2XValueLog(i, xMin, xMax, n);
            if (xi < xMin || xi > xMax)
            {
                outX.Add(xi);
                outRe.Add(double.NaN);
                outIm.Add(double.NaN);
            }
            else
            {
                var (sRe, sIm) = ApplyNthOctavePowerSmooth(q, xi, rawX, signedSqRe, signedSqIm);
                outX.Add(xi);
                outRe.Add(sRe);
                outIm.Add(sIm);
            }
        }

        finalX = outX.ToArray();
        finalRe = outRe.ToArray();
        finalIm = outIm.ToArray();
    }

    private static (double re, double im) ApplyNthOctavePowerSmooth(
        double q, double f0, double[] freqs, double[] sqRe, double[] sqIm)
    {
        double wSum = 0.0, accRe = 0.0, accIm = 0.0;
        for (int i = 0; i < freqs.Length; i++)
        {
            double fi = freqs[i];
            double u = (fi / f0 - f0 / fi) * q;
            double u2 = u * u;
            double w = 1.0 / (1.0 + u2 * u2 * u2);
            wSum += w;
            accRe += w * sqRe[i];
            accIm += w * sqIm[i];
        }
        double avgRe = accRe / wSum;
        double avgIm = accIm / wSum;
        return (Math.Sqrt(Math.Abs(avgRe)) * SignedUnit(avgRe), Math.Sqrt(Math.Abs(avgIm)) * SignedUnit(avgIm));
    }

    private static double GraphIdx2XValueLog(int idx, double xMin, double xMax, int totalPts)
    {
        if (idx == 0) return xMin;
        if (idx == totalPts - 1) return xMax;
        return xMin * Math.Pow(xMax / xMin, (double)idx / (totalPts - 1));
    }

    private static double SignedUnit(double v) => v < 0.0 ? -1.0 : v > 0.0 ? 1.0 : 0.0;

    private static void TrimToFrequencyRange(ref double[] f, ref double[] re, ref double[] im, double fMin, double fMax)
    {
        int n = f.Length;
        int start = 0, last = n - 1, end = last;
        bool foundStart = false, foundEnd = false;
        for (int i = 0; i < last; i++)
        {
            if (!foundStart && f[i] >= fMin)
            {
                start = i;
                foundStart = true;
            }
            else if (foundStart && f[i] > fMax)
            {
                end = i - 1;
                foundEnd = true;
                break;
            }
        }
        if (!foundEnd) end = last;
        int count = Math.Min(n, Math.Max(0, end - start + 1));
        var nf = new double[count];
        var nre = new double[count];
        var nim = new double[count];
        Array.Copy(f, start, nf, 0, count);
        Array.Copy(re, start, nre, 0, count);
        Array.Copy(im, start, nim, 0, count);
        f = nf;
        re = nre;
        im = nim;
    }

    private static double[] ConcatArrays(double[] a, double[] b)
    {
        var res = new double[a.Length + b.Length];
        Array.Copy(a, 0, res, 0, a.Length);
        Array.Copy(b, 0, res, a.Length, b.Length);
        return res;
    }
}
