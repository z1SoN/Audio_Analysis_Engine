namespace ApxRemix.Core.Dsp;

public enum OptimizationObjective { Minimize, Maximize }

public sealed record OptimizationPoint(double X, double Y);

public sealed record OptimumRegulationResult<TZData>(
    double OptimalX,
    double OptimalY,
    TZData Data,
    OptimizationPoint[] History,
    bool Converged);

/// <summary>
/// APx500 10.0.0.17 极值调节器与无导数优化器独立复刻（对齐 <c>AudioPrecision.Processing.Regulation.Optimization</c>
/// 与 <c>OptimumRegulation&lt;ZDataType&gt;</c>：Brent 黄金分割/抛物线插值一维搜索 + N 维 Nelder-Mead 单形求解器）。
/// </summary>
public static class ApxOptimumRegulator
{
    private static readonly double GoldenRatioComplement = 0.5 * (3.0 - Math.Sqrt(5.0));

    public delegate double EvalWithData<TZData>(double x, out TZData data);

    /// <summary>对齐 <c>Optimization.BrentMin</c> / <c>OptimumRegulation.Regulate</c>。</summary>
    public static OptimumRegulationResult<TZData> RegulateOptimum<TZData>(
        EvalWithData<TZData> evaluate,
        double xStart,
        double xStop,
        int initialSteps = 10,
        double absTol = 0.0,
        double relTol = 0.0001,
        int measurementsPerPoint = 1,
        OptimizationObjective objective = OptimizationObjective.Minimize)
    {
        if (xStart == xStop)
            throw new ArgumentException("Search bounds xStart and xStop must differ.");
        initialSteps = Math.Max(2, initialSteps);
        measurementsPerPoint = Math.Max(1, measurementsPerPoint);

        var history = new List<OptimizationPoint>();
        var cacheY = new Dictionary<double, double>();
        var cacheZ = new Dictionary<double, TZData>();

        double EvalCached(double x, out TZData z)
        {
            if (cacheY.TryGetValue(x, out double cached) && cacheZ.TryGetValue(x, out var cz))
            {
                z = cz;
                return cached;
            }
            double sum = 0.0;
            z = default!;
            for (int m = 0; m < measurementsPerPoint; m++)
            {
                double raw = evaluate(x, out z);
                if (!double.IsFinite(raw)) throw new InvalidOperationException($"Non-finite evaluation at x={x}.");
                history.Add(new(x, raw));
                sum += raw;
            }
            double avg = sum / measurementsPerPoint;
            double obj = objective == OptimizationObjective.Minimize ? avg : -avg;
            cacheY[x] = obj;
            cacheZ[x] = z;
            return obj;
        }

        // 1. 粗扫描寻找包含极小值的三点括号 [a, b]
        double fStart = EvalCached(xStart, out TZData bestZ);
        double xPrev = xStart, fPrev = fStart;
        double step = (xStop - xStart) / initialSteps;
        double xCurr = xStart + step;
        double fCurr = EvalCached(xCurr, out bestZ);
        double a = xStart, b = xStop;
        bool bracketed = false;

        for (int i = 2; i <= initialSteps; i++)
        {
            if (fCurr >= fPrev)
                continue;
            double xNext = xStart + i * step;
            double fNext = EvalCached(xNext, out bestZ);
            if (fNext > fCurr)
            {
                bracketed = true;
                a = xPrev;
                b = xNext;
                break;
            }
            xPrev = xCurr; fPrev = fCurr;
            xCurr = xNext; fCurr = fNext;
        }

        if (!bracketed)
        {
            double fStop = EvalCached(xStop, out bestZ);
            double edgeX = fStart < fStop ? xStart : xStop;
            double edgeF = Math.Min(fStart, fStop);
            double refinedF = edgeF;
            int steps = initialSteps;
            for (int pass = 1; pass < 3 && !bracketed; pass++)
            {
                steps *= 3;
                step = (xStop - xStart) / steps;
                for (int k = 1; k < steps; k++)
                {
                    if (k % 3 == 0) continue;
                    double xProbe = xStart + k * step;
                    refinedF = EvalCached(xProbe, out bestZ);
                    if (refinedF < edgeF)
                    {
                        b = xProbe - step;
                        a = xProbe + (3 - k % 3) * step;
                        bracketed = true;
                        break;
                    }
                }
            }
            if (!bracketed)
            {
                double finalY = objective == OptimizationObjective.Minimize ? edgeF : -edgeF;
                return new(edgeX, finalY, cacheZ[edgeX], history.ToArray(), true);
            }
        }

        // 2. Brent 抛物线 + 黄金分割精修（最多 50 步，精确对齐 Optimization.BrentMin / VUjKrqHGfM，Om.decompiled.cs:4606-4790）
        double effAbsTol = absTol;
        double effRelTol = relTol;
        if (effAbsTol == 0.0 && effRelTol == 0.0) effRelTol = 1e-4;
        if (absTol == 0.0 && a * b < 0.0) effAbsTol = 1e-6 * (b - a);

        double x = a + GoldenRatioComplement * (b - a);
        double w = x, v = x;
        double fx = EvalCached(x, out bestZ);
        double fw = fx, fv = fx;
        double d = 0.0, e = 0.0;
        double loBound = Math.Min(xStart, xStop);
        double hiBound = Math.Max(xStart, xStop);
        if (a > b) (a, b) = (b, a);
        bool converged = false;

        for (int iter = 1; iter <= 50; iter++)
        {
            double xm = 0.5 * (a + b);
            double tol1 = 1.4901161193847656e-8 * Math.Abs(x);
            double tol2 = 2.0 * tol1;
            if (Math.Abs(x - xm) > tol2 - 0.5 * (b - a))
            {
                double p = 0.0, q = 0.0, r = 0.0;
                if (Math.Abs(e) > tol1)
                {
                    r = (x - w) * (fx - fv);
                    q = (x - v) * (fx - fw);
                    p = (x - v) * q - (x - w) * r;
                    q = 2.0 * (q - r);
                    if (q > 0.0) p = -p; else q = -q;
                    r = e;
                    e = d;
                }

                double u;
                if (Math.Abs(p) < 0.5 * Math.Abs(q * r) && p > q * (a - x) && p < q * (b - x))
                {
                    d = p / q;
                    u = x + d;
                    if (u - a < tol2 || b - u < tol2) d = x < xm ? tol1 : -tol1;
                }
                else
                {
                    e = (x < xm ? b : a) - x;
                    d = GoldenRatioComplement * e;
                }

                u = x + (Math.Abs(d) >= tol1 ? d : (d > 0.0 ? tol1 : -tol1));
                u = Math.Clamp(u, loBound, hiBound);
                double fu = EvalCached(u, out TZData zu);

                if (fu <= fx)
                {
                    if (u < x) b = x; else a = x;
                    v = w; fv = fw;
                    w = x; fw = fx;
                    x = u; fx = fu; bestZ = zu;
                }
                else
                {
                    if (u < x) a = u; else b = u;
                    if (fu <= fw || w == x) { v = w; fv = fw; w = u; fw = fu; }
                    else if (fu <= fv || v == x || v == w) { v = u; fv = fu; }
                }
            }

            double stopTol = effAbsTol + effRelTol * Math.Abs(x);
            double xmPost = 0.5 * (a + b);
            if (Math.Abs(x - xmPost) <= 2.0 * stopTol - 0.5 * (b - a))
            {
                converged = true;
                break;
            }
        }

        double reportedY = objective == OptimizationObjective.Minimize ? fx : -fx;
        return new(x, reportedY, bestZ, history.ToArray(), converged);
    }

    /// <summary>对齐 <c>Optimization.NelderMead</c>（5% 初始单形步长、方差 &lt; tol² 或单形尺寸 &lt; 1e-12 终止）。</summary>
    public static double[] NelderMead(Func<double[], double> f, ReadOnlySpan<double> x0,
        double tol, int maxIterations, out double finalValue, out int iterations, out bool converged)
    {
        int n = x0.Length;
        int m = n + 1;
        double tolSq = tol * tol;
        var simplex = new double[m][];
        var fvals = new double[m];
        simplex[0] = x0.ToArray();
        fvals[0] = f(simplex[0]);

        for (int j = 1; j < m; j++)
        {
            simplex[j] = x0.ToArray();
            simplex[j][j - 1] = x0[j - 1] != 0.0 ? 1.05 * x0[j - 1] : 0.00025;
            fvals[j] = f(simplex[j]);
        }

        iterations = 0;
        var centroid = new double[n];
        while (iterations <= maxIterations)
        {
            FindExtrema(fvals, out int best, out int worst, out int secondWorst);
            double variance = SampleVariance(fvals);
            double size = SimplexDiameter(simplex);
            if (size < 1e-12 || variance < tolSq || fvals[best] == fvals[worst])
            {
                finalValue = fvals[best];
                converged = true;
                return (double[])simplex[best].Clone();
            }

            Array.Clear(centroid);
            for (int i = 0; i < m; i++)
            {
                if (i == worst) continue;
                for (int k = 0; k < n; k++) centroid[k] += simplex[i][k];
            }
            for (int k = 0; k < n; k++) centroid[k] /= n;

            var reflected = new double[n];
            for (int k = 0; k < n; k++) reflected[k] = 2.0 * centroid[k] - simplex[worst][k];
            double fRef = f(reflected);

            if (fRef >= fvals[best] && fRef < fvals[secondWorst])
            {
                simplex[worst] = reflected;
                fvals[worst] = fRef;
            }
            else if (fRef < fvals[best])
            {
                var expanded = new double[n];
                for (int k = 0; k < n; k++) expanded[k] = centroid[k] + 2.0 * (centroid[k] - simplex[worst][k]);
                double fExp = f(expanded);
                if (fExp < fRef) { simplex[worst] = expanded; fvals[worst] = fExp; }
                else { simplex[worst] = reflected; fvals[worst] = fRef; }
            }
            else
            {
                bool contracted = false;
                var trial = new double[n];
                if (fRef < fvals[worst])
                {
                    for (int k = 0; k < n; k++) trial[k] = 0.5 * (reflected[k] + centroid[k]);
                    double fCon = f(trial);
                    if (fCon < fRef) { simplex[worst] = trial; fvals[worst] = fCon; contracted = true; }
                }
                else
                {
                    for (int k = 0; k < n; k++) trial[k] = 0.5 * (simplex[worst][k] + centroid[k]);
                    double fCon = f(trial);
                    if (fCon < fvals[worst]) { simplex[worst] = trial; fvals[worst] = fCon; contracted = true; }
                }
                if (!contracted)
                {
                    for (int i = 0; i < m; i++)
                    {
                        if (i == best) continue;
                        for (int k = 0; k < n; k++) simplex[i][k] = 0.5 * (simplex[best][k] + simplex[i][k]);
                        fvals[i] = f(simplex[i]);
                    }
                }
            }
            iterations++;
        }

        FindExtrema(fvals, out int bestFinal, out _, out _);
        finalValue = fvals[bestFinal];
        converged = false;
        return (double[])simplex[bestFinal].Clone();
    }

    private static void FindExtrema(double[] vals, out int best, out int worst, out int secondWorst)
    {
        best = 0; worst = 0;
        for (int i = 1; i < vals.Length; i++)
        {
            if (vals[i] < vals[best]) best = i;
            if (vals[i] > vals[worst]) worst = i;
        }
        secondWorst = best;
        for (int i = 0; i < vals.Length; i++)
        {
            if (i != best && i != worst && vals[i] > vals[secondWorst]) secondWorst = i;
        }
    }

    private static double SampleVariance(double[] vals)
    {
        double mean = 0.0;
        for (int i = 0; i < vals.Length; i++) mean += vals[i];
        mean /= vals.Length;
        double ss = 0.0;
        for (int i = 0; i < vals.Length; i++) { double d = vals[i] - mean; ss += d * d; }
        return vals.Length > 1 ? Math.Abs(ss / (vals.Length - 1)) : 0.0;
    }

    private static double SimplexDiameter(double[][] simplex)
    {
        int n = simplex[0].Length;
        double scale = 0.0;
        for (int k = 0; k < n; k++) scale = Math.Max(scale, Math.Abs(simplex[0][k]));
        double maxDist = 0.0;
        for (int i = 1; i < simplex.Length; i++)
            for (int k = 0; k < n; k++)
                maxDist = Math.Max(maxDist, Math.Abs(simplex[i][k] - simplex[0][k]));
        return maxDist / Math.Max(1.0, scale);
    }
}
