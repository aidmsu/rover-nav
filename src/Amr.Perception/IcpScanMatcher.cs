namespace Amr.Perception;

/// <summary>
/// Point-to-point iterative closest point (ICP) matcher for 2D laser scans. Each iteration pairs every scan point
/// with its nearest reference point and solves the closed-form least-squares rigid alignment of those pairs.
/// </summary>
public sealed class IcpScanMatcher
{
    private readonly Point2[] _reference;
    private readonly KdTree _index;

    public IcpScanMatcher(IReadOnlyList<Point2> reference)
    {
        ArgumentNullException.ThrowIfNull(reference);
        if (reference.Count == 0) throw new ArgumentException("The reference scan has no points.", nameof(reference));
        _reference = [.. reference];
        _index = new KdTree(_reference);
    }

    /// <summary>Upper bound on the number of match-and-align iterations. The result reports whether it converged earlier.</summary>
    public int MaxIterations { get; init; } = 100;

    /// <summary>
    /// Iteration stops once the incremental rotation (radians) and translation (metres) both fall below this value.
    /// Point-to-point ICP only converges to the nearest local minimum, so the initial guess should be within a few
    /// degrees and a few tens of centimetres of the true transform.
    /// </summary>
    public double Tolerance { get; init; } = 1e-9;

    public IcpResult Match(IReadOnlyList<Point2> scan) => Match(scan, RigidTransform2D.Identity);

    public IcpResult Match(IReadOnlyList<Point2> scan, RigidTransform2D initialGuess)
    {
        ArgumentNullException.ThrowIfNull(scan);
        if (scan.Count == 0) throw new ArgumentException("The scan has no points.", nameof(scan));
        if (MaxIterations <= 0) throw new InvalidOperationException($"{nameof(MaxIterations)} must be positive.");
        if (!(Tolerance > 0)) throw new InvalidOperationException($"{nameof(Tolerance)} must be positive.");

        var moved = new Point2[scan.Count];
        var matched = new Point2[scan.Count];
        var estimate = initialGuess;
        var rmse = double.PositiveInfinity;

        for (var iteration = 1; iteration <= MaxIterations; iteration++)
        {
            var squaredError = 0.0;
            for (var i = 0; i < scan.Count; i++)
            {
                moved[i] = estimate.Apply(scan[i]);
                matched[i] = _reference[FindCorrespondence(moved[i], out var distanceSquared)];
                squaredError += distanceSquared;
            }

            rmse = Math.Sqrt(squaredError / scan.Count);
            var step = SolveAlignment(moved, matched);
            estimate = step.After(estimate);

            if (Math.Abs(step.Angle) < Tolerance && Math.Abs(step.Tx) < Tolerance && Math.Abs(step.Ty) < Tolerance)
            {
                return new IcpResult(estimate, iteration, rmse, Converged: true);
            }
        }

        return new IcpResult(estimate, MaxIterations, rmse, Converged: false);
    }

    /// <summary>
    /// Returns the index of the reference point closest to <paramref name="point"/>. Uses the k-d tree index
    /// (O(log n) per query) — a linear scan here makes every ICP iteration O(scan × reference).
    /// </summary>
    private int FindCorrespondence(Point2 point, out double distanceSquared) => _index.FindNearest(point, out distanceSquared);

    /// <summary>Closed-form least-squares rigid transform that maps <paramref name="source"/> onto <paramref name="target"/>.</summary>
    private static RigidTransform2D SolveAlignment(Point2[] source, Point2[] target)
    {
        double sx = 0, sy = 0, tx = 0, ty = 0;
        for (var i = 0; i < source.Length; i++)
        {
            sx += source[i].X;
            sy += source[i].Y;
            tx += target[i].X;
            ty += target[i].Y;
        }

        sx /= source.Length;
        sy /= source.Length;
        tx /= source.Length;
        ty /= source.Length;

        double sxx = 0, sxy = 0, syx = 0, syy = 0;
        for (var i = 0; i < source.Length; i++)
        {
            double ax = source[i].X - sx, ay = source[i].Y - sy;
            double bx = target[i].X - tx, by = target[i].Y - ty;
            sxx += ax * bx;
            sxy += ax * by;
            syx += ay * bx;
            syy += ay * by;
        }

        var angle = Math.Atan2(sxy - syx, sxx + syy);
        var (sin, cos) = Math.SinCos(angle);
        return new RigidTransform2D(angle, tx - (cos * sx - sin * sy), ty - (sin * sx + cos * sy));
    }
}
