namespace Amr.Perception;

/// <summary>
/// A static 2D k-d tree for nearest-neighbour queries. The tree is stored implicitly: each subrange of the internal
/// point array is split at its median, alternating between the X and Y axes.
/// </summary>
public sealed class KdTree
{
    private static readonly Comparer<Point2> ByX = Comparer<Point2>.Create((a, b) => a.X.CompareTo(b.X));
    private static readonly Comparer<Point2> ByY = Comparer<Point2>.Create((a, b) => a.Y.CompareTo(b.Y));

    private readonly Point2[] _points;
    private readonly int[] _indices;

    public KdTree(IReadOnlyList<Point2> points)
    {
        ArgumentNullException.ThrowIfNull(points);
        if (points.Count == 0) throw new ArgumentException("A k-d tree needs at least one point.", nameof(points));

        _points = [.. points];
        _indices = [.. Enumerable.Range(0, _points.Length)];
        Build(0, _points.Length, 0);
    }

    public int Count => _points.Length;

    /// <summary>Returns the index (in the original input order) of the point closest to <paramref name="query"/>.</summary>
    public int FindNearest(Point2 query, out double distanceSquared)
    {
        var best = -1;
        distanceSquared = double.PositiveInfinity;
        Search(0, _points.Length, 0, query, ref best, ref distanceSquared);
        return _indices[best];
    }

    private void Build(int lo, int hi, int depth)
    {
        if (hi - lo <= 1) return;

        Array.Sort(_points, _indices, lo, hi - lo, depth % 2 == 0 ? ByX : ByY);
        var mid = lo + (hi - lo) / 2;
        Build(lo, mid, depth + 1);
        Build(mid + 1, hi, depth + 1);
    }

    private void Search(int lo, int hi, int depth, Point2 query, ref int best, ref double bestDistance)
    {
        if (lo >= hi) return;

        var mid = lo + (hi - lo) / 2;
        var pivot = _points[mid];
        var distance = pivot.DistanceSquaredTo(query);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            best = mid;
        }

        var delta = depth % 2 == 0 ? query.X - pivot.X : query.Y - pivot.Y;
        if (delta < 0)
        {
            Search(lo, mid, depth + 1, query, ref best, ref bestDistance);
            if (delta * delta < bestDistance) Search(mid + 1, hi, depth + 1, query, ref best, ref bestDistance);
        }
        else
        {
            Search(mid + 1, hi, depth + 1, query, ref best, ref bestDistance);
            if (delta * delta < bestDistance) Search(lo, mid, depth + 1, query, ref best, ref bestDistance);
        }
    }
}
