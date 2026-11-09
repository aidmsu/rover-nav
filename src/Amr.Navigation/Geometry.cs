namespace Amr.Navigation;

/// <summary>A point or vector in the map frame, in metres.</summary>
public readonly record struct Vec2(double X, double Y)
{
    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);

    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);

    public double Dot(Vec2 other) => X * other.X + Y * other.Y;
}

/// <summary>A robot pose: position in metres and heading in radians.</summary>
public readonly record struct Pose2D(double X, double Y, double Heading);

/// <summary>An axis-aligned bounding box.</summary>
public readonly record struct Aabb(double MinX, double MinY, double MaxX, double MaxY)
{
    public bool Overlaps(in Aabb other) =>
        MinX <= other.MaxX && other.MinX <= MaxX && MinY <= other.MaxY && other.MinY <= MaxY;

    public static Aabb Of(ReadOnlySpan<Vec2> points)
    {
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        foreach (var p in points)
        {
            minX = Math.Min(minX, p.X);
            minY = Math.Min(minY, p.Y);
            maxX = Math.Max(maxX, p.X);
            maxY = Math.Max(maxY, p.Y);
        }

        return new Aabb(minX, minY, maxX, maxY);
    }
}

/// <summary>A convex polygon with vertices in counter-clockwise order.</summary>
public sealed class ConvexPolygon
{
    public ConvexPolygon(IReadOnlyList<Vec2> vertices)
    {
        ArgumentNullException.ThrowIfNull(vertices);
        if (vertices.Count < 3) throw new ArgumentException("A polygon needs at least three vertices.", nameof(vertices));
        Vertices = [.. vertices];
        Bounds = Aabb.Of(Vertices);
    }

    public Vec2[] Vertices { get; }

    public Aabb Bounds { get; }

    /// <summary>Exact overlap test between two convex polygons using the separating axis theorem.</summary>
    public static bool Intersects(ReadOnlySpan<Vec2> a, ReadOnlySpan<Vec2> b) => !HasSeparatingAxis(a, b) && !HasSeparatingAxis(b, a);

    private static bool HasSeparatingAxis(ReadOnlySpan<Vec2> edges, ReadOnlySpan<Vec2> other)
    {
        for (var i = 0; i < edges.Length; i++)
        {
            var edge = edges[(i + 1) % edges.Length] - edges[i];
            var axis = new Vec2(-edge.Y, edge.X);
            Project(edges, axis, out var minA, out var maxA);
            Project(other, axis, out var minB, out var maxB);
            if (maxA < minB || maxB < minA) return true;
        }

        return false;
    }

    private static void Project(ReadOnlySpan<Vec2> polygon, Vec2 axis, out double min, out double max)
    {
        min = double.PositiveInfinity;
        max = double.NegativeInfinity;
        foreach (var vertex in polygon)
        {
            var projection = vertex.Dot(axis);
            min = Math.Min(min, projection);
            max = Math.Max(max, projection);
        }
    }
}
