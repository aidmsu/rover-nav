namespace Amr.Navigation;

/// <summary>
/// Checks a rectangular robot footprint against static convex obstacles. Obstacles whose bounding boxes do not
/// overlap the footprint's bounding box are rejected before the exact separating-axis test.
/// </summary>
public sealed class CollisionChecker
{
    private readonly ConvexPolygon[] _obstacles;
    private readonly Aabb[] _bounds;
    private readonly double _halfLength;
    private readonly double _halfWidth;

    public CollisionChecker(IEnumerable<ConvexPolygon> obstacles, double footprintLength, double footprintWidth)
    {
        ArgumentNullException.ThrowIfNull(obstacles);
        _obstacles = [.. obstacles];
        _bounds = [.. _obstacles.Select(o => o.Bounds)];
        _halfLength = footprintLength / 2;
        _halfWidth = footprintWidth / 2;
    }

    public int ObstacleCount => _obstacles.Length;

    /// <summary>Footprint length along the robot heading, in metres.</summary>
    public double FootprintLength => 2 * _halfLength;

    /// <summary>Footprint width across the robot heading, in metres.</summary>
    public double FootprintWidth => 2 * _halfWidth;

    /// <summary>Returns true when the footprint at the given pose overlaps any obstacle.</summary>
    public bool Collides(Pose2D pose)
    {
        Span<Vec2> footprint = stackalloc Vec2[4];
        GetFootprint(pose, footprint);
        var box = Aabb.Of(footprint);

        for (var i = 0; i < _obstacles.Length; i++)
        {
            if (!_bounds[i].Overlaps(box)) continue;
            if (ConvexPolygon.Intersects(footprint, _obstacles[i].Vertices)) return true;
        }

        return false;
    }

    /// <summary>Returns the index of the first colliding pose of the path, or -1 when the whole path is clear.</summary>
    public int FindFirstCollision(IReadOnlyList<Pose2D> path)
    {
        ArgumentNullException.ThrowIfNull(path);
        for (var i = 0; i < path.Count; i++)
        {
            if (Collides(path[i])) return i;
        }

        return -1;
    }

    /// <summary>Writes the four footprint corners at the pose in counter-clockwise order.</summary>
    public void GetFootprint(Pose2D pose, Span<Vec2> corners)
    {
        var (sin, cos) = Math.SinCos(pose.Heading);
        ReadOnlySpan<(double Forward, double Left)> local =
        [
            (_halfLength, _halfWidth), (-_halfLength, _halfWidth), (-_halfLength, -_halfWidth), (_halfLength, -_halfWidth),
        ];
        for (var i = 0; i < 4; i++)
        {
            var (forward, left) = local[i];
            corners[i] = new Vec2(pose.X + forward * cos - left * sin, pose.Y + forward * sin + left * cos);
        }
    }
}
