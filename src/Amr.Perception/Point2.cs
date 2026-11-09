namespace Amr.Perception;

/// <summary>A 2D point in the sensor or map frame, in metres.</summary>
public readonly record struct Point2(double X, double Y)
{
    public double DistanceSquaredTo(Point2 other)
    {
        var dx = X - other.X;
        var dy = Y - other.Y;
        return dx * dx + dy * dy;
    }
}
