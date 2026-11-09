namespace Amr.Perception;

/// <summary>A planar rigid transform: rotation by <see cref="Angle"/> radians followed by a translation.</summary>
public readonly record struct RigidTransform2D(double Angle, double Tx, double Ty)
{
    public static RigidTransform2D Identity => default;

    public Point2 Apply(Point2 p)
    {
        var (sin, cos) = Math.SinCos(Angle);
        return new Point2(cos * p.X - sin * p.Y + Tx, sin * p.X + cos * p.Y + Ty);
    }

    /// <summary>Returns the transform that applies <paramref name="first"/> and then this transform.</summary>
    public RigidTransform2D After(RigidTransform2D first)
    {
        var (sin, cos) = Math.SinCos(Angle);
        return new RigidTransform2D(
            first.Angle + Angle,
            cos * first.Tx - sin * first.Ty + Tx,
            sin * first.Tx + cos * first.Ty + Ty);
    }

    public RigidTransform2D Inverse()
    {
        var (sin, cos) = Math.SinCos(-Angle);
        return new RigidTransform2D(-Angle, -(cos * Tx - sin * Ty), -(sin * Tx + cos * Ty));
    }
}
