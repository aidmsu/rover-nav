using Amr.Navigation;

namespace Amr.Navigation.Tests;

/// <summary>
/// A long aisle with a gently weaving reference path and randomly placed convex obstacles that keep clear of it.
/// </summary>
internal sealed record CollisionWorld(CollisionChecker Checker, IReadOnlyList<Pose2D> Path)
{
    public const double FootprintLength = 0.8;
    public const double FootprintWidth = 0.5;

    private const double Amplitude = 2.0;
    private const double Wavelength = 8.0;

    public static double PathY(double x) => Amplitude * Math.Sin(x / Wavelength);

    public static CollisionWorld Create(int obstacleCount, int poseCount, int seed)
    {
        var length = poseCount * 0.05;
        var path = new List<Pose2D>(poseCount);
        for (var i = 0; i < poseCount; i++)
        {
            var x = i * 0.05;
            var heading = Math.Atan(Amplitude / Wavelength * Math.Cos(x / Wavelength));
            path.Add(new Pose2D(x, PathY(x), heading));
        }

        var random = new Random(seed);
        var obstacles = new List<ConvexPolygon>(obstacleCount);
        while (obstacles.Count < obstacleCount)
        {
            var radius = 0.2 + 0.4 * random.NextDouble();
            var cx = random.NextDouble() * length;
            var cy = (random.NextDouble() - 0.5) * 30.0;
            if (Math.Abs(cy - PathY(cx)) < 1.2 * (radius + FootprintLength)) continue;

            obstacles.Add(RandomConvex(random, cx, cy, radius));
        }

        return new CollisionWorld(new CollisionChecker(obstacles, FootprintLength, FootprintWidth), path);
    }

    private static ConvexPolygon RandomConvex(Random random, double cx, double cy, double radius)
    {
        var count = random.Next(6, 11);
        var angles = new double[count];
        for (var i = 0; i < count; i++) angles[i] = random.NextDouble() * 2 * Math.PI;
        Array.Sort(angles);

        var vertices = new Vec2[count];
        for (var i = 0; i < count; i++)
        {
            vertices[i] = new Vec2(cx + radius * Math.Cos(angles[i]), cy + radius * Math.Sin(angles[i]));
        }

        return new ConvexPolygon(vertices);
    }
}
