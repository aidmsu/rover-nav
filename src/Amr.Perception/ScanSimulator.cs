namespace Amr.Perception;

/// <summary>
/// Generates deterministic synthetic laser scans of an irregular room outline, for tests and offline tuning.
/// </summary>
public static class ScanSimulator
{
    /// <summary>Samples <paramref name="count"/> points on the room outline with Gaussian range noise.</summary>
    public static Point2[] SampleRoom(int count, int seed, double noise = 0.005)
    {
        var random = new Random(seed);
        var points = new Point2[count];
        for (var i = 0; i < count; i++)
        {
            var theta = random.NextDouble() * 2 * Math.PI;
            var radius = RoomRadius(theta) + noise * NextGaussian(random);
            points[i] = new Point2(radius * Math.Cos(theta), radius * Math.Sin(theta));
        }

        return points;
    }

    private static double RoomRadius(double theta) =>
        6.0 + 1.5 * Math.Sin(3 * theta) + 0.8 * Math.Cos(5 * theta + 0.7) + 0.4 * Math.Sin(7 * theta + 1.3);

    private static double NextGaussian(Random random)
    {
        var u1 = 1.0 - random.NextDouble();
        var u2 = random.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2);
    }
}
