using Amr.Perception;

namespace Amr.Perception.Tests;

public class IcpScanMatcherTests
{
    [Theory]
    [InlineData(2000)]
    [InlineData(5000)]
    [InlineData(10000)]
    public void RecoversKnownTransform(int pointCount)
    {
        var reference = ScanSimulator.SampleRoom(pointCount, seed: 2024);
        var truth = new RigidTransform2D(Angle: 3.0 * Math.PI / 180, Tx: 0.25, Ty: -0.15);
        var noise = new Random(99);
        var scan = reference
            .Select(truth.Inverse().Apply)
            .Select(p => new Point2(p.X + (noise.NextDouble() - 0.5) * 0.004, p.Y + (noise.NextDouble() - 0.5) * 0.004))
            .ToArray();

        var result = new IcpScanMatcher(reference).Match(scan);

        Assert.True(result.Converged);
        Assert.Equal(truth.Angle, result.Transform.Angle, 1e-3);
        Assert.Equal(truth.Tx, result.Transform.Tx, 0.01);
        Assert.Equal(truth.Ty, result.Transform.Ty, 0.01);
        Assert.InRange(result.RootMeanSquareError, 0, 0.01);
    }

    [Fact]
    public void IdenticalScansMatchWithIdentity()
    {
        var reference = ScanSimulator.SampleRoom(500, seed: 5);

        var result = new IcpScanMatcher(reference).Match(reference);

        Assert.True(result.Converged);
        Assert.Equal(1, result.Iterations);
        Assert.Equal(0, result.RootMeanSquareError, 1e-12);
    }
}
