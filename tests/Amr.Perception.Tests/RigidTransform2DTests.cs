using Amr.Perception;

namespace Amr.Perception.Tests;

public class RigidTransform2DTests
{
    [Fact]
    public void InverseUndoesTheTransform()
    {
        var transform = new RigidTransform2D(0.7, 1.5, -2.0);
        var point = new Point2(3, 4);

        var roundTrip = transform.Inverse().Apply(transform.Apply(point));

        Assert.Equal(point.X, roundTrip.X, 1e-12);
        Assert.Equal(point.Y, roundTrip.Y, 1e-12);
    }
}
