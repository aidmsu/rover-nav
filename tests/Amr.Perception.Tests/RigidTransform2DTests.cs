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

    [Fact]
    public void AfterComposesInApplicationOrder()
    {
        var first = new RigidTransform2D(0.3, 1, 0);
        var second = new RigidTransform2D(-0.1, 0, 2);
        var point = new Point2(-1, 2);

        var expected = second.Apply(first.Apply(point));
        var actual = second.After(first).Apply(point);

        Assert.Equal(expected.X, actual.X, 1e-12);
        Assert.Equal(expected.Y, actual.Y, 1e-12);
    }
}
