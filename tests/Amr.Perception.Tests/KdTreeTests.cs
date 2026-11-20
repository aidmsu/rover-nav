using Amr.Perception;

namespace Amr.Perception.Tests;

public class KdTreeTests
{
    [Theory]
    [InlineData(1000)]
    [InlineData(10000)]
    [InlineData(30000)]
    public void NearestNeighbourMatchesBruteForce(int count)
    {
        var random = new Random(123);
        var points = Enumerable.Range(0, count)
            .Select(_ => new Point2(random.NextDouble() * 50, random.NextDouble() * 50))
            .ToArray();
        var tree = new KdTree(points);

        for (var q = 0; q < 300; q++)
        {
            var query = new Point2(random.NextDouble() * 60 - 5, random.NextDouble() * 60 - 5);
            var nearest = tree.FindNearest(query, out var distanceSquared);

            var expected = points.Min(p => p.DistanceSquaredTo(query));
            Assert.Equal(expected, distanceSquared);
            Assert.Equal(expected, points[nearest].DistanceSquaredTo(query));
        }
    }

    [Fact]
    public void DuplicatePointsAreFoundAtZeroDistance()
    {
        var points = Enumerable.Range(0, 64).Select(i => new Point2(i % 4, i % 4)).ToArray();
        var tree = new KdTree(points);

        var nearest = tree.FindNearest(new Point2(2, 2), out var distanceSquared);

        Assert.Equal(0, distanceSquared);
        Assert.Equal(new Point2(2, 2), points[nearest]);
    }

    [Fact]
    public void SinglePointTreeAlwaysReturnsThatPoint()
    {
        var tree = new KdTree([new Point2(3, 4)]);

        Assert.Equal(0, tree.FindNearest(new Point2(0, 0), out var distanceSquared));
        Assert.Equal(25, distanceSquared);
    }

    [Fact]
    public void RejectsEmptyInput()
    {
        Assert.Throws<ArgumentException>(() => new KdTree([]));
    }
}
