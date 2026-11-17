using Amr.Navigation;

namespace Amr.Navigation.Tests;

public class CollisionCheckerTests
{
    [Theory]
    [InlineData(1000)]
    [InlineData(4000)]
    [InlineData(10000)]
    public void ReferencePathThroughAisleIsClear(int obstacleCount)
    {
        var world = CollisionWorld.Create(obstacleCount, poseCount: 4000, seed: 11);

        Assert.Equal(obstacleCount, world.Checker.ObstacleCount);
        Assert.Equal(-1, world.Checker.FindFirstCollision(world.Path));
    }

    [Fact]
    public void ReportsFirstPoseThatTouchesAnObstacle()
    {
        var box = new ConvexPolygon([new Vec2(10, -0.2), new Vec2(11, -0.2), new Vec2(11, 0.2), new Vec2(10, 0.2)]);
        var checker = new CollisionChecker([box], footprintLength: 0.8, footprintWidth: 0.5);
        var path = Enumerable.Range(0, 200).Select(i => new Pose2D(i * 0.1, 0, 0)).ToList();

        Assert.Equal(0.8, checker.FootprintLength);
        Assert.Equal(0.5, checker.FootprintWidth);
        Assert.Equal(96, checker.FindFirstCollision(path));
    }

    [Fact]
    public void RotatedFootprintIsCheckedExactly()
    {
        var box = new ConvexPolygon([new Vec2(0.5, 0.5), new Vec2(1.5, 0.5), new Vec2(1.5, 1.5), new Vec2(0.5, 1.5)]);
        var checker = new CollisionChecker([box], footprintLength: 2.0, footprintWidth: 0.2);

        Assert.False(checker.Collides(new Pose2D(0, 0, -Math.PI / 4)));
        Assert.True(checker.Collides(new Pose2D(0, 0, Math.PI / 4)));
    }
}
