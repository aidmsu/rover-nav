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

        Assert.Equal(96, checker.FindFirstCollision(path));
    }
}
