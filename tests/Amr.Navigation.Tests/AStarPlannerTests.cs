using Amr.Navigation;

namespace Amr.Navigation.Tests;

public class AStarPlannerTests
{
    [Theory]
    [InlineData(128)]
    [InlineData(256)]
    [InlineData(512)]
    public void FindsShortestPathAcrossWarehouse(int size)
    {
        var grid = OccupancyGrid.CreateWarehouse(size, seed: 42);
        var planner = new AStarPlanner(grid);
        var last = size - 1;
        (GridPoint Start, GridPoint Goal)[] queries =
        [
            (new GridPoint(0, 0), new GridPoint(last, last)),
            (new GridPoint(last, 0), new GridPoint(0, last)),
        ];

        foreach (var (start, goal) in queries)
        {
            var path = planner.FindPath(start, goal);

            Assert.NotNull(path);
            Assert.Equal(start, path.Cells[0]);
            Assert.Equal(goal, path.Cells[^1]);
            Assert.Equal(Reference.ShortestDistance(grid, start, goal), path.Cost, 1e-6);
            AssertTraversable(grid, path);
        }
    }

    [Fact]
    public void StartEqualsGoalGivesSingleCellPath()
    {
        var grid = new OccupancyGrid(16, 16);

        var path = new AStarPlanner(grid).FindPath(new GridPoint(5, 7), new GridPoint(5, 7));

        Assert.NotNull(path);
        Assert.Equal([new GridPoint(5, 7)], path.Cells);
        Assert.Equal(0, path.Cost);
    }

    [Fact]
    public void ReturnsNullWhenGoalIsWalledOff()
    {
        var grid = new OccupancyGrid(32, 32);
        grid.FillRectangle(20, 20, 28, 20);
        grid.FillRectangle(20, 28, 28, 28);
        grid.FillRectangle(20, 20, 20, 28);
        grid.FillRectangle(28, 20, 28, 28);

        var path = new AStarPlanner(grid).FindPath(new GridPoint(1, 1), new GridPoint(24, 24));

        Assert.Null(path);
    }

    private static void AssertTraversable(OccupancyGrid grid, PlannedPath path)
    {
        for (var i = 0; i < path.Cells.Count; i++)
        {
            var cell = path.Cells[i];
            Assert.True(grid.IsFree(cell));
            if (i == 0) continue;

            var previous = path.Cells[i - 1];
            int dx = cell.X - previous.X, dy = cell.Y - previous.Y;
            Assert.InRange(Math.Abs(dx), 0, 1);
            Assert.InRange(Math.Abs(dy), 0, 1);
            if (dx != 0 && dy != 0)
            {
                Assert.True(grid.IsFree(previous.X + dx, previous.Y) && grid.IsFree(previous.X, previous.Y + dy));
            }
        }
    }
}
