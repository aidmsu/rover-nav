using Amr.Navigation;

namespace Amr.Navigation.Tests;

public class OccupancyGridTests
{
    [Fact]
    public void NewGridIsFree()
    {
        var grid = new OccupancyGrid(20, 10);

        Assert.Equal(20, grid.Width);
        Assert.Equal(10, grid.Height);
        for (var y = 0; y < grid.Height; y++)
        {
            for (var x = 0; x < grid.Width; x++)
            {
                Assert.True(grid.IsFree(x, y));
            }
        }
    }

    [Theory]
    [InlineData(0, 10)]
    [InlineData(10, 0)]
    [InlineData(-4, 4)]
    public void RejectsNonPositiveDimensions(int width, int height)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new OccupancyGrid(width, height));
    }

    [Fact]
    public void CellsOutsideTheGridAreNotFree()
    {
        var grid = new OccupancyGrid(8, 8);

        Assert.False(grid.IsFree(-1, 0));
        Assert.False(grid.IsFree(0, -1));
        Assert.False(grid.IsFree(8, 0));
        Assert.False(grid.IsFree(0, 8));
        Assert.Throws<ArgumentOutOfRangeException>(() => grid.SetOccupied(8, 8));
    }

    [Fact]
    public void LineOfSightIsBlockedByAWall()
    {
        var grid = new OccupancyGrid(16, 16);
        grid.FillRectangle(8, 0, 8, 10);

        Assert.False(grid.HasLineOfSight(new GridPoint(2, 2), new GridPoint(14, 4)));
        Assert.True(grid.HasLineOfSight(new GridPoint(2, 13), new GridPoint(14, 15)));
        Assert.True(grid.HasLineOfSight(new GridPoint(3, 3), new GridPoint(3, 3)));
    }

    [Fact]
    public void WarehouseKeepsDockingCornersClear()
    {
        var grid = OccupancyGrid.CreateWarehouse(128, seed: 7);

        Assert.True(grid.IsFree(0, 0));
        Assert.True(grid.IsFree(127, 0));
        Assert.True(grid.IsFree(0, 127));
        Assert.True(grid.IsFree(127, 127));
    }

    [Fact]
    public void CountFreeExcludesFilledRectangles()
    {
        var grid = new OccupancyGrid(10, 10);
        grid.FillRectangle(2, 2, 4, 3);
        grid.FillRectangle(8, 8, 12, 12);

        Assert.Equal(100 - 6 - 4, grid.CountFree());
    }

    [Fact]
    public void DiagonalLineOfSightPassesThroughAGap()
    {
        var grid = new OccupancyGrid(12, 12);
        grid.FillRectangle(0, 6, 4, 6);
        grid.FillRectangle(7, 6, 11, 6);

        Assert.True(grid.HasLineOfSight(new GridPoint(1, 1), new GridPoint(10, 10)));
        Assert.False(grid.HasLineOfSight(new GridPoint(0, 0), new GridPoint(2, 11)));
    }
}
