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
}
