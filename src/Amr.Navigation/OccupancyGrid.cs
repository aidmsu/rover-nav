namespace Amr.Navigation;

/// <summary>
/// A 2D occupancy grid. Cells are either free or occupied; everything outside the grid is treated as occupied.
/// </summary>
/// <remarks>
/// Cell (0, 0) is the top-left corner of the map; X grows to the right and Y grows downwards. Cells are stored
/// row by row, so the cell (x, y) lives at index <c>y * Width + x</c>.
/// </remarks>
public sealed class OccupancyGrid
{
    private readonly bool[] _occupied;

    public OccupancyGrid(int width, int height)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(width);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(height);

        Width = width;
        Height = height;
        _occupied = new bool[width * height];
    }

    /// <summary>Number of columns.</summary>
    public int Width { get; }

    /// <summary>Number of rows.</summary>
    public int Height { get; }

    /// <summary>Returns true when the cell lies inside the grid.</summary>
    public bool InBounds(int x, int y) => (uint)x < (uint)Width && (uint)y < (uint)Height;

    /// <summary>Returns true when the cell lies inside the grid and is not occupied.</summary>
    public bool IsFree(int x, int y) => InBounds(x, y) && !_occupied[y * Width + x];

    public bool IsFree(GridPoint cell) => IsFree(cell.X, cell.Y);

    public void SetOccupied(int x, int y, bool occupied = true)
    {
        if (!InBounds(x, y))
        {
            throw new ArgumentOutOfRangeException(nameof(x), $"Cell ({x}, {y}) is outside the {Width}x{Height} grid.");
        }

        _occupied[y * Width + x] = occupied;
    }

    /// <summary>Returns the number of free cells.</summary>
    public int CountFree()
    {
        var free = 0;
        foreach (var occupied in _occupied)
        {
            if (!occupied) free++;
        }

        return free;
    }

    /// <summary>Marks every cell of the inclusive rectangle as occupied, clipped to the grid.</summary>
    public void FillRectangle(int x0, int y0, int x1, int y1)
    {
        for (var y = Math.Max(0, Math.Min(y0, y1)); y <= Math.Min(Height - 1, Math.Max(y0, y1)); y++)
        {
            for (var x = Math.Max(0, Math.Min(x0, x1)); x <= Math.Min(Width - 1, Math.Max(x0, x1)); x++)
            {
                _occupied[y * Width + x] = true;
            }
        }
    }

    /// <summary>Returns true when the straight segment between the two cells crosses only free cells (Bresenham).</summary>
    public bool HasLineOfSight(GridPoint from, GridPoint to)
    {
        int x = from.X, y = from.Y;
        int dx = Math.Abs(to.X - x), dy = -Math.Abs(to.Y - y);
        int sx = x < to.X ? 1 : -1, sy = y < to.Y ? 1 : -1;
        var error = dx + dy;
        while (true)
        {
            if (!IsFree(x, y)) return false;
            if (x == to.X && y == to.Y) return true;
            var doubled = 2 * error;
            if (doubled >= dy)
            {
                error += dy;
                x += sx;
            }

            if (doubled <= dx)
            {
                error += dx;
                y += sy;
            }
        }
    }

    /// <summary>
    /// Builds a warehouse-like map: random rectangular racks and pallets, and a partition wall that splits the hall
    /// with a single doorway at the top. The four corners are kept clear so that docking positions are reachable.
    /// </summary>
    public static OccupancyGrid CreateWarehouse(int size, int seed)
    {
        var grid = new OccupancyGrid(size, size);
        var random = new Random(seed);
        var obstacleCount = size * size / 40;
        for (var i = 0; i < obstacleCount; i++)
        {
            var x = random.Next(size);
            var y = random.Next(size);
            grid.FillRectangle(x, y, x + random.Next(1, 7), y + random.Next(1, 4));
        }

        // Partition wall between the two halves of the hall, with a single doorway at the top.
        var wall = size / 2;
        grid.FillRectangle(wall, size / 8, wall + 1, size - 1);
        for (var y = 0; y < size / 8; y++)
        {
            grid.SetOccupied(wall, y, false);
            grid.SetOccupied(wall + 1, y, false);
        }

        var margin = Math.Max(2, size / 32);
        foreach (var (cx, cy) in new[] { (0, 0), (size - 1, 0), (0, size - 1), (size - 1, size - 1) })
        {
            for (var y = Math.Max(0, cy - margin); y <= Math.Min(size - 1, cy + margin); y++)
            {
                for (var x = Math.Max(0, cx - margin); x <= Math.Min(size - 1, cx + margin); x++)
                {
                    grid.SetOccupied(x, y, false);
                }
            }
        }

        return grid;
    }
}
