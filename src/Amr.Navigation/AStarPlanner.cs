namespace Amr.Navigation;

/// <summary>
/// 8-connected A* planner on an <see cref="OccupancyGrid"/>. Straight moves cost 1, diagonal moves cost sqrt(2),
/// and diagonal moves may not cut the corner of an occupied cell.
/// </summary>
public sealed class AStarPlanner
{
    private static readonly double Sqrt2 = Math.Sqrt(2.0);

    private static readonly (int Dx, int Dy, double Cost)[] Moves =
    [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, Sqrt2), (1, -1, Sqrt2), (-1, 1, Sqrt2), (-1, -1, Sqrt2),
    ];

    private readonly OccupancyGrid _grid;

    public AStarPlanner(OccupancyGrid grid)
    {
        ArgumentNullException.ThrowIfNull(grid);
        _grid = grid;
    }

    /// <summary>Octile distance, the exact path length on an empty 8-connected grid.</summary>
    public static double Heuristic(int x, int y, GridPoint goal)
    {
        var dx = Math.Abs(x - goal.X);
        var dy = Math.Abs(y - goal.Y);
        return Math.Max(dx, dy) + (Sqrt2 - 1.0) * Math.Min(dx, dy);
    }

    /// <summary>Returns the shortest path between two free cells, or null when the goal is unreachable.</summary>
    public PlannedPath? FindPath(GridPoint start, GridPoint goal)
    {
        if (!_grid.IsFree(start) || !_grid.IsFree(goal)) return null;

        var width = _grid.Width;
        var cellCount = width * _grid.Height;
        var costSoFar = new double[cellCount];
        Array.Fill(costSoFar, double.PositiveInfinity);
        var cameFrom = new int[cellCount];
        var closed = new bool[cellCount];

        var startIndex = start.Y * width + start.X;
        var goalIndex = goal.Y * width + goal.X;
        costSoFar[startIndex] = 0;
        cameFrom[startIndex] = -1;

        var open = new PriorityQueue<int, double>();
        open.Enqueue(startIndex, Heuristic(start.X, start.Y, goal));
        var expanded = 0;

        while (open.TryDequeue(out var current, out _))
        {
            if (closed[current]) continue;
            if (current == goalIndex) return Reconstruct(cameFrom, goalIndex, width, costSoFar[goalIndex], expanded);

            closed[current] = true;
            expanded++;
            int cx = current % width, cy = current / width;

            foreach (var (dx, dy, stepCost) in Moves)
            {
                int nx = cx + dx, ny = cy + dy;
                if (!_grid.IsFree(nx, ny)) continue;
                if (dx != 0 && dy != 0 && (!_grid.IsFree(cx + dx, cy) || !_grid.IsFree(cx, cy + dy))) continue;

                var next = ny * width + nx;
                if (closed[next]) continue;

                var tentative = costSoFar[current] + stepCost;
                if (tentative >= costSoFar[next]) continue;

                costSoFar[next] = tentative;
                cameFrom[next] = current;
                open.Enqueue(next, tentative + Heuristic(nx, ny, goal));
            }
        }

        return null;
    }

    private static PlannedPath Reconstruct(int[] cameFrom, int goalIndex, int width, double cost, int expanded)
    {
        var cells = new List<GridPoint>();
        for (var index = goalIndex; index != -1; index = cameFrom[index])
        {
            cells.Add(new GridPoint(index % width, index / width));
        }

        cells.Reverse();
        return new PlannedPath(cells, cost, expanded);
    }
}
