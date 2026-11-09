using Amr.Navigation;

namespace Amr.Navigation.Tests;

/// <summary>Plain Dijkstra over the same move model as the planner, used as the ground truth for path length.</summary>
internal static class Reference
{
    private static readonly double Sqrt2 = Math.Sqrt(2.0);

    public static double ShortestDistance(OccupancyGrid grid, GridPoint start, GridPoint goal)
    {
        var width = grid.Width;
        var distance = new double[width * grid.Height];
        Array.Fill(distance, double.PositiveInfinity);
        var queue = new PriorityQueue<int, double>();
        distance[start.Y * width + start.X] = 0;
        queue.Enqueue(start.Y * width + start.X, 0);

        while (queue.TryDequeue(out var current, out var d))
        {
            if (d > distance[current]) continue;
            int cx = current % width, cy = current / width;
            if (cx == goal.X && cy == goal.Y) return d;

            for (var dy = -1; dy <= 1; dy++)
            {
                for (var dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dy == 0) continue;
                    if (!grid.IsFree(cx + dx, cy + dy)) continue;
                    if (dx != 0 && dy != 0 && (!grid.IsFree(cx + dx, cy) || !grid.IsFree(cx, cy + dy))) continue;

                    var next = (cy + dy) * width + cx + dx;
                    var candidate = d + (dx != 0 && dy != 0 ? Sqrt2 : 1.0);
                    if (candidate < distance[next])
                    {
                        distance[next] = candidate;
                        queue.Enqueue(next, candidate);
                    }
                }
            }
        }

        return double.PositiveInfinity;
    }
}
