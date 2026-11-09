namespace Amr.Navigation;

/// <summary>A path on the grid from start to goal, its total length in cells and the number of expanded nodes.</summary>
public sealed record PlannedPath(IReadOnlyList<GridPoint> Cells, double Cost, int ExpandedNodes);
