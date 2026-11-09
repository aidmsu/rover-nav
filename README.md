# rover-nav

Grid navigation and scan-matching core for an autonomous mobile robot (AMR), written in C# on .NET 10.

Used as a showcase repository for PerfGuard AI; CI timings are genuine.

## Projects

| Project | Contents |
| --- | --- |
| `src/Amr.Navigation` | `OccupancyGrid`, 8-connected `AStarPlanner`, footprint `CollisionChecker` (convex obstacles, separating-axis test) |
| `src/Amr.Perception` | 2D `KdTree` for nearest-neighbour queries, point-to-point `IcpScanMatcher`, synthetic `ScanSimulator` |
| `tests/Amr.Navigation.Tests` | xUnit tests for planning and collision checking |
| `tests/Amr.Perception.Tests` | xUnit tests for the k-d tree and scan matching |

## Build and test

```bash
dotnet build RoverNav.slnx -c Release
dotnet test RoverNav.slnx -c Release
```

The SDK version is pinned in `global.json`. Test inputs are generated from fixed seeds, so every run sees the same
maps and scans.
