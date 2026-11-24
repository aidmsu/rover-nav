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
| `docs/` | Notes on the [navigation](docs/navigation.md) and [perception](docs/perception.md) code |

Contribution guidelines are in [CONTRIBUTING.md](CONTRIBUTING.md).

## Usage

```csharp
using Amr.Navigation;
using Amr.Perception;

// Plan a path across a warehouse map.
var grid = OccupancyGrid.CreateWarehouse(size: 256, seed: 42);
var path = new AStarPlanner(grid).FindPath(new GridPoint(0, 0), new GridPoint(255, 255));
Console.WriteLine($"{path?.Cells.Count} cells, length {path?.Cost:F1}");

// Check a sampled path for a 0.8 m x 0.5 m robot against convex obstacles.
var box = new ConvexPolygon([new Vec2(10, -0.2), new Vec2(11, -0.2), new Vec2(11, 0.2), new Vec2(10, 0.2)]);
var checker = new CollisionChecker([box], footprintLength: 0.8, footprintWidth: 0.5);
var poses = Enumerable.Range(0, 200).Select(i => new Pose2D(i * 0.1, 0, 0)).ToList();
Console.WriteLine($"first collision at pose {checker.FindFirstCollision(poses)}");

// Align a new laser scan with a reference scan.
var reference = ScanSimulator.SampleRoom(count: 5000, seed: 1);
var scan = reference.Select(new RigidTransform2D(0.05, 0.2, -0.1).Apply).ToArray();
var match = new IcpScanMatcher(reference).Match(scan);
Console.WriteLine($"rotation {match.Transform.Angle:F4} rad after {match.Iterations} iterations");
```

## Build and test

```bash
dotnet build RoverNav.slnx -c Release
dotnet test RoverNav.slnx -c Release
```

The SDK version is pinned in `global.json`. Test inputs are generated from fixed seeds, so every run sees the same
maps and scans.
