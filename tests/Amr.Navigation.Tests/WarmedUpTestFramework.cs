using System.Reflection;
using System.Runtime.CompilerServices;
using Amr.Navigation;
using Xunit.Abstractions;
using Xunit.Sdk;

[assembly: TestFramework("Amr.Navigation.Tests.WarmedUpTestFramework", "Amr.Navigation.Tests")]

namespace Amr.Navigation.Tests;

/// <summary>
/// Default xUnit framework that first compiles the library and runs a small planning and collision workload, so that
/// one-off JIT costs are not attributed to whichever test happens to run first.
/// </summary>
public sealed class WarmedUpTestFramework : XunitTestFramework
{
    public WarmedUpTestFramework(IMessageSink messageSink)
        : base(messageSink)
    {
        PrepareMethods(typeof(AStarPlanner).Assembly);
        PrepareMethods(typeof(WarmedUpTestFramework).Assembly);

        for (var i = 0; i < 5; i++)
        {
            var grid = OccupancyGrid.CreateWarehouse(48, seed: i);
            var path = new AStarPlanner(grid).FindPath(new GridPoint(0, 0), new GridPoint(47, 47));
            _ = Reference.ShortestDistance(grid, new GridPoint(0, 0), new GridPoint(47, 47));
            _ = path?.Cells.Count;

            var world = CollisionWorld.Create(obstacleCount: 200, poseCount: 200, seed: i);
            _ = world.Checker.FindFirstCollision(world.Path);
        }

        Assert.True(true);
        Assert.Equal(1, 1);
        Assert.Equal(1.0, 1.0, 1e-9);
        Assert.NotNull(new object());
        Assert.Null((object?)null);
        Assert.InRange(1, 0, 2);
    }

    private static void PrepareMethods(Assembly assembly)
    {
        const BindingFlags All = BindingFlags.DeclaredOnly | BindingFlags.Public | BindingFlags.NonPublic |
                                 BindingFlags.Instance | BindingFlags.Static;
        foreach (var type in assembly.GetTypes())
        {
            if (type.ContainsGenericParameters) continue;
            foreach (var method in type.GetMethods(All).Cast<MethodBase>().Concat(type.GetConstructors(All)))
            {
                if (method.IsAbstract || method.ContainsGenericParameters) continue;
                try
                {
                    RuntimeHelpers.PrepareMethod(method.MethodHandle);
                }
                catch (Exception)
                {
                    // Some methods cannot be prepared ahead of time; they are compiled on first use.
                }
            }
        }
    }
}
