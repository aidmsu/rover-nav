using System.Reflection;
using System.Runtime.CompilerServices;
using Amr.Perception;
using Xunit.Abstractions;
using Xunit.Sdk;

[assembly: TestFramework("Amr.Perception.Tests.WarmedUpTestFramework", "Amr.Perception.Tests")]

namespace Amr.Perception.Tests;

/// <summary>
/// Default xUnit framework that first compiles the library and runs a small matching workload, so that one-off JIT
/// costs are not attributed to whichever test happens to run first.
/// </summary>
public sealed class WarmedUpTestFramework : XunitTestFramework
{
    public WarmedUpTestFramework(IMessageSink messageSink)
        : base(messageSink)
    {
        PrepareMethods(typeof(KdTree).Assembly);
        PrepareMethods(typeof(WarmedUpTestFramework).Assembly);

        for (var i = 0; i < 5; i++)
        {
            var reference = ScanSimulator.SampleRoom(300, seed: i);
            var tree = new KdTree(reference);
            _ = tree.FindNearest(new Point2(1, 1), out _);

            var truth = new RigidTransform2D(0.05, 0.1, -0.1);
            var scan = reference.Select(truth.Inverse().Apply).ToArray();
            _ = new IcpScanMatcher(reference).Match(scan);
        }

        Assert.True(true);
        Assert.Equal(1, 1);
        Assert.Equal(1.0, 1.0, 1e-9);
        Assert.InRange(1.0, 0.0, 2.0);
        Assert.Throws<ArgumentException>(() => new KdTree([]));
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
