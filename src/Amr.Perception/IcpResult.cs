namespace Amr.Perception;

/// <summary>Outcome of a scan match: the transform that maps the scan onto the reference, and fit statistics.</summary>
public sealed record IcpResult(RigidTransform2D Transform, int Iterations, double RootMeanSquareError, bool Converged);
