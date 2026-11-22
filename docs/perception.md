# Perception

## Scan matching

`IcpScanMatcher` aligns a new laser scan with a reference scan using point-to-point ICP. Each iteration:

1. applies the current estimate to every scan point;
2. pairs each moved point with its closest reference point;
3. solves the closed-form least-squares rotation and translation for those pairs and composes it with the estimate.

Iteration stops when the incremental step falls below `Tolerance` or after `MaxIterations`. The result carries the
final transform, the number of iterations, the RMS distance of the last pairing and whether it converged.

## k-d tree

`KdTree` is a static 2D tree stored implicitly in one array, split at the median and alternating between X and Y.
`FindNearest` returns the index of the closest input point and its squared distance.

## Test scans

`ScanSimulator.SampleRoom(count, seed)` samples points on an irregular room outline with a little range noise. The
outline has no rotational symmetry, so ICP has a single correct answer for small offsets.
