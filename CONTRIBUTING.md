# Contributing

- Target .NET 10; the exact SDK is pinned in `global.json`.
- Keep test inputs deterministic: generate maps and scans from fixed seeds.
- Every change to the planner, collision checker or scan matcher needs a test that checks the result against a
  simple reference (Dijkstra for path length, brute force for nearest neighbours, a known transform for ICP).
- Run `dotnet test RoverNav.slnx -c Release` before opening a pull request. CI runs the same tests on linux-x64 and
  linux-arm64.
