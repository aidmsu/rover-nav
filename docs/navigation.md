# Navigation

## Occupancy grid

`OccupancyGrid` stores one flag per cell. Cell (0, 0) is the top-left corner; X grows to the right and Y grows
downwards. Anything outside the grid counts as occupied, so planners never need separate bounds checks.

`OccupancyGrid.CreateWarehouse(size, seed)` builds the maps used by the tests: random racks and pallets, a partition
wall with a single doorway, and clear docking areas in the four corners.

## Planner

`AStarPlanner` searches the 8-connected grid:

- straight moves cost 1, diagonal moves cost sqrt(2);
- a diagonal move is only allowed when both adjacent straight cells are free, so the robot never cuts the corner of
  a rack;
- the heuristic is the octile distance, which is admissible and consistent for this move model, so the first time
  the goal is taken from the open set its cost is optimal.

When several open nodes have the same estimated total cost, the planner expands the one closest to the goal first.
That does not change the path length, but on open floor it keeps the search on the straight line to the goal.

## Collision checking

`CollisionChecker` tests a rectangular footprint (length along the heading, width across it) against convex
obstacles using the separating axis theorem. `FindFirstCollision` walks a sampled path and returns the index of the
first pose that touches an obstacle.
