#  Lunarlockout

## puzzle description
Lunar Lockout has the following formal description. Read the description carefully.
 - The puzzle is played on a square board that is a grid board with N squares in the x-dimension and N
squares in the y-dimension. The dimension N is always odd.
 - Each state in the game contains the x and y coordinates for each robot as well as the x and y coordinates
for each rover (or xanadu).
 - From each state, each robot and each rover can move North, South, East, or West, but only if there
is a second robot or rover that lies in that direction. When a robot or rover moves, it must move all
the way to the second piece until the pieces collide. For example, if a robot located at position (4;0)
moves South toward a rover located at (4;3), the robot will end at the location (4;2). No two pieces
(robots or rovers) can move simultaneously or diagonally and pieces cannot pass through walls or
one another.
 - The escape hatch is always located in the center of the board (i.e. at the grid location ((N-1)/2, (N-1)/2).
 -  Once a rover arrives at the escape hatch, it exits the board through the escape hatch. It then disappears
from subsequent play.
 - Each movement is of equal cost.
- The goal is achieved when all rovers have exited the board via the escape hatch.

## Heuristic search solution for lunar lockout puzzle
f = g(node)+w*h(node)

