# AI-Assignment3

# Dijkstra’s Algorithm
This Python program implements Dijkstra’s Algorithm to compute the shortest road distances between Indian cities. The road network is represented as a weighted undirected graph, where cities are nodes and road distances are edge weights.
The code builds the graph from a list of city connections, then runs Dijkstra’s algorithm using a priority queue (heapq) to compute the minimum distance from a source city. It also reconstructs the actual path using a prev dictionary.
The program outputs:

The shortest distance from Delhi to all other cities
The shortest route between specific city pairs
The sequence of cities forming each path

# UGV Static
This program simulates UGV navigation on a 70×70 grid with static obstacles, using the A* pathfinding algorithm. Obstacles are randomly generated based on different density levels (Low, Medium, High).

The algorithm uses a heuristic distance function and a priority queue to efficiently search for the shortest path from the start cell to the goal cell while avoiding obstacles.

The program outputs:

Whether a valid path was found
The total path length
The number of nodes expanded during search
The execution time
A grid visualization showing start (S), goal (G), obstacles (█), and the path (·)
It also prints a summary comparing performance across obstacle densities.

# UGV Dynamic
This program simulates a UGV navigating a grid with dynamically discovered obstacles using the D* Lite algorithm, which supports efficient replanning when the environment changes.
The environment contains hidden obstacles that the UGV cannot initially see. As the vehicle moves, it detects obstacles within its sensor range, updates the map, and recomputes the path only where necessary.
The code implements the D* Lite planner, which maintains g and rhs values for nodes and uses a priority queue to update affected nodes when new obstacles are discovered.
The program outputs:

Whether the UGV successfully reached the goal
The total distance traveled
The number of replanning events
The number of nodes updated
The execution time
A grid visualization showing known obstacles, hidden obstacles, and the path taken
The simulation runs multiple times with different random obstacle layouts to demonstrate how the algorithm adapts to dynamic environments.
