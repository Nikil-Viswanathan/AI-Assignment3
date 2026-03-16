# AI-Assignment3

- Dijkstra’s Algorithm
  This program implements Dijkstra’s Algorithm in Python to compute the shortest road distances between Indian cities. The cities are represented as nodes in a weighted graph, and the road distances between them are the edge weights.

The program builds the graph from a list of city connections, then calculates the minimum distance from a source city to all other cities using a priority queue (heapq). It also reconstructs and prints the actual shortest path between cities.

The code demonstrates:
Finding shortest paths from Delhi to all other cities
Finding shortest paths between specific city pairs (e.g., Mumbai -> Kolkata).

- UGV Static
  This program simulates UGV (Unmanned Ground Vehicle) navigation in a grid with static obstacles using the A* search algorithm. A random grid environment is generated with different obstacle densities (Low, Medium, High), and the UGV must find the shortest path from the start position to the goal.

The A* algorithm uses a heuristic distance function to efficiently search for the optimal path while avoiding obstacles. The simulation reports performance metrics such as path length, nodes expanded, and execution time, and displays a grid visualization showing the start, goal, obstacles, and computed path.

- UGV Dynamic
  This program simulates an Unmanned Ground Vehicle (UGV) navigating a grid environment using the D* Lite path planning algorithm. The grid contains randomly generated hidden obstacles that the UGV discovers only when they enter its sensor range.

The algorithm continuously replans the path dynamically whenever new obstacles are detected, allowing the UGV to adapt and still attempt to reach the goal. The simulation tracks metrics such as distance traveled, number of replanning events, nodes updated, and execution time, and also prints a grid visualization of the path and obstacles.
