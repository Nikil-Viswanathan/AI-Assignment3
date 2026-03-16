'''Part2, UGV Navigation with static obstacles, we use A* Algorithm here'''
import heapq
import random
import time
import math



GRID_SIZE = 70          # 70x70 km grid
CELL_SIZE = 1           # each cell = 1 km

OBSTACLE_DENSITY = {
    "LOW": 0.10,
    "MEDIUM": 0.25,
    "HIGH": 0.40,
}

#Can move in 8 directions, including diagonals
MOVES = [
    (0, 1),   (0, -1),  (1, 0),   (-1, 0),  
    (1, 1),   (1, -1),  (-1, 1),  (-1, -1), 
]

DIAGONAL_COST = math.sqrt(2)
CARDINAL_COST = 1.0

#This geneates a random grid
def generate_grid(size, density, start, goal, seed=42):
    random.seed(seed)
    grid = [[0] * size for _ in range(size)]
    for r in range(size):
        for c in range(size):
            if (r, c) != start and (r, c) != goal:
                if random.random() < density:
                    grid[r][c] = 1  # 1 = obstacle
    return grid

#Heurstic
def heuristic(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return CARDINAL_COST * max(dx, dy) + (DIAGONAL_COST - CARDINAL_COST) * min(dx, dy)


def astar(grid, start, goal):
    size = len(grid)
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))

    g_cost = {start: 0}
    came_from = {start: None}
    nodes_expanded = 0

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            #Reconstruct path
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path, nodes_expanded

        nodes_expanded += 1

        r, c = current
        for dr, dc in MOVES:
            nr, nc = r + dr, c + dc
            if 0 <= nr < size and 0 <= nc < size and grid[nr][nc] == 0:
                move_cost = DIAGONAL_COST if (dr != 0 and dc != 0) else CARDINAL_COST
                new_g = cost + move_cost
                neighbor = (nr, nc)
                if new_g < g_cost.get(neighbor, float('inf')):
                    g_cost[neighbor] = new_g
                    came_from[neighbor] = current
                    f = new_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, new_g, neighbor))

    return None, nodes_expanded#No path found


def visualize_grid(grid, path, start, goal, max_display=30):
    size = min(len(grid), max_display)
    path_set = set(path) if path else set()

    symbols = {
        'obstacle': '█',
        'path':     '·',
        'start':    'S',
        'goal':     'G',
        'free':     ' ',
    }

    print(f"\n  Grid visualization (top-left {size}x{size} corner):")
    print("  +" + "─" * size + "+")
    for r in range(size):
        row = "  |"
        for c in range(size):
            if (r, c) == start:
                row += symbols['start']
            elif (r, c) == goal:
                row += symbols['goal']
            elif grid[r][c] == 1:
                row += symbols['obstacle']
            elif (r, c) in path_set:
                row += symbols['path']
            else:
                row += symbols['free']
        row += "|"
        print(row)
    print("  +" + "─" * size + "+")
    print(f"  Legend: S=Start  G=Goal  █=Obstacle  ·=Path\n")


def run_ugv_static(density_label, start=(0, 0), goal=(69, 69), seed=42):
    density = OBSTACLE_DENSITY[density_label]
    grid = generate_grid(GRID_SIZE, density, start, goal, seed=seed)

    print(f"\n  UGV Navigation — Static Obstacles ({density_label} density: {int(density*100)}%)")
    print(f"  Grid: {GRID_SIZE}x{GRID_SIZE} km  |  Start: {start}  |  Goal: {goal}")

    t0 = time.time()
    path, nodes_expanded = astar(grid, start, goal)
    elapsed_ms = (time.time() - t0) * 1000

    if path:
        path_length = 0.0
        for i in range(1, len(path)):
            dr = abs(path[i][0] - path[i-1][0])
            dc = abs(path[i][1] - path[i-1][1])
            path_length += DIAGONAL_COST if (dr and dc) else CARDINAL_COST

        print(f"\n  Path found!")
        print(f"\n  Measures of Effectiveness (MoE):")
        print(f"    Path length      : {path_length:.2f} km")
        print(f"    Path nodes       : {len(path)}")
        print(f"    Nodes expanded   : {nodes_expanded}")
        print(f"    Time taken       : {elapsed_ms:.2f} ms")
        print(f"    Obstacle density : {density_label} ({int(density*100)}%)")

        visualize_grid(grid, path, start, goal)
    else:
        print(f"\n  No path found (grid too densely blocked).")
        print(f"\n  Measures of Effectiveness (MoE):")
        print(f"    Path length      : N/A")
        print(f"    Nodes expanded   : {nodes_expanded}")
        print(f"    Time taken       : {elapsed_ms:.2f} ms")
        print(f"    Obstacle density : {density_label} ({int(density*100)}%)")
        visualize_grid(grid, [], start, goal)

    return path, nodes_expanded, elapsed_ms




START = (0, 0)
GOAL  = (69, 69)

results = {}
for level in ["LOW", "MEDIUM", "HIGH"]:
    path, nodes, ms = run_ugv_static(level, START, GOAL)
    results[level] = {
        "success": path is not None,
        "length": round(sum(
            DIAGONAL_COST if (abs(path[i][0]-path[i-1][0]) and abs(path[i][1]-path[i-1][1])) else CARDINAL_COST
            for i in range(1, len(path))
        ), 2) if path else None,
        "nodes_expanded": nodes,
        "time_ms": round(ms, 2),
    }

   
print("\n  SUMMARY")
print(f"  {'Level':<10} {'Success':<10} {'Length (km)':<15} {'Nodes':<12} {'Time (ms)'}")
for level, r in results.items():
    length_str = f"{r['length']:.2f}" if r['length'] else "N/A"
    print(f"  {level:<10} {'Yes' if r['success'] else 'No':<10} {length_str:<15} {r['nodes_expanded']:<12} {r['time_ms']}")
print()