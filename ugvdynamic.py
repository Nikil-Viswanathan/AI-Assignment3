'''UGV Dynamic version, we use D* lite algorithm to plan as that is easier'''

import heapq
import random
import time
import math
from collections import defaultdict


GRID_SIZE    = 70
SENSOR_RANGE = 5#UGV sees obstacles within 5 km
DIAGONAL_COST = math.sqrt(2)
CARDINAL_COST = 1.0
INF = float('inf')

MOVES = [
    (0, 1), (0, -1), (1, 0), (-1, 0),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
]

OBSTACLE_DENSITY = 0.20

# ─────────────────────────────────────────────
# Grid helpers
# ─────────────────────────────────────────────

def move_cost(a, b):
    dr, dc = abs(a[0]-b[0]), abs(a[1]-b[1])
    return DIAGONAL_COST if (dr and dc) else CARDINAL_COST


def neighbors(pos, size):
    r, c = pos
    for dr, dc in MOVES:
        nr, nc = r+dr, c+dc
        if 0 <= nr < size and 0 <= nc < size:
            yield (nr, nc)


def heuristic(a, b):
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    return CARDINAL_COST * max(dx, dy) + (DIAGONAL_COST - CARDINAL_COST) * min(dx, dy)


class DStarLite:
    """
    D* Lite planner for dynamic obstacle replanning.
    Plans backwards from goal -> start.
    When an obstacle is discovered, UpdateVertex is called on affected
    nodes and the priority queue is repaired cheaply.
    """

    def __init__(self, grid_size, start, goal):
        self.size   = grid_size
        self.start  = start
        self.goal   = goal
        self.k_m    = 0  

        self.g   = defaultdict(lambda: INF)
        self.rhs = defaultdict(lambda: INF)
        self.open_heap = []
        self.open_set  = {} 

        self.known_obstacles = set()

        self.rhs[goal] = 0
        k = self._calculate_key(goal)
        self._push(goal, k)

        self.nodes_updated = 0


    def _calculate_key(self, s):
        g_rhs = min(self.g[s], self.rhs[s])
        return (
            g_rhs + heuristic(self.start, s) + self.k_m,
            g_rhs
        )

    def _push(self, s, key):
        self.open_set[s] = key
        heapq.heappush(self.open_heap, (key, s))

    def _pop(self):
        while self.open_heap:
            key, s = heapq.heappop(self.open_heap)
            if s in self.open_set and self.open_set[s] == key:
                del self.open_set[s]
                return key, s
        return None, None

    def _top_key(self):
        while self.open_heap:
            key, s = self.open_heap[0]
            if s in self.open_set and self.open_set[s] == key:
                return key
            heapq.heappop(self.open_heap)
        return (INF, INF)


    def _cost(self, u, v):
        #Edge cost — INF if v is a known obstacle.
        if v in self.known_obstacles or u in self.known_obstacles:
            return INF
        return move_cost(u, v)

    def _update_vertex(self, u):
        self.nodes_updated += 1
        if u != self.goal:
            self.rhs[u] = min(
                self._cost(u, s) + self.g[s]
                for s in neighbors(u, self.size)
            )
        if u in self.open_set:
            del self.open_set[u]
        if self.g[u] != self.rhs[u]:
            self._push(u, self._calculate_key(u))

    def compute_shortest_path(self):
        while (self._top_key() < self._calculate_key(self.start)
               or self.rhs[self.start] != self.g[self.start]):
            k_old, u = self._pop()
            if u is None:
                break
            k_new = self._calculate_key(u)
            if k_old < k_new:
                self._push(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in neighbors(u, self.size):
                    self._update_vertex(s)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for s in neighbors(u, self.size):
                    self._update_vertex(s)

    def notify_obstacles(self, new_obstacles):
        #Called when the UGV sensor detects new obstacles.
        changed = []
        for obs in new_obstacles:
            if obs not in self.known_obstacles:
                self.known_obstacles.add(obs)
                changed.append(obs)

        if not changed:
            return False

        #Update k_m for heuristic consistency after start shift
        for obs in changed:
            self._update_vertex(obs)
            for nb in neighbors(obs, self.size):
                self._update_vertex(nb)
        return True

    def next_step(self, current):
        #Return the best next cell to move to from current.
        if self.g[current] == INF:
            return None  # trapped
        best, best_cost = None, INF
        for nb in neighbors(current, self.size):
            c = self._cost(current, nb) + self.g[nb]
            if c < best_cost:
                best_cost = c
                best = nb
        return best

    def update_start(self, new_start):
        self.k_m += heuristic(self.start, new_start)
        self.start = new_start



def run_ugv_dynamic(start=(0, 0), goal=(69, 69), seed=99):
    #Simulate the UGV navigating with dynamic obstacle discovery.
    random.seed(seed)

    #Hidden obstacle map (unknown to UGV at start)
    hidden_obstacles = set()
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if (r, c) not in (start, goal):
                if random.random() < OBSTACLE_DENSITY:
                    hidden_obstacles.add((r, c))

    print(f"\n  UGV Navigation — Dynamic Obstacles (D* Lite)")
    print(f"  Grid: {GRID_SIZE}x{GRID_SIZE} km  |  Sensor range: {SENSOR_RANGE} km")
    print(f"  Start: {start}  ->  Goal: {goal}")
    print(f"  Hidden obstacles: {len(hidden_obstacles)} cells ({OBSTACLE_DENSITY*100:.0f}% density)")

    planner = DStarLite(GRID_SIZE, start, goal)
    planner.compute_shortest_path()

    current      = start
    path_taken   = [current]
    total_dist   = 0.0
    replans      = 0
    max_steps    = GRID_SIZE * GRID_SIZE 
    steps        = 0

    t0 = time.time()

    while current != goal and steps < max_steps:
        steps += 1

        #Senses, discovers new objects
        newly_seen = set()
        sr, sc = current
        for obs in hidden_obstacles:
            dr, dc = abs(obs[0]-sr), abs(obs[1]-sc)
            if math.sqrt(dr*dr + dc*dc) <= SENSOR_RANGE:
                if obs not in planner.known_obstacles:
                    newly_seen.add(obs)

        #Replans if obstacles found
        if newly_seen:
            changed = planner.notify_obstacles(newly_seen)
            if changed:
                planner.update_start(current)
                planner.compute_shortest_path()
                replans += 1

        #Next step
        nxt = planner.next_step(current)
        if nxt is None or nxt in planner.known_obstacles:
            print(f"\n   UGV is trapped at {current}. No navigable path.")
            elapsed_ms = (time.time() - t0) * 1000
            _print_moe(False, total_dist, replans, planner.nodes_updated,
                       elapsed_ms, path_taken)
            return path_taken, False

        total_dist += move_cost(current, nxt)
        current = nxt
        path_taken.append(current)

    elapsed_ms = (time.time() - t0) * 1000

    if current == goal:
        print(f"\n   UGV reached the goal!")
        _print_moe(True, total_dist, replans, planner.nodes_updated,
                   elapsed_ms, path_taken)
        _visualize_dynamic(hidden_obstacles, planner.known_obstacles,
                           path_taken, start, goal)
    else:
        print(f"\n   UGV did not reach goal within step limit.")
        _print_moe(False, total_dist, replans, planner.nodes_updated,
                   elapsed_ms, path_taken)

    return path_taken, current == goal


def _print_moe(success, dist, replans, nodes_updated, elapsed_ms, path):
    print(f"\n  Measures of Effectiveness (MoE):")
    print(f"    Status           : {'Success ' if success else 'Failed '}")
    print(f"    Distance traveled: {dist:.2f} km")
    print(f"    Steps taken      : {len(path)-1}")
    print(f"    Replanning events: {replans}")
    print(f"    Nodes updated    : {nodes_updated}")
    print(f"    Time taken       : {elapsed_ms:.2f} ms")


def _visualize_dynamic(hidden_obs, known_obs, path, start, goal, max_display=30):
    size = min(GRID_SIZE, max_display)
    path_set = set(path)

    print(f"\n  Grid visualization (top-left {size}x{size} corner):")
    print("  +" + "─" * size + "+")
    for r in range(size):
        row = "  |"
        for c in range(size):
            p = (r, c)
            if p == start:          row += 'S'
            elif p == goal:         row += 'G'
            elif p in known_obs:    row += '█'
            elif p in hidden_obs:   row += '▒'  #undiscovered obstacle
            elif p in path_set:     row += '·'
            else:                   row += ' '
        row += "|"
        print(row)
    print("  +" + "─" * size + "+")
    print("  Legend: S=Start  G=Goal  █=Known obstacle  ▒=Hidden obstacle  ·=Path\n")


path, success = run_ugv_dynamic(start=(0, 0), goal=(69, 69), seed=99)

  
print("\n  Running again with different random layout (seed=7)...")
path2, success2 = run_ugv_dynamic(start=(0, 0), goal=(69, 69), seed=7)