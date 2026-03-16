'''Part 1, Dijkstra's algorithm'''

import heapq
from collections import defaultdict


#Indian cities and their road distances to other cities (approx)
INDIA_ROADS = [
    ("Delhi", "Jaipur", 281),
    ("Delhi", "Agra", 233),
    ("Delhi", "Lucknow", 555),
    ("Jaipur", "Ahmedabad", 660),
    ("Agra", "Lucknow", 363),
    ("Lucknow", "Varanasi", 320),
    ("Lucknow", "Patna", 730),
    ("Varanasi", "Patna", 283),
    ("Patna", "Kolkata", 600),
    ("Ahmedabad", "Mumbai", 524),
    ("Mumbai", "Pune", 148),
    ("Mumbai", "Hyderabad", 711),
    ("Hyderabad", "Chennai", 627),
    ("Hyderabad", "Bengaluru", 575),
    ("Chennai", "Bengaluru", 346),
    ("Kolkata", "Hyderabad", 1495),
]


def build_graph(edges):
    graph = defaultdict(list)
    for u, v, w in edges:
        graph[u].append((v, w))
        graph[v].append((u, w))
    return graph


def dijkstra(graph, source):
    dist = {node: float('inf') for node in graph}
    dist[source] = 0
    prev = {node: None for node in graph}

    # Min-heap(cost, node)
    heap = [(0, source)]

    visited = set()

    while heap:
        cost, u = heapq.heappop(heap)

        if u in visited:
            continue
        visited.add(u)

        for v, weight in graph[u]:
            new_cost = cost + weight
            if new_cost < dist[v]:
                dist[v] = new_cost
                prev[v] = u
                heapq.heappush(heap, (new_cost, v))

    return dist, prev


def reconstruct_path(prev, source, target):
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = prev.get(node)
    path.reverse()
    if path[0] != source:
        return []  #Occurs if no path is found
    return path


def print_all_shortest_paths(source):
    graph = build_graph(INDIA_ROADS)

    #Add any city referenced in edges
    all_cities = set()
    for u, v, _ in INDIA_ROADS:
        all_cities.add(u)
        all_cities.add(v)
    for city in all_cities:
        if city not in graph:
            graph[city] = []

    dist, prev = dijkstra(graph, source)

    print(f"\n  Dijkstra's Algorithm — Shortest Road Distances from {source}")
    print(f"  {'Destination':<25} {'Distance (km)':>15}  Path")

    sorted_cities = sorted(
        [(d, c) for c, d in dist.items() if c != source and d < float('inf')]
    )

    for d, city in sorted_cities:
        path = reconstruct_path(prev, source, city)
        path_str = " -> ".join(path)
        print(f"  {city:<25} {d:>13} km  {path_str}")

    unreachable = [c for c, d in dist.items() if d == float('inf') and c != source]
    if unreachable:
        print(f"\n  Unreachable cities: {', '.join(unreachable)}")

    print()


def point_to_point(source, target):
    graph = build_graph(INDIA_ROADS)
    all_cities = set()
    for u, v, _ in INDIA_ROADS:
        all_cities.add(u)
        all_cities.add(v)
    for city in all_cities:
        if city not in graph:
            graph[city] = []

    dist, prev = dijkstra(graph, source)

    if dist[target] == float('inf'):
        print(f"\nNo path found from {source} to {target}.")
        return

    path = reconstruct_path(prev, source, target)
    print(f"\n  Shortest path: {' -> '.join(path)}")
    print(f"  Total distance: {dist[target]} km\n")


#This will print all the shortest paths from Delhi to other cities
print_all_shortest_paths("Delhi")

print("Point-to-point: Mumbai -> Kolkata")
#This will print the shortest path from a city to city
point_to_point("Mumbai", "Kolkata")

print("Point-to-point: Chennai -> Amritsar")
point_to_point("Chennai", "Delhi")