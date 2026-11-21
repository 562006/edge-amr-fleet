"""
A* Pathfinding and Dijkstra's Algorithm Implementation.
Calculates optimal collision-free routes for AMRs in 2D warehouse grid environments.
"""

import heapq
from typing import List, Tuple, Optional, Set, Dict

def heuristic(a: Tuple[int, int], b: Tuple[int, int], metric: str = "manhattan") -> float:
    """Calculates admissible heuristic distance between two points."""
    if metric == "euclidean":
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
    # Default: Manhattan distance (ideal for 4-connected grid warehouse)
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    is_walkable_fn,
    get_neighbors_fn,
    blocked_cells: Optional[Set[Tuple[int, int]]] = None,
    heuristic_weight: float = 1.05
) -> Optional[List[Tuple[int, int]]]:
    """
    Standard A* algorithm on 2D grid.
    
    Args:
        start: (x, y) start coordinate
        goal: (x, y) target coordinate
        is_walkable_fn: Callable(x, y) -> bool
        get_neighbors_fn: Callable(x, y) -> List[(x, y)]
        blocked_cells: Optional set of temporarily blocked or avoided coordinates
        heuristic_weight: Epsilon weight for slightly greedy tie-breaking (speeds up search)

    Returns:
        List of coordinates from start to goal (inclusive), or None if no path exists.
    """
    if start == goal:
        return [start]

    if not is_walkable_fn(goal[0], goal[1]):
        return None

    blocked = blocked_cells or set()

    # Priority queue: (f_score, g_score, (x, y))
    open_set = []
    initial_h = heuristic(start, goal)
    heapq.heappush(open_set, (initial_h * heuristic_weight, 0, start))

    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}

    closed_set: Set[Tuple[int, int]] = set()

    while open_set:
        _, current_g, current = heapq.heappop(open_set)

        if current in closed_set:
            continue
        closed_set.add(current)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor in get_neighbors_fn(current[0], current[1]):
            if neighbor in closed_set or neighbor in blocked:
                continue

            tentative_g = current_g + 1.0  # Uniform edge cost of 1.0 per grid step

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                h = heuristic(neighbor, goal)
                f = tentative_g + (h * heuristic_weight)
                heapq.heappush(open_set, (f, tentative_g, neighbor))

    return None  # No path reachable

def dijkstra_search(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    is_walkable_fn,
    get_neighbors_fn,
    blocked_cells: Optional[Set[Tuple[int, int]]] = None
) -> Optional[List[Tuple[int, int]]]:
    """
    Dijkstra's Algorithm (uniform-cost search) for comparison and benchmark.
    Guarantees shortest path with h(n) = 0.
    """
    return a_star_search(
        start=start,
        goal=goal,
        is_walkable_fn=is_walkable_fn,
        get_neighbors_fn=get_neighbors_fn,
        blocked_cells=blocked_cells,
        heuristic_weight=0.0
    )
