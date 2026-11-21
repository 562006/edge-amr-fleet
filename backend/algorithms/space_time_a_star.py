"""
Space-Time Cooperative A* (Time-Expanded A*) Algorithm.
Searches in 3D (x, y, time_tick) space to guarantee collision-free multi-robot trajectories.
Prevents both Vertex Collisions and Edge Swap Collisions.
"""

import heapq
from typing import List, Tuple, Optional, Set, Dict

class SpaceTimeReservationTable:
    """
    Tracks cell and edge reservations across discrete time ticks.
    Enables distributed or cooperative multi-agent path deconfliction.
    """
    def __init__(self):
        # Vertex reservations: (x, y, t) -> robot_id
        self.vertex_reservations: Dict[Tuple[int, int, int], str] = {}
        # Edge reservations: ((u_x, u_y), (v_x, v_y), t) -> robot_id
        self.edge_reservations: Dict[Tuple[Tuple[int, int], Tuple[int, int], int], str] = {}

    def is_vertex_reserved(self, x: int, y: int, t: int, exclude_robot_id: Optional[str] = None) -> bool:
        res = self.vertex_reservations.get((x, y, t))
        return res is not None and res != exclude_robot_id

    def is_edge_swap_conflict(self, u: Tuple[int, int], v: Tuple[int, int], t: int, exclude_robot_id: Optional[str] = None) -> bool:
        """Checks if another robot is moving in the reverse direction (v -> u) at the same time step."""
        res = self.edge_reservations.get((v, u, t))
        return res is not None and res != exclude_robot_id

    def reserve_path(self, robot_id: str, path: List[Tuple[int, int]], start_time: int = 0):
        """Reserves a time-expanded trajectory for an AMR."""
        current_time = start_time
        prev_pos = None

        for pos in path:
            self.vertex_reservations[(pos[0], pos[1], current_time)] = robot_id
            if prev_pos and prev_pos != pos:
                self.edge_reservations[(prev_pos, pos, current_time)] = robot_id
            prev_pos = pos
            current_time += 1

    def clear_robot_reservations(self, robot_id: str):
        """Releases all reservations for a specific robot."""
        self.vertex_reservations = {k: v for k, v in self.vertex_reservations.items() if v != robot_id}
        self.edge_reservations = {k: v for k, v in self.edge_reservations.items() if v != robot_id}


def space_time_a_star(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    start_time: int,
    robot_id: str,
    reservation_table: SpaceTimeReservationTable,
    is_walkable_fn,
    get_neighbors_fn,
    max_time_horizon: int = 60
) -> Optional[List[Tuple[int, int]]]:
    """
    Plans a collision-free route in Space-Time (x, y, t).
    Allows WAIT actions at current cell if moving forward would collide.
    """
    if start == goal:
        return [start]

    # Priority queue: (f_score, g_score, (x, y, t))
    open_set = []
    initial_h = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
    heapq.heappush(open_set, (initial_h, 0, (start[0], start[1], start_time)))

    came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
    g_score: Dict[Tuple[int, int, int], float] = {(start[0], start[1], start_time): 0.0}
    closed_set: Set[Tuple[int, int, int]] = set()

    while open_set:
        _, current_g, current_node = heapq.heappop(open_set)
        cx, cy, ct = current_node

        if current_node in closed_set:
            continue
        closed_set.add(current_node)

        # Goal condition: reached target location
        if (cx, cy) == goal:
            # Reconstruct trajectory
            path = [(cx, cy)]
            curr = current_node
            while curr in came_from:
                curr = came_from[curr]
                path.append((curr[0], curr[1]))
            path.reverse()
            return path

        if ct >= start_time + max_time_horizon:
            continue

        next_time = ct + 1

        # Possible candidate actions:
        # 1. Move to any adjacent walkable neighbor
        # 2. Wait in place at (cx, cy)
        candidates = get_neighbors_fn(cx, cy) + [(cx, cy)]

        for nx, ny in candidates:
            # Check vertex conflict at next_time
            if reservation_table.is_vertex_reserved(nx, ny, next_time, exclude_robot_id=robot_id):
                continue

            # Check edge swap conflict
            if (nx, ny) != (cx, cy):
                if reservation_table.is_edge_swap_conflict((cx, cy), (nx, ny), next_time, exclude_robot_id=robot_id):
                    continue

            # Step cost: 1.0 for move, 1.1 for waiting (prefer moving over stalling if equal)
            step_cost = 1.0 if (nx, ny) != (cx, cy) else 1.1
            tentative_g = current_g + step_cost
            next_state = (nx, ny, next_time)

            if next_state not in g_score or tentative_g < g_score[next_state]:
                g_score[next_state] = tentative_g
                came_from[next_state] = current_node
                h = abs(nx - goal[0]) + abs(ny - goal[1])
                f = tentative_g + h
                heapq.heappush(open_set, (f, tentative_g, next_state))

    return None
