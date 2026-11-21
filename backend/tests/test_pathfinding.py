"""
Unit Tests for Pathfinding Algorithms (A* and Space-Time Cooperative A*).
"""

import pytest
from backend.warehouse.warehouse_map import WarehouseMap
from backend.algorithms.a_star import a_star_search, dijkstra_search
from backend.algorithms.space_time_a_star import SpaceTimeReservationTable, space_time_a_star

def test_a_star_basic():
    wmap = WarehouseMap()
    start = (1, 1)
    goal = (1, 5)
    
    path = a_star_search(
        start=start,
        goal=goal,
        is_walkable_fn=wmap.is_walkable,
        get_neighbors_fn=wmap.get_neighbors
    )
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    assert len(path) == 5

def test_a_star_obstacle_avoidance():
    wmap = WarehouseMap()
    start = (3, 3)
    goal = (6, 3)
    # Rack is at (4, 3) and (5, 3)
    path = a_star_search(
        start=start,
        goal=goal,
        is_walkable_fn=wmap.is_walkable,
        get_neighbors_fn=wmap.get_neighbors
    )
    assert path is not None
    # Ensure path goes around racks
    for step in path:
        assert step not in wmap.racks

def test_space_time_vertex_collision_avoidance():
    wmap = WarehouseMap()
    table = SpaceTimeReservationTable()
    
    # AMR-01 reserves cell (5, 1) at tick 3
    table.vertex_reservations[(5, 1, 3)] = "AMR-01"
    
    # AMR-02 plans path from (1, 1) to (8, 1) starting at t=0
    path = space_time_a_star(
        start=(1, 1),
        goal=(8, 1),
        start_time=0,
        robot_id="AMR-02",
        reservation_table=table,
        is_walkable_fn=wmap.is_walkable,
        get_neighbors_fn=wmap.get_neighbors
    )
    assert path is not None
    # Verify AMR-02 does not occupy (5, 1) at t=3
    for t_idx, step in enumerate(path):
        if step == (5, 1):
            assert t_idx != 3

def test_space_time_edge_swap_collision_avoidance():
    wmap = WarehouseMap()
    table = SpaceTimeReservationTable()
    
    # AMR-01 moves from (5, 1) to (4, 1) between t=2 and t=3
    table.edge_reservations[((5, 1), (4, 1), 3)] = "AMR-01"
    
    # AMR-02 plans path starting at (3, 1) towards (6, 1) starting at t=1
    path = space_time_a_star(
        start=(3, 1),
        goal=(6, 1),
        start_time=1,
        robot_id="AMR-02",
        reservation_table=table,
        is_walkable_fn=wmap.is_walkable,
        get_neighbors_fn=wmap.get_neighbors
    )
    assert path is not None
    # Verify AMR-02 does not do edge swap ((4, 1) -> (5, 1) at t=3)
    for t_idx in range(len(path) - 1):
        if path[t_idx] == (4, 1) and path[t_idx+1] == (5, 1):
            assert (t_idx + 1) != 3
