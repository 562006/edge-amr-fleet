"""
Unit Tests for Contract Net Protocol (CNP) Task Allocation Engine.
"""

import pytest
from backend.models import Task, RobotState, TaskPriority, TaskStatus
from backend.algorithms.task_allocator import TaskAllocator

def test_cnp_bid_calculation():
    robot = RobotState(id="AMR-01", name="Atlas", color="#fff", x=1, y=1, grid_x=1, grid_y=1, battery=90.0)
    task = Task(id="T1", description="Move Parts", pickup_x=3, pickup_y=1, dropoff_x=10, dropoff_y=1)
    
    bid = TaskAllocator.calculate_edge_bid(robot, task)
    assert bid.can_execute is True
    assert bid.bid_cost > 0
    assert bid.estimated_steps == (2 + 7)

def test_cnp_low_battery_refusal():
    # Robot with 15% battery (< 25% threshold) must refuse task
    robot = RobotState(id="AMR-01", name="Atlas", color="#fff", x=1, y=1, grid_x=1, grid_y=1, battery=15.0)
    task = Task(id="T1", description="Move Parts", pickup_x=3, pickup_y=1, dropoff_x=10, dropoff_y=1)
    
    bid = TaskAllocator.calculate_edge_bid(robot, task)
    assert bid.can_execute is False
    assert bid.bid_cost == float('inf')

def test_auction_winner_determination():
    # R1 is at (1, 1), R2 is at (15, 15). Task pickup is at (2, 1).
    r1 = RobotState(id="AMR-01", name="Atlas", color="#fff", x=1, y=1, grid_x=1, grid_y=1, battery=95.0)
    r2 = RobotState(id="AMR-02", name="Bolt", color="#fff", x=15, y=15, grid_x=15, grid_y=15, battery=95.0)
    task = Task(id="T1", description="Pickup Pallet", pickup_x=2, pickup_y=1, dropoff_x=5, dropoff_y=1)
    
    winner, bids = TaskAllocator.conduct_auction(task, [r1, r2])
    # AMR-01 is much closer, so it should win the auction
    assert winner == "AMR-01"
