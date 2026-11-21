"""
Unit Tests for FleetManager Simulation Engine.
"""

import pytest
from backend.warehouse.fleet_manager import FleetManager
from backend.models import TaskPriority

def test_fleet_initialization():
    manager = FleetManager()
    assert len(manager.robots) == 3
    assert "AMR-01" in manager.robots
    assert "AMR-02" in manager.robots
    assert "AMR-03" in manager.robots
    assert manager.metrics["total_robots"] == 3

def test_simulation_step_and_task_execution():
    manager = FleetManager()
    task = manager.add_task("Express delivery", 2, 2, 2, 4, TaskPriority.HIGH)
    assert task.id in manager.tasks

    # Step simulation 10 times
    for _ in range(10):
        manager.step()

    assert manager.current_tick == 10
    assert len(manager.event_logs) > 0

def test_load_all_scenarios():
    manager = FleetManager()
    for scn_id in range(1, 7):
        manager.load_scenario(scn_id)
        assert len(manager.event_logs) > 0
        manager.step()
