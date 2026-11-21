"""
Unit Tests for Deadlock Detection and Wait-For Graph (WFG) Analysis.
"""

import pytest
from backend.algorithms.deadlock_detector import DeadlockDetector
from backend.models import RobotState, RobotStatus

def test_no_deadlock():
    detector = DeadlockDetector()
    # AMR-01 waits for AMR-02, but AMR-02 is free
    detector.update_waiting_relationship("AMR-01", "AMR-02")
    detector.update_waiting_relationship("AMR-02", None)
    
    cycles = detector.detect_cycles()
    assert len(cycles) == 0

def test_two_robot_head_on_deadlock():
    detector = DeadlockDetector()
    # AMR-01 waits for AMR-02, AMR-02 waits for AMR-01 (head-on loop)
    detector.update_waiting_relationship("AMR-01", "AMR-02")
    detector.update_waiting_relationship("AMR-02", "AMR-01")
    
    cycles = detector.detect_cycles()
    assert len(cycles) >= 1
    cycle = cycles[0]
    assert "AMR-01" in cycle and "AMR-02" in cycle

def test_three_robot_junction_deadlock():
    detector = DeadlockDetector()
    # Circular 3-robot deadlock: R1 -> R2 -> R3 -> R1
    detector.update_waiting_relationship("AMR-01", "AMR-02")
    detector.update_waiting_relationship("AMR-02", "AMR-03")
    detector.update_waiting_relationship("AMR-03", "AMR-01")
    
    cycles = detector.detect_cycles()
    assert len(cycles) >= 1
    assert len(cycles[0]) == 3

def test_victim_selection():
    detector = DeadlockDetector()
    r1 = RobotState(id="AMR-01", name="Atlas", color="#fff", x=0, y=0, grid_x=0, grid_y=0, battery=90.0)
    r2 = RobotState(id="AMR-02", name="Bolt", color="#fff", x=1, y=0, grid_x=1, grid_y=0, battery=40.0, current_task_id="T1")
    
    cycle = ["AMR-01", "AMR-02"]
    robots = {"AMR-01": r1, "AMR-02": r2}
    
    victim = detector.select_resolution_victim(cycle, robots)
    # AMR-01 has no active task and higher battery (more maneuverable), so it should be chosen to yield
    assert victim == "AMR-01"
