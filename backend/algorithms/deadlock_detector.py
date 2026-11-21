"""
Deadlock Detection and Resolution Engine using Wait-For Graph (WFG) Analysis.
Detects circular waiting dependencies among AMRs and initiates automated retreat/reroute maneuvers.
"""

from typing import Dict, List, Optional, Set, Tuple
from backend.models import RobotState, RobotStatus

class DeadlockDetector:
    """
    Constructs a dynamic Wait-For Graph (WFG) of AMRs and checks for directed cycles.
    """

    def __init__(self):
        # Wait-For Graph adjacency list: waiting_robot_id -> target_robot_id
        self.wait_for_graph: Dict[str, str] = {}
        self.wait_counters: Dict[str, int] = {}

    def update_waiting_relationship(
        self,
        waiting_robot_id: str,
        blocking_robot_id: Optional[str]
    ):
        """Updates or clears a wait dependency for an AMR."""
        if blocking_robot_id:
            self.wait_for_graph[waiting_robot_id] = blocking_robot_id
            self.wait_counters[waiting_robot_id] = self.wait_counters.get(waiting_robot_id, 0) + 1
        else:
            if waiting_robot_id in self.wait_for_graph:
                del self.wait_for_graph[waiting_robot_id]
            self.wait_counters[waiting_robot_id] = 0

    def clear_robot(self, robot_id: str):
        if robot_id in self.wait_for_graph:
            del self.wait_for_graph[robot_id]
        self.wait_counters[robot_id] = 0
        # Also remove any edges pointing to this robot
        self.wait_for_graph = {k: v for k, v in self.wait_for_graph.items() if v != robot_id}

    def detect_cycles(self) -> List[List[str]]:
        """
        Detects all directed cycles in the Wait-For Graph using DFS.
        Returns a list of cycles (each cycle is a list of robot_ids forming a loop).
        """
        visited: Set[str] = set()
        rec_stack: Set[str] = set()
        parent_map: Dict[str, str] = {}
        cycles: List[List[str]] = []

        nodes = list(self.wait_for_graph.keys())

        for node in nodes:
            if node not in visited:
                curr = node
                path_stack = []

                while curr and curr not in visited:
                    visited.add(curr)
                    rec_stack.add(curr)
                    path_stack.append(curr)

                    nxt = self.wait_for_graph.get(curr)
                    if nxt:
                        parent_map[nxt] = curr
                        if nxt in rec_stack:
                            # Cycle detected! Reconstruct cycle
                            cycle = [nxt]
                            p = curr
                            while p != nxt:
                                cycle.append(p)
                                p = parent_map.get(p)
                            cycle.reverse()
                            cycles.append(cycle)
                            break
                        curr = nxt
                    else:
                        break

                for item in path_stack:
                    rec_stack.discard(item)

        return cycles

    def select_resolution_victim(
        self,
        cycle: List[str],
        robots_dict: Dict[str, RobotState]
    ) -> str:
        """
        Chooses which robot in the deadlock cycle should yield/evacuate.
        Prefers robot with:
        1. Non-urgent task status
        2. Highest battery level (able to maneuver)
        3. Lexicographical ID fallback
        """
        best_candidate = cycle[0]
        lowest_priority_score = float('inf')

        for r_id in cycle:
            r = robots_dict.get(r_id)
            if not r:
                continue

            # Score: lower means better candidate to yield
            # (e.g. higher battery means more reserve to take a detour)
            priority_score = 0
            if r.current_task_id:
                priority_score += 10
            # Prefer robots with more battery to do the retreat
            priority_score -= (r.battery / 100.0)

            if priority_score < lowest_priority_score:
                lowest_priority_score = priority_score
                best_candidate = r_id

        return best_candidate
