"""
Distributed Collision Avoidance and Local Negotiation Module.
Handles peer-to-peer conflict detection, right-of-way arbitration, and safe deceleration.
"""

from typing import Tuple, Optional, Dict, Any
from backend.models import RobotState, TaskPriority, RobotStatus

class CollisionNegotiator:
    """
    Decentralized collision avoidance engine running on each AMR's edge controller.
    Evaluates peer heartbeats and determines right-of-way or evasive actions.
    """

    @staticmethod
    def check_proximity_conflict(
        amr_a: RobotState,
        amr_b: RobotState,
        critical_distance: float = 1.0,
        warning_distance: float = 2.0
    ) -> Dict[str, Any]:
        """
        Calculates spatial distance and checks for imminent or potential collision.
        """
        dx = amr_a.grid_x - amr_b.grid_x
        dy = amr_a.grid_y - amr_b.grid_y
        dist = (dx**2 + dy**2) ** 0.5

        conflict_type = "NONE"
        if dist <= critical_distance:
            conflict_type = "CRITICAL"
        elif dist <= warning_distance:
            conflict_type = "WARNING"

        # Check intended next step overlap
        next_step_a = amr_a.path[0] if amr_a.path else (amr_a.grid_x, amr_a.grid_y)
        next_step_b = amr_b.path[0] if amr_b.path else (amr_b.grid_x, amr_b.grid_y)

        # Head-on conflict: A wants B's cell, B wants A's cell
        is_head_on = (
            next_step_a == (amr_b.grid_x, amr_b.grid_y) and
            next_step_b == (amr_a.grid_x, amr_a.grid_y)
        )

        # Same target cell conflict: Both want to enter the same intersection cell
        is_same_target = (
            next_step_a == next_step_b and
            next_step_a != (amr_a.grid_x, amr_a.grid_y)
        )

        return {
            "distance": dist,
            "conflict_type": conflict_type,
            "is_head_on": is_head_on,
            "is_same_target": is_same_target,
            "target_cell": next_step_a if is_same_target else None
        }

    @staticmethod
    def arbitrate_right_of_way(
        amr_a: RobotState,
        amr_b: RobotState,
        task_a_priority: TaskPriority = TaskPriority.NORMAL,
        task_b_priority: TaskPriority = TaskPriority.NORMAL
    ) -> Tuple[str, str, str]:
        """
        Determines which robot has right-of-way based on distributed priority heuristics:
        1. Critical Battery State (low battery robot must reach charger)
        2. Task Priority (Urgent/VIP tasks preempt normal tasks)
        3. Proximity to Destination (quicker completion clears warehouse traffic faster)
        4. Tie-breaker: Lexicographical ID (deterministic decentralized consensus)

        Returns:
            Tuple: (winner_id, yielding_id, rationale)
        """
        # 1. Critical Battery Check
        if amr_a.battery < 20.0 and amr_b.battery >= 20.0:
            return (amr_a.id, amr_b.id, f"{amr_a.id} has critical battery ({amr_a.battery:.1f}%) and right-of-way to charger")
        if amr_b.battery < 20.0 and amr_a.battery >= 20.0:
            return (amr_b.id, amr_a.id, f"{amr_b.id} has critical battery ({amr_b.battery:.1f}%) and right-of-way to charger")

        # 2. Task Priority Check
        if task_a_priority.value > task_b_priority.value:
            return (amr_a.id, amr_b.id, f"{amr_a.id} holds higher task priority ({task_a_priority.name})")
        if task_b_priority.value > task_a_priority.value:
            return (amr_b.id, amr_a.id, f"{amr_b.id} holds higher task priority ({task_b_priority.name})")

        # 3. Path Length / Distance to Goal
        len_a = len(amr_a.path)
        len_b = len(amr_b.path)
        if len_a > 0 and len_b > 0 and len_a != len_b:
            if len_a < len_b:
                return (amr_a.id, amr_b.id, f"{amr_a.id} is closer to goal ({len_a} steps vs {len_b} steps)")
            else:
                return (amr_b.id, amr_a.id, f"{amr_b.id} is closer to goal ({len_b} steps vs {len_a} steps)")

        # 4. Tie-breaker (Lexicographical ID)
        if amr_a.id < amr_b.id:
            return (amr_a.id, amr_b.id, f"Arbitration tie-breaker favored {amr_a.id} (ID rule)")
        else:
            return (amr_b.id, amr_a.id, f"Arbitration tie-breaker favored {amr_b.id} (ID rule)")
