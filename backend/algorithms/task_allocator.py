"""
Distributed Task Allocation Engine using Contract Net Protocol (CNP).
Implements multi-agent auction bidding with multi-criteria Edge-AI cost functions and preemption.
"""

from typing import List, Dict, Optional, Tuple
from config import (
    WEIGHT_DISTANCE, WEIGHT_BATTERY, WEIGHT_WORKLOAD,
    WEIGHT_URGENCY, BATTERY_LOW_THRESHOLD
)
from backend.models import Task, RobotState, AuctionBid, TaskPriority, TaskStatus, RobotStatus

class TaskAllocator:
    """
    Decentralized / Distributed Contract Net Protocol (CNP) Task Allocator.
    """

    @staticmethod
    def calculate_edge_bid(
        robot: RobotState,
        task: Task,
        current_queue_len: int = 0
    ) -> AuctionBid:
        """
        Calculates an AMR's Edge-AI bid for an announced warehouse task.
        Evaluates proximity, battery margin, existing workload, and task priority.
        """
        # Constraint Check: Low Battery
        if robot.battery < BATTERY_LOW_THRESHOLD:
            return AuctionBid(
                task_id=task.id,
                robot_id=robot.id,
                bid_cost=float('inf'),
                estimated_steps=999,
                battery_level=robot.battery,
                can_execute=False,
                explanation=f"Battery too low ({robot.battery:.1f}% < {BATTERY_LOW_THRESHOLD}%)"
            )

        # Constraint Check: Robot is currently charging or emergency stopped
        if robot.status in [RobotStatus.CHARGING, RobotStatus.EMERGENCY_STOP]:
            return AuctionBid(
                task_id=task.id,
                robot_id=robot.id,
                bid_cost=float('inf'),
                estimated_steps=999,
                battery_level=robot.battery,
                can_execute=False,
                explanation=f"Robot is {robot.status.value}"
            )

        # Distance from current position to pickup point
        dist_to_pickup = abs(robot.grid_x - task.pickup_x) + abs(robot.grid_y - task.pickup_y)
        # Transit distance from pickup to dropoff point
        dist_pickup_to_dropoff = abs(task.pickup_x - task.dropoff_x) + abs(task.pickup_y - task.dropoff_y)
        total_estimated_steps = dist_to_pickup + dist_pickup_to_dropoff

        # Workload factor
        workload = (1 if robot.current_task_id else 0) + current_queue_len

        # Edge-AI Multi-Criteria Utility Cost Formula
        cost = (
            WEIGHT_DISTANCE * dist_to_pickup +
            WEIGHT_BATTERY * (100.0 - robot.battery) * 0.15 +
            WEIGHT_WORKLOAD * (workload * 12.0) -
            WEIGHT_URGENCY * (task.priority.value * 5.0)
        )

        explanation = (
            f"Dist:{dist_to_pickup} steps, Batt:{robot.battery:.0f}%, "
            f"Workload:{workload}, Score:{cost:.2f}"
        )

        return AuctionBid(
            task_id=task.id,
            robot_id=robot.id,
            bid_cost=max(0.1, cost),
            estimated_steps=total_estimated_steps,
            battery_level=robot.battery,
            can_execute=True,
            explanation=explanation
        )

    @classmethod
    def conduct_auction(
        cls,
        task: Task,
        robots: List[RobotState]
    ) -> Tuple[Optional[str], List[AuctionBid]]:
        """
        Conducts a decentralized Contract Net Protocol auction among all available AMRs.
        Returns:
            Tuple of (winning_robot_id, list_of_all_bids)
        """
        bids: List[AuctionBid] = []

        for r in robots:
            bid = cls.calculate_edge_bid(r, task)
            bids.append(bid)

        # Filter valid executable bids
        valid_bids = [b for b in bids if b.can_execute and b.bid_cost < float('inf')]

        if not valid_bids:
            return (None, bids)

        # Lowest cost bid wins the contract
        valid_bids.sort(key=lambda b: b.bid_cost)
        winner = valid_bids[0].robot_id

        return (winner, bids)
