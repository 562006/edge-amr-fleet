"""
Autonomous Mobile Robot (AMR) Edge Agent.
Simulates an independent robot controller equipped with onboard sensors,
Edge-AI decision models, local path planner, and peer-to-peer communicator.
"""

from typing import List, Tuple, Optional, Dict, Set, Any
import math
import time
from backend.models import (
    RobotState, RobotStatus, Task, TaskPriority, TaskStatus,
    HeartbeatMessage, EventLog
)
from backend.edge.edge_ai_decision import EdgeAIDecisionEngine
from backend.algorithms.a_star import a_star_search
from backend.algorithms.collision_avoidance import CollisionNegotiator
from config import (
    BATTERY_DRAIN_PER_STEP, BATTERY_DRAIN_IDLE,
    BATTERY_RECHARGE_RATE, SENSOR_DETECTION_RANGE
)

class AMREdgeAgent:
    """
    Independent AMR Agent running its own decentralized edge control loop.
    """

    def __init__(
        self,
        robot_id: str,
        name: str,
        color: str,
        initial_x: int,
        initial_y: int,
        warehouse_map
    ):
        self.state = RobotState(
            id=robot_id,
            name=name,
            color=color,
            x=float(initial_x),
            y=float(initial_y),
            grid_x=initial_x,
            grid_y=initial_y,
            heading=0.0,
            status=RobotStatus.IDLE,
            battery=100.0,
            speed=1.0
        )
        self.warehouse_map = warehouse_map
        self.ai_engine = EdgeAIDecisionEngine(robot_id)

        # Current active task object
        self.active_task: Optional[Task] = None
        self.task_phase: str = "NONE" # "TO_PICKUP", "LOADING", "TO_DROPOFF", "UNLOADING"
        self.work_timer: int = 0      # Ticks spent loading/unloading

        # Local knowledge base
        self.local_blocked_cells: Set[Tuple[int, int]] = set()
        self.peer_heartbeats: Dict[str, HeartbeatMessage] = {}

        # Deadlock / Yield tracking
        self.yielding_ticks_remaining: int = 0
        self.stuck_counter: int = 0

    def generate_heartbeat(self) -> HeartbeatMessage:
        """Generates outgoing status message to broadcast to peer AMRs."""
        next_step = self.state.path[0] if self.state.path else None
        return HeartbeatMessage(
            robot_id=self.state.id,
            timestamp=time.time(),
            grid_x=self.state.grid_x,
            grid_y=self.state.grid_y,
            heading=self.state.heading,
            status=self.state.status,
            battery=self.state.battery,
            speed=self.state.speed,
            current_task_id=self.state.current_task_id,
            next_step=next_step,
            intended_path=list(self.state.path[:5])
        )

    def receive_peer_heartbeat(self, heartbeat: HeartbeatMessage):
        """Processes incoming peer status update."""
        if heartbeat.robot_id != self.state.id:
            self.peer_heartbeats[heartbeat.robot_id] = heartbeat

    def receive_map_update(self, blocked_cell: Tuple[int, int], is_blocked: bool):
        """Receives a peer-broadcasted alert about a newly blocked or cleared aisle."""
        if is_blocked:
            self.local_blocked_cells.add(blocked_cell)
            # If current planned path passes through this newly blocked cell, trigger immediate reroute!
            if any(step == blocked_cell for step in self.state.path):
                self.replan_current_mission()
        else:
            self.local_blocked_cells.discard(blocked_cell)

    def assign_task(self, task: Task) -> bool:
        """Assigns a new task to this AMR and plans path to pickup point."""
        self.active_task = task
        self.state.current_task_id = task.id
        self.task_phase = "TO_PICKUP"
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot_id = self.state.id

        # Plan path from current position to pickup location
        path = self._plan_path((self.state.grid_x, self.state.grid_y), (task.pickup_x, task.pickup_y))
        if path and len(path) > 1:
            self.state.path = path[1:] # Exclude current starting cell
            self.state.status = RobotStatus.MOVING
            self.state.target_x = task.pickup_x
            self.state.target_y = task.pickup_y
            return True
        elif path and len(path) == 1:
            # Already at pickup location
            self.task_phase = "LOADING"
            self.work_timer = 2
            self.state.status = RobotStatus.WAITING
            return True
        else:
            # Cannot reach pickup
            return False

    def replan_current_mission(self) -> bool:
        """Recalculates route to current destination using updated obstacle knowledge."""
        if not self.state.target_x or not self.state.target_y:
            return False

        goal = (self.state.target_x, self.state.target_y)
        new_path = self._plan_path((self.state.grid_x, self.state.grid_y), goal)

        if new_path and len(new_path) > 1:
            self.state.path = new_path[1:]
            self.state.status = RobotStatus.REROUTING
            return True
        return False

    def _plan_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Internal path planning helper combining static map and local dynamic knowledge."""
        blocked = set(self.local_blocked_cells)
        # Avoid cells where peer robots are currently stopped
        for pid, hb in self.peer_heartbeats.items():
            if hb.status in [RobotStatus.CHARGING, RobotStatus.WAITING]:
                blocked.add((hb.grid_x, hb.grid_y))

        return a_star_search(
            start=start,
            goal=goal,
            is_walkable_fn=self.warehouse_map.is_walkable,
            get_neighbors_fn=self.warehouse_map.get_neighbors,
            blocked_cells=blocked
        )

    def route_to_nearest_charger(self):
        """Redirects AMR to nearest charging bay."""
        charger_pos = self.warehouse_map.get_nearest_charging_bay(self.state.grid_x, self.state.grid_y)
        path = self._plan_path((self.state.grid_x, self.state.grid_y), charger_pos)
        if path and len(path) > 1:
            self.state.path = path[1:]
            self.state.target_x = charger_pos[0]
            self.state.target_y = charger_pos[1]
            self.state.status = RobotStatus.MOVING
            self.task_phase = "TO_CHARGER"

    def retreat_to_siding(self, siding_pos: Tuple[int, int]):
        """Executes a retreat maneuver into a siding/bay to resolve a deadlock."""
        path = self._plan_path((self.state.grid_x, self.state.grid_y), siding_pos)
        if path and len(path) > 1:
            self.state.path = path[1:]
            self.state.status = RobotStatus.YIELDING
            self.yielding_ticks_remaining = 3

    def step_simulation(self) -> List[EventLog]:
        """
        Executes one discrete simulation step for this AMR.
        Returns any generated events (collisions avoided, deadlocks resolved, reroutes, battery alarms).
        """
        events: List[EventLog] = []

        # 1. Charging Station Behavior
        if self.state.status == RobotStatus.CHARGING:
            self.state.battery = min(100.0, self.state.battery + BATTERY_RECHARGE_RATE)
            if self.state.battery >= 98.0:
                self.state.status = RobotStatus.IDLE
                self.task_phase = "NONE"
                events.append(EventLog(
                    id=f"evt_{time.time()}_{self.state.id}",
                    timestamp=time.time(),
                    event_type="BATTERY_CHARGED",
                    robot_id=self.state.id,
                    title="Battery Fully Charged",
                    details=f"{self.state.name} ({self.state.id}) completed recharge (100%). Ready for tasks."
                ))
            return events

        # 2. Check Autonomous Battery Return Threshold
        nearest_charger = self.warehouse_map.get_nearest_charging_bay(self.state.grid_x, self.state.grid_y)
        dynamic_threshold = self.ai_engine.calculate_dynamic_charging_threshold(
            (self.state.grid_x, self.state.grid_y), nearest_charger
        )

        if self.state.battery <= dynamic_threshold and self.task_phase != "TO_CHARGER":
            events.append(EventLog(
                id=f"evt_{time.time()}_{self.state.id}",
                timestamp=time.time(),
                event_type="BATTERY_LOW",
                robot_id=self.state.id,
                title="Low Battery Warning",
                details=f"{self.state.name} battery at {self.state.battery:.1f}%. Autonomously rerouting to charging station."
            ))
            # Handoff active task if any
            if self.active_task:
                self.active_task.status = TaskStatus.PENDING
                self.active_task.assigned_robot_id = None
                self.active_task = None
                self.state.current_task_id = None

            self.route_to_nearest_charger()

        # 3. Handle Yielding Wait Timer
        if self.yielding_ticks_remaining > 0:
            self.yielding_ticks_remaining -= 1
            self.state.status = RobotStatus.YIELDING
            self.state.battery = max(0.0, self.state.battery - BATTERY_DRAIN_IDLE)
            return events

        # 4. Handle In-Place Loading / Unloading
        if self.task_phase == "LOADING":
            self.work_timer -= 1
            self.state.status = RobotStatus.WAITING
            self.state.battery = max(0.0, self.state.battery - BATTERY_DRAIN_IDLE)
            if self.work_timer <= 0:
                # Finished loading! Plan path to dropoff
                if self.active_task:
                    self.task_phase = "TO_DROPOFF"
                    self.active_task.status = TaskStatus.IN_TRANSIT
                    path = self._plan_path((self.state.grid_x, self.state.grid_y), (self.active_task.dropoff_x, self.active_task.dropoff_y))
                    if path and len(path) > 1:
                        self.state.path = path[1:]
                        self.state.status = RobotStatus.MOVING
                        self.state.target_x = self.active_task.dropoff_x
                        self.state.target_y = self.active_task.dropoff_y
            return events

        if self.task_phase == "UNLOADING":
            self.work_timer -= 1
            self.state.status = RobotStatus.WAITING
            self.state.battery = max(0.0, self.state.battery - BATTERY_DRAIN_IDLE)
            if self.work_timer <= 0:
                # Finished task!
                if self.active_task:
                    self.active_task.status = TaskStatus.COMPLETED
                    self.active_task.completed_at = time.time()
                    events.append(EventLog(
                        id=f"evt_{time.time()}_{self.state.id}",
                        timestamp=time.time(),
                        event_type="TASK_COMPLETED",
                        robot_id=self.state.id,
                        title="Task Completed",
                        details=f"{self.state.name} successfully delivered task '{self.active_task.description}'."
                    ))
                self.active_task = None
                self.state.current_task_id = None
                self.task_phase = "NONE"
                self.state.status = RobotStatus.IDLE
                self.state.target_x = None
                self.state.target_y = None
            return events

        # 5. Check Forward Sensor for Dynamic Blocked Aisles
        if self.state.path:
            next_cell = self.state.path[0]
            # Check if warehouse map has dynamic obstacle on our next step
            if not self.warehouse_map.is_walkable(next_cell[0], next_cell[1]):
                self.local_blocked_cells.add(next_cell)
                events.append(EventLog(
                    id=f"evt_{time.time()}_{self.state.id}",
                    timestamp=time.time(),
                    event_type="AISLE_BLOCKED",
                    robot_id=self.state.id,
                    title="Blocked Aisle Detected",
                    details=f"{self.state.name} sensor detected blocked path at ({next_cell[0]}, {next_cell[1]}). Broadcasting alert."
                ))
                self.replan_current_mission()
                return events

        # 6. Execute Path Navigation Step
        if self.state.path:
            next_cell = self.state.path[0]

            # Update heading angle towards next cell
            dx = next_cell[0] - self.state.grid_x
            dy = next_cell[1] - self.state.grid_y
            angle = math.degrees(math.atan2(dy, dx))
            self.state.heading = (angle + 360) % 360

            # Advance to next cell
            self.state.grid_x = next_cell[0]
            self.state.grid_y = next_cell[1]
            self.state.x = float(next_cell[0])
            self.state.y = float(next_cell[1])
            self.state.path.pop(0)

            self.state.total_steps += 1
            self.state.total_distance += 1.0
            self.state.battery = max(0.0, self.state.battery - BATTERY_DRAIN_PER_STEP)
            self.state.status = RobotStatus.MOVING

            # Check if arrived at milestone
            if not self.state.path:
                if self.task_phase == "TO_PICKUP":
                    self.task_phase = "LOADING"
                    self.work_timer = 2
                    self.state.status = RobotStatus.WAITING
                elif self.task_phase == "TO_DROPOFF":
                    self.task_phase = "UNLOADING"
                    self.work_timer = 2
                    self.state.status = RobotStatus.WAITING
                elif self.task_phase == "TO_CHARGER":
                    self.state.status = RobotStatus.CHARGING
                else:
                    self.state.status = RobotStatus.IDLE
        else:
            # Idle at current spot
            self.state.battery = max(0.0, self.state.battery - BATTERY_DRAIN_IDLE)
            if self.state.status not in [RobotStatus.CHARGING, RobotStatus.WAITING]:
                self.state.status = RobotStatus.IDLE

        return events
