"""
Data Models and Enums for AMR Fleet Coordination System.
Designed to be compatible with both standard Python dataclasses and Pydantic.
"""

from enum import Enum
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field, asdict
import time

class RobotStatus(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    WAITING = "WAITING"
    YIELDING = "YIELDING"
    DEADLOCKED = "DEADLOCKED"
    CHARGING = "CHARGING"
    REROUTING = "REROUTING"
    EMERGENCY_STOP = "EMERGENCY_STOP"

class TaskPriority(int, Enum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    URGENT = 4

class TaskStatus(str, Enum):
    PENDING = "PENDING"
    AUCTIONING = "AUCTIONING"
    ASSIGNED = "ASSIGNED"
    IN_TRANSIT = "IN_TRANSIT"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"

@dataclass
class Coordinate:
    x: int
    y: int

    def to_tuple(self) -> Tuple[int, int]:
        return (self.x, self.y)

    def manhattan_distance(self, other: "Coordinate") -> int:
        return abs(self.x - other.x) + abs(self.y - other.y)

    def euclidean_distance(self, other: "Coordinate") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

@dataclass
class Task:
    id: str
    description: str
    pickup_x: int
    pickup_y: int
    dropoff_x: int
    dropoff_y: int
    priority: TaskPriority = TaskPriority.NORMAL
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot_id: Optional[str] = None
    created_at: float = field(default_factory=time.time)
    completed_at: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["priority"] = self.priority.value
        d["status"] = self.status.value
        return d

@dataclass
class RobotState:
    id: str
    name: str
    color: str
    x: float
    y: float
    grid_x: int
    grid_y: int
    heading: float = 0.0          # Angle in degrees (0 = East, 90 = South, 180 = West, 270 = North)
    status: RobotStatus = RobotStatus.IDLE
    battery: float = 100.0
    current_task_id: Optional[str] = None
    target_x: Optional[int] = None
    target_y: Optional[int] = None
    path: List[Tuple[int, int]] = field(default_factory=list)
    speed: float = 1.0
    yield_counter: int = 0
    total_steps: int = 0
    total_distance: float = 0.0
    reserved_cells: List[Tuple[int, int, int]] = field(default_factory=list) # (x, y, time_tick)
    local_blocked_cells: List[Tuple[int, int]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["status"] = self.status.value
        return d

@dataclass
class HeartbeatMessage:
    robot_id: str
    timestamp: float
    grid_x: int
    grid_y: int
    heading: float
    status: RobotStatus
    battery: float
    speed: float
    current_task_id: Optional[str]
    next_step: Optional[Tuple[int, int]]
    intended_path: List[Tuple[int, int]]

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["status"] = self.status.value
        return d

@dataclass
class AuctionBid:
    task_id: str
    robot_id: str
    bid_cost: float
    estimated_steps: int
    battery_level: float
    can_execute: bool
    explanation: str

@dataclass
class Obstacle:
    id: str
    x: int
    y: int
    is_blocked_aisle: bool = False
    description: str = "Obstacle"

@dataclass
class EventLog:
    id: str
    timestamp: float
    event_type: str     # "COLLISION_AVOIDED", "DEADLOCK_RESOLVED", "TASK_AWARDED", "AISLE_BLOCKED", "BATTERY_CHARGING", "REROUTE"
    robot_id: Optional[str]
    title: str
    details: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
