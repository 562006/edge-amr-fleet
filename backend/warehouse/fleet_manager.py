"""
Fleet Manager and Simulation Coordinator.
Integrates Multi-AMR agents, Space-Time deconfliction, Wait-For Graph deadlock resolution,
Contract Net Protocol auctions, and pre-packaged real-world demonstration scenarios.
"""

from typing import Dict, List, Optional, Tuple, Any
import time
import asyncio
from config import NUM_INITIAL_ROBOTS, GRID_WIDTH, GRID_HEIGHT
from backend.models import (
    RobotState, RobotStatus, Task, TaskPriority, TaskStatus,
    EventLog, Coordinate
)
from backend.warehouse.warehouse_map import WarehouseMap
from backend.edge.amr_agent import AMREdgeAgent
from backend.algorithms.space_time_a_star import SpaceTimeReservationTable
from backend.algorithms.collision_avoidance import CollisionNegotiator
from backend.algorithms.deadlock_detector import DeadlockDetector
from backend.algorithms.task_allocator import TaskAllocator
from backend import database

class FleetManager:
    """
    Central simulation harness and peer communication bus for AMRs.
    """

    def __init__(self):
        self.warehouse_map = WarehouseMap()
        self.reservation_table = SpaceTimeReservationTable()
        self.deadlock_detector = DeadlockDetector()

        # Agents map: robot_id -> AMREdgeAgent
        self.robots: Dict[str, AMREdgeAgent] = {}

        # Tasks
        self.tasks: Dict[str, Task] = {}

        # Simulation metrics
        self.is_running: bool = False
        self.sim_speed: float = 1.0
        self.current_tick: int = 0
        self.event_logs: List[EventLog] = []

        # Metrics counters
        self.metrics = {
            "total_robots": NUM_INITIAL_ROBOTS,
            "active_robots": 0,
            "idle_robots": NUM_INITIAL_ROBOTS,
            "waiting_robots": 0,
            "charging_robots": 0,
            "completed_tasks": 0,
            "pending_tasks": 0,
            "collision_events": 0,
            "deadlock_events": 0,
            "average_speed": 1.0
        }

        # Initialize Default 3 AMRs
        self._init_fleet()

    def _init_fleet(self):
        """Spawns standard 3 AMRs with distinct visual colors and start zones."""
        self.robots.clear()
        initial_configs = [
            ("AMR-01", "Atlas", "#00d2ff", 2, 1),
            ("AMR-02", "Bolt",  "#ff007f", 2, 8),
            ("AMR-03", "Titan", "#00ff88", 2, 14)
        ]

        for r_id, name, color, init_x, init_y in initial_configs:
            agent = AMREdgeAgent(
                robot_id=r_id,
                name=name,
                color=color,
                initial_x=init_x,
                initial_y=init_y,
                warehouse_map=self.warehouse_map
            )
            self.robots[r_id] = agent
            database.upsert_robot(agent.state)

    def add_task(
        self,
        description: str,
        pickup_x: int,
        pickup_y: int,
        dropoff_x: int,
        dropoff_y: int,
        priority: TaskPriority = TaskPriority.NORMAL
    ) -> Task:
        """Creates a new warehouse logistics task and announces it for auction."""
        task_id = f"TSK_{int(time.time()*1000) % 100000}"
        task = Task(
            id=task_id,
            description=description,
            pickup_x=pickup_x,
            pickup_y=pickup_y,
            dropoff_x=dropoff_x,
            dropoff_y=dropoff_y,
            priority=priority,
            status=TaskStatus.PENDING,
            created_at=time.time()
        )
        self.tasks[task_id] = task
        database.save_task(task)

        # Log Task Announcement Event
        evt = EventLog(
            id=f"evt_{time.time()}_newtask",
            timestamp=time.time(),
            event_type="TASK_ANNOUNCED",
            robot_id=None,
            title="New Task Created",
            details=f"Task '{description}' (Priority: {priority.name}) announced for CNP auction."
        )
        self._add_event(evt)
        return task

    def _add_event(self, evt: EventLog):
        self.event_logs.insert(0, evt)
        if len(self.event_logs) > 60:
            self.event_logs.pop()
        database.log_event(evt)

    def trigger_blocked_aisle(self, x: int, y: int) -> bool:
        """Toggles a blocked aisle obstacle and broadcasts update to all AMRs."""
        is_blocked = self.warehouse_map.toggle_dynamic_obstacle(x, y)
        action_str = "blocked" if is_blocked else "cleared"

        # Broadcast map update to all edge agents
        for agent in self.robots.values():
            agent.receive_map_update((x, y), is_blocked)

        evt = EventLog(
            id=f"evt_{time.time()}_block",
            timestamp=time.time(),
            event_type="AISLE_BLOCKED" if is_blocked else "AISLE_CLEARED",
            robot_id=None,
            title=f"Aisle Obstacle {action_str.title()}",
            details=f"Location ({x}, {y}) was {action_str}. Map alert broadcast to all AMRs."
        )
        self._add_event(evt)
        return is_blocked

    def step(self):
        """
        Executes one full synchronized coordination tick across the AMR fleet.
        Runs distributed communication, negotiation, deadlock resolution, and agent steps.
        """
        self.current_tick += 1

        # 1. Distributed Task Allocation: Auction pending tasks via Contract Net Protocol
        pending_tasks = [t for t in self.tasks.values() if t.status == TaskStatus.PENDING]
        for t in pending_tasks:
            winner_id, bids = TaskAllocator.conduct_auction(t, [a.state for a in self.robots.values()])
            if winner_id:
                agent = self.robots[winner_id]
                success = agent.assign_task(t)
                if success:
                    database.save_task(t)
                    self._add_event(EventLog(
                        id=f"evt_{time.time()}_auction_{t.id}",
                        timestamp=time.time(),
                        event_type="TASK_AWARDED",
                        robot_id=winner_id,
                        title="CNP Task Contract Awarded",
                        details=f"Task '{t.description}' awarded to {agent.state.name} ({winner_id}). Edge bid winner."
                    ))

        # 2. Multi-Robot Peer-to-Peer Communication (Heartbeat Exchange)
        heartbeats = [agent.generate_heartbeat() for agent in self.robots.values()]
        for hb in heartbeats:
            for agent in self.robots.values():
                agent.receive_peer_heartbeat(hb)

        # 3. Distributed Collision Avoidance & Right-of-Way Arbitration
        robot_list = list(self.robots.values())
        for i in range(len(robot_list)):
            for j in range(i + 1, len(robot_list)):
                r1 = robot_list[i]
                r2 = robot_list[j]

                conflict_info = CollisionNegotiator.check_proximity_conflict(r1.state, r2.state)
                if conflict_info["is_head_on"] or conflict_info["is_same_target"]:
                    self.metrics["collision_events"] += 1
                    winner_id, yielding_id, rationale = CollisionNegotiator.arbitrate_right_of_way(
                        r1.state, r2.state,
                        r1.active_task.priority if r1.active_task else TaskPriority.NORMAL,
                        r2.active_task.priority if r2.active_task else TaskPriority.NORMAL
                    )

                    yielder = self.robots[yielding_id]
                    # Yield action: pause yielding robot for 2 simulation ticks
                    yielder.yielding_ticks_remaining = 2
                    yielder.state.status = RobotStatus.YIELDING

                    self._add_event(EventLog(
                        id=f"evt_{time.time()}_col_{winner_id}_{yielding_id}",
                        timestamp=time.time(),
                        event_type="COLLISION_AVOIDED",
                        robot_id=yielding_id,
                        title="Collision Avoidance Negotiation",
                        details=f"{rationale}. {yielder.state.name} yielding right-of-way."
                    ))

        # 4. Deadlock Detection (Wait-For Graph Cycles)
        for agent in self.robots.values():
            if agent.state.status in [RobotStatus.WAITING, RobotStatus.YIELDING] and agent.state.path:
                desired_cell = agent.state.path[0]
                # Check who is occupying desired cell
                blocker_id = None
                for other_agent in self.robots.values():
                    if other_agent.state.id != agent.state.id and (other_agent.state.grid_x, other_agent.state.grid_y) == desired_cell:
                        blocker_id = other_agent.state.id
                        break
                self.deadlock_detector.update_waiting_relationship(agent.state.id, blocker_id)
            else:
                self.deadlock_detector.update_waiting_relationship(agent.state.id, None)

        cycles = self.deadlock_detector.detect_cycles()
        if cycles:
            for cycle in cycles:
                self.metrics["deadlock_events"] += 1
                victim_id = self.deadlock_detector.select_resolution_victim(
                    cycle, {r.state.id: r.state for r in self.robots.values()}
                )
                victim_agent = self.robots[victim_id]
                occupied_cells = {(r.state.grid_x, r.state.grid_y) for r in self.robots.values()}
                siding = self.warehouse_map.get_nearest_siding(victim_agent.state.grid_x, victim_agent.state.grid_y, occupied_cells)

                if siding:
                    victim_agent.retreat_to_siding(siding)
                    self._add_event(EventLog(
                        id=f"evt_{time.time()}_deadlock_{victim_id}",
                        timestamp=time.time(),
                        event_type="DEADLOCK_RESOLVED",
                        robot_id=victim_id,
                        title="Circular Deadlock Resolved",
                        details=f"Wait cycle detected: {' -> '.join(cycle)}. {victim_agent.state.name} evacuating to siding ({siding[0]}, {siding[1]})."
                    ))
                else:
                    # Alternative: recalculate path
                    victim_agent.replan_current_mission()

        # 5. Step Individual Edge Agents
        for agent in self.robots.values():
            step_events = agent.step_simulation()
            for evt in step_events:
                self._add_event(evt)
            # Update database
            completed_count = 1 if (agent.active_task and agent.active_task.status == TaskStatus.COMPLETED) else 0
            database.upsert_robot(agent.state, tasks_completed=completed_count)

        # 6. Update Telemetry Metrics
        self._update_metrics()

    def _update_metrics(self):
        active = sum(1 for r in self.robots.values() if r.state.status == RobotStatus.MOVING)
        idle = sum(1 for r in self.robots.values() if r.state.status == RobotStatus.IDLE)
        waiting = sum(1 for r in self.robots.values() if r.state.status in [RobotStatus.WAITING, RobotStatus.YIELDING, RobotStatus.DEADLOCKED])
        charging = sum(1 for r in self.robots.values() if r.state.status == RobotStatus.CHARGING)
        completed = sum(1 for t in self.tasks.values() if t.status == TaskStatus.COMPLETED)
        pending = sum(1 for t in self.tasks.values() if t.status in [TaskStatus.PENDING, TaskStatus.AUCTIONING, TaskStatus.ASSIGNED, TaskStatus.IN_TRANSIT])

        self.metrics.update({
            "total_robots": len(self.robots),
            "active_robots": active,
            "idle_robots": idle,
            "waiting_robots": waiting,
            "charging_robots": charging,
            "completed_tasks": completed,
            "pending_tasks": pending,
        })
        database.record_telemetry(self.metrics)

    # ---------------- Advanced Demonstration Scenarios ----------------

    def load_scenario(self, scenario_id: int):
        """
        Loads one of the 6 requested real-world demonstration scenarios:
        1: Multi-Robot Simultaneous Tasks
        2: Head-On Narrow Aisle Conflict
        3: 3-Way Deadlock Detection & Resolution
        4: Blocked-Aisle Dynamic Rerouting & Peer Broadcast
        5: Low Battery Autonomous Re-docking & Task Handoff
        6: Urgent VIP Task Preemption
        """
        self.reset_simulation()

        if scenario_id == 1:
            # Scenario 1: Multi-Robot Concurrent Operations
            self.robots["AMR-01"].state.grid_x, self.robots["AMR-01"].state.grid_y = 2, 2
            self.robots["AMR-02"].state.grid_x, self.robots["AMR-02"].state.grid_y = 2, 8
            self.robots["AMR-03"].state.grid_x, self.robots["AMR-03"].state.grid_y = 2, 14

            self.add_task("Zone A Pallet Delivery", 1, 2, 22, 2, TaskPriority.NORMAL)
            self.add_task("Zone B Auto-Parts Transport", 1, 8, 22, 8, TaskPriority.NORMAL)
            self.add_task("Zone C Heavy Crates Transfer", 1, 13, 22, 13, TaskPriority.HIGH)

            self._add_event(EventLog(
                id=f"evt_scn_1", timestamp=time.time(), event_type="SCENARIO_STARTED",
                robot_id=None, title="Scenario 1 Loaded",
                details="3 AMRs running simultaneous cross-warehouse missions with Space-Time coordination."
            ))

        elif scenario_id == 2:
            # Scenario 2: Head-On Narrow Aisle Conflict
            # Place AMR-01 at (11, 7) heading south, AMR-02 at (11, 13) heading north inside narrow column 11
            self.robots["AMR-01"].state.grid_x, self.robots["AMR-01"].state.grid_y = 11, 7
            self.robots["AMR-01"].state.path = [(11, 8), (11, 9), (11, 10), (11, 11), (11, 12), (11, 13)]
            self.robots["AMR-01"].state.status = RobotStatus.MOVING
            self.robots["AMR-01"].state.target_x, self.robots["AMR-01"].state.target_y = 11, 13

            self.robots["AMR-02"].state.grid_x, self.robots["AMR-02"].state.grid_y = 11, 13
            self.robots["AMR-02"].state.path = [(11, 12), (11, 11), (11, 10), (11, 9), (11, 8), (11, 7)]
            self.robots["AMR-02"].state.status = RobotStatus.MOVING
            self.robots["AMR-02"].state.target_x, self.robots["AMR-02"].state.target_y = 11, 7

            self._add_event(EventLog(
                id=f"evt_scn_2", timestamp=time.time(), event_type="SCENARIO_STARTED",
                robot_id=None, title="Scenario 2 Loaded",
                details="AMR-01 and AMR-02 approaching head-on in narrow single aisle. System will negotiate right-of-way."
            ))

        elif scenario_id == 3:
            # Scenario 3: 3-Way Deadlock Detection & Resolution
            # Position 3 AMRs in a tight circular junction at (6, 6), (7, 6), (7, 7)
            self.robots["AMR-01"].state.grid_x, self.robots["AMR-01"].state.grid_y = 6, 6
            self.robots["AMR-01"].state.path = [(7, 6)]
            self.robots["AMR-01"].state.status = RobotStatus.WAITING

            self.robots["AMR-02"].state.grid_x, self.robots["AMR-02"].state.grid_y = 7, 6
            self.robots["AMR-02"].state.path = [(7, 7)]
            self.robots["AMR-02"].state.status = RobotStatus.WAITING

            self.robots["AMR-03"].state.grid_x, self.robots["AMR-03"].state.grid_y = 7, 7
            self.robots["AMR-03"].state.path = [(6, 6)]
            self.robots["AMR-03"].state.status = RobotStatus.WAITING

            self._add_event(EventLog(
                id=f"evt_scn_3", timestamp=time.time(), event_type="SCENARIO_STARTED",
                robot_id=None, title="Scenario 3 Loaded",
                details="3-Way circular deadlock injected. Wait-For Graph will detect loop and initiate evacuation."
            ))

        elif scenario_id == 4:
            # Scenario 4: Dynamic Blocked Aisle & Broadcast Rerouting
            self.robots["AMR-01"].state.grid_x, self.robots["AMR-01"].state.grid_y = 3, 7
            t = self.add_task("Express Aisle Route", 3, 7, 21, 7, TaskPriority.HIGH)
            # Inject blocked aisle right in the middle at (10, 7)
            self.warehouse_map.add_dynamic_obstacle(10, 7, is_blocked_aisle=True, desc="Blocked Aisle Obstacle")

            self._add_event(EventLog(
                id=f"evt_scn_4", timestamp=time.time(), event_type="SCENARIO_STARTED",
                robot_id=None, title="Scenario 4 Loaded",
                details="Central corridor (10, 7) blocked. AMR-01 forward sensor will detect blockage and notify fleet."
            ))

        elif scenario_id == 5:
            # Scenario 5: Low Battery Autonomous Re-docking & Task Handoff
            self.robots["AMR-02"].state.battery = 18.0 # Force battery below low threshold
            self.robots["AMR-02"].state.grid_x, self.robots["AMR-02"].state.grid_y = 10, 8
            self.add_task("Priority Medical Package", 10, 8, 22, 2, TaskPriority.HIGH)

            self._add_event(EventLog(
                id=f"evt_scn_5", timestamp=time.time(), event_type="SCENARIO_STARTED",
                robot_id="AMR-02", title="Scenario 5 Loaded",
                details="AMR-02 battery depleted to 18%. AMR-02 will handoff mission and head to Charger 2."
            ))

        elif scenario_id == 6:
            # Scenario 6: Urgent VIP Task Preemption
            self.robots["AMR-01"].state.grid_x, self.robots["AMR-01"].state.grid_y = 4, 1
            self.robots["AMR-02"].state.grid_x, self.robots["AMR-02"].state.grid_y = 4, 8
            self.add_task("Standard Routine Pallet", 4, 1, 20, 1, TaskPriority.LOW)
            # Inject VIP Urgent Task
            self.add_task("URGENT: Hazardous Spill Containment Kit", 6, 8, 22, 13, TaskPriority.URGENT)

            self._add_event(EventLog(
                id=f"evt_scn_6", timestamp=time.time(), event_type="SCENARIO_STARTED",
                robot_id=None, title="Scenario 6 Loaded",
                details="Urgent VIP task injected. Contract Net Protocol auction gives top priority to urgent mission."
            ))

    def reset_simulation(self):
        """Restores warehouse map and AMRs to default initial condition."""
        self.is_running = False
        self.current_tick = 0
        self.tasks.clear()
        self.event_logs.clear()
        self.warehouse_map.clear_dynamic_obstacles()
        self._init_fleet()
        self.metrics["collision_events"] = 0
        self.metrics["deadlock_events"] = 0
        self.metrics["completed_tasks"] = 0
        self.metrics["pending_tasks"] = 0

    def get_simulation_state(self) -> Dict[str, Any]:
        """Serializes current entire simulation state for WebSockets and REST APIs."""
        return {
            "tick": self.current_tick,
            "is_running": self.is_running,
            "speed": self.sim_speed,
            "metrics": self.metrics,
            "robots": [agent.state.to_dict() for agent in self.robots.values()],
            "tasks": [t.to_dict() for t in self.tasks.values()],
            "map": self.warehouse_map.to_dict(),
            "recent_events": [e.to_dict() for e in self.event_logs[:20]]
        }
