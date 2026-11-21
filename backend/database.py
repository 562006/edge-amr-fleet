"""
SQLite Database Layer for Edge-AI AMR Fleet Coordination.
Stores persistent fleet states, task records, event audit logs, and performance telemetry.
"""

import sqlite3
import time
from typing import List, Dict, Any, Optional
from config import DB_PATH
from backend.models import Task, EventLog, RobotState

def get_connection() -> sqlite3.Connection:
    conn = sqlite3.connect(str(DB_PATH), check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    """Initializes tables with proper schema if they do not exist."""
    conn = get_connection()
    cursor = conn.cursor()

    # Robots Table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS robots (
        id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        color TEXT NOT NULL,
        grid_x INTEGER NOT NULL,
        grid_y INTEGER NOT NULL,
        battery REAL NOT NULL,
        status TEXT NOT NULL,
        total_distance REAL DEFAULT 0.0,
        tasks_completed INTEGER DEFAULT 0,
        updated_at REAL NOT NULL
    );
    """)

    # Tasks Table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS tasks (
        id TEXT PRIMARY KEY,
        description TEXT NOT NULL,
        pickup_x INTEGER NOT NULL,
        pickup_y INTEGER NOT NULL,
        dropoff_x INTEGER NOT NULL,
        dropoff_y INTEGER NOT NULL,
        priority INTEGER NOT NULL,
        status TEXT NOT NULL,
        assigned_robot_id TEXT,
        created_at REAL NOT NULL,
        completed_at REAL
    );
    """)

    # Event Logs Table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS event_logs (
        id TEXT PRIMARY KEY,
        timestamp REAL NOT NULL,
        event_type TEXT NOT NULL,
        robot_id TEXT,
        title TEXT NOT NULL,
        details TEXT NOT NULL
    );
    """)

    # Telemetry Snapshot History Table
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS telemetry_history (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp REAL NOT NULL,
        active_robots INTEGER NOT NULL,
        idle_robots INTEGER NOT NULL,
        waiting_robots INTEGER NOT NULL,
        charging_robots INTEGER NOT NULL,
        completed_tasks INTEGER NOT NULL,
        pending_tasks INTEGER NOT NULL,
        collision_events INTEGER NOT NULL,
        deadlock_events INTEGER NOT NULL
    );
    """)

    conn.commit()
    conn.close()

def upsert_robot(robot: RobotState, tasks_completed: int = 0):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO robots (id, name, color, grid_x, grid_y, battery, status, total_distance, tasks_completed, updated_at)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    ON CONFLICT(id) DO UPDATE SET
        grid_x = excluded.grid_x,
        grid_y = excluded.grid_y,
        battery = excluded.battery,
        status = excluded.status,
        total_distance = excluded.total_distance,
        tasks_completed = excluded.tasks_completed,
        updated_at = excluded.updated_at
    """, (
        robot.id, robot.name, robot.color, robot.grid_x, robot.grid_y,
        robot.battery, robot.status.value, robot.total_distance,
        tasks_completed, time.time()
    ))
    conn.commit()
    conn.close()

def save_task(task: Task):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO tasks (id, description, pickup_x, pickup_y, dropoff_x, dropoff_y, priority, status, assigned_robot_id, created_at, completed_at)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    ON CONFLICT(id) DO UPDATE SET
        status = excluded.status,
        assigned_robot_id = excluded.assigned_robot_id,
        completed_at = excluded.completed_at
    """, (
        task.id, task.description, task.pickup_x, task.pickup_y,
        task.dropoff_x, task.dropoff_y, task.priority.value,
        task.status.value, task.assigned_robot_id,
        task.created_at, task.completed_at
    ))
    conn.commit()
    conn.close()

def log_event(event: EventLog):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO event_logs (id, timestamp, event_type, robot_id, title, details)
    VALUES (?, ?, ?, ?, ?, ?)
    ON CONFLICT(id) DO NOTHING
    """, (
        event.id, event.timestamp, event.event_type,
        event.robot_id, event.title, event.details
    ))
    conn.commit()
    conn.close()

def record_telemetry(metrics: Dict[str, Any]):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
    INSERT INTO telemetry_history (
        timestamp, active_robots, idle_robots, waiting_robots,
        charging_robots, completed_tasks, pending_tasks,
        collision_events, deadlock_events
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, (
        time.time(),
        metrics.get("active_robots", 0),
        metrics.get("idle_robots", 0),
        metrics.get("waiting_robots", 0),
        metrics.get("charging_robots", 0),
        metrics.get("completed_tasks", 0),
        metrics.get("pending_tasks", 0),
        metrics.get("collision_events", 0),
        metrics.get("deadlock_events", 0)
    ))
    conn.commit()
    conn.close()

def get_recent_events(limit: int = 40) -> List[Dict[str, Any]]:
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM event_logs ORDER BY timestamp DESC LIMIT ?", (limit,))
    rows = cursor.fetchall()
    conn.close()
    return [dict(row) for row in rows]

def get_all_tasks() -> List[Dict[str, Any]]:
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM tasks ORDER BY created_at DESC")
    rows = cursor.fetchall()
    conn.close()
    return [dict(row) for row in rows]

def get_analytics_summary() -> Dict[str, Any]:
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT COUNT(*) as total_tasks, SUM(CASE WHEN status='COMPLETED' THEN 1 ELSE 0 END) as completed FROM tasks")
    task_row = cursor.fetchone()
    cursor.execute("SELECT COUNT(*) as total_events, SUM(CASE WHEN event_type='DEADLOCK_RESOLVED' THEN 1 ELSE 0 END) as deadlocks_resolved, SUM(CASE WHEN event_type='COLLISION_AVOIDED' THEN 1 ELSE 0 END) as collisions_avoided FROM event_logs")
    event_row = cursor.fetchone()
    conn.close()
    return {
        "total_tasks": task_row["total_tasks"] if task_row else 0,
        "completed_tasks": task_row["completed"] if task_row and task_row["completed"] else 0,
        "total_events": event_row["total_events"] if event_row else 0,
        "deadlocks_resolved": event_row["deadlocks_resolved"] if event_row and event_row["deadlocks_resolved"] else 0,
        "collisions_avoided": event_row["collisions_avoided"] if event_row and event_row["collisions_avoided"] else 0,
    }
