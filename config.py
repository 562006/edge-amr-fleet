"""
Configuration and Parameters for Edge-AI Distributed Fleet Coordination System.
Configures warehouse dimensions, robot physical limits, Edge-AI parameters, and communication settings.
"""

import os
from pathlib import Path

# Base Paths
BASE_DIR = Path(__file__).resolve().parent
DB_PATH = BASE_DIR / "fleet_warehouse.db"

# Warehouse Grid Dimensions
GRID_WIDTH = 24    # 24 columns (X: 0 to 23)
GRID_HEIGHT = 16   # 16 rows    (Y: 0 to 15)
CELL_SIZE_PX = 42  # Base pixel width/height per grid cell on canvas

# Simulation Engine Timing
TICK_INTERVAL_SEC = 0.25   # Real-time seconds per simulation step (4 Hz)
DEFAULT_SIM_SPEED = 1.0    # 1.0x, 2.0x, 4.0x multiplier

# Robot Physical & Operational Parameters
NUM_INITIAL_ROBOTS = 3
ROBOT_MAX_SPEED = 1.0          # Max cells per simulation step
BATTERY_MAX = 100.0            # Percentage
BATTERY_DRAIN_PER_STEP = 0.20  # % drained per movement step
BATTERY_DRAIN_IDLE = 0.02      # % drained while waiting/idle
BATTERY_RECHARGE_RATE = 2.50   # % recharged per tick at charging station
BATTERY_LOW_THRESHOLD = 25.0   # % threshold triggering autonomous charging return
BATTERY_CRITICAL_THRESHOLD = 15.0 # % threshold for urgent task preemption/abandonment

# Safety & Collision Avoidance Radii (in grid units)
SAFETY_CRITICAL_DISTANCE = 1.0  # Immediate danger: stop and negotiate
SAFETY_WARNING_DISTANCE = 2.0   # Caution zone: decelerate or evaluate alternative
SENSOR_DETECTION_RANGE = 3      # Cells visible in forward direction by edge sensors

# Edge-AI Contract Net Protocol (CNP) Auction Weights
# Cost = W_dist * dist + W_batt * (100 - batt) + W_queue * queue - W_urg * urgency
WEIGHT_DISTANCE = 0.45
WEIGHT_BATTERY = 0.30
WEIGHT_WORKLOAD = 0.15
WEIGHT_URGENCY = 0.10

# Deadlock Detection & Resolution
MAX_WAIT_TICKS_BEFORE_DEADLOCK = 3  # Ticks waiting for blocked cell before declaring deadlock
DEADLOCK_SEARCH_DEPTH = 5           # Search radius for siding/retreat cell during deadlock resolution

# Network & Server Settings
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8000
WS_ENDPOINT = "/ws/fleet"
