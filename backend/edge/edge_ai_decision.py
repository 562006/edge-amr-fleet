"""
Edge-AI Decision-Making Engine for Autonomous Mobile Robots.
Combines machine-learning heuristics, energy consumption forecasting,
and adaptive speed modulation based on localized spatial traffic density.

AI vs Traditional Algorithm Differentiation:
- Traditional Algorithms: Deterministic A* graph search, Manhattan distance, Wait-For Graph cycle detection.
- Edge-AI Models: Multi-criteria utility networks for task bidding, dynamic battery depletion forecasting,
  localized congestion density estimation, and predictive deadlock avoidance.
"""

from typing import List, Tuple, Dict, Any, Optional
import math
from config import BATTERY_DRAIN_PER_STEP

class EdgeAIDecisionEngine:
    """
    Onboard Edge-AI intelligence embedded within each AMR's edge compute unit.
    """

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        # Historical step metrics for adaptive learning
        self.recent_speeds: List[float] = []
        self.stop_and_go_events: int = 0

    def predict_energy_consumption(
        self,
        planned_path: List[Tuple[int, int]],
        is_carrying_payload: bool = False
    ) -> float:
        """
        Edge-AI Energy Consumption Model.
        Predicts total battery percentage required to traverse planned path.
        Accounts for:
        1. Linear travel steps
        2. Cornering / 90-degree turns (acceleration/deceleration energy spikes)
        3. Payload inertia penalty (carrying cargo uses ~25% more energy)
        """
        if not planned_path or len(planned_path) < 2:
            return 0.0

        base_steps = len(planned_path) - 1
        turn_count = 0

        # Calculate number of direction changes / turns
        for i in range(len(planned_path) - 2):
            p1, p2, p3 = planned_path[i], planned_path[i+1], planned_path[i+2]
            v1 = (p2[0] - p1[0], p2[1] - p1[1])
            v2 = (p3[0] - p2[0], p3[1] - p2[1])
            if v1 != v2:
                turn_count += 1

        payload_factor = 1.25 if is_carrying_payload else 1.00
        turn_penalty = turn_count * 0.08  # Additional 0.08% battery per turn maneuver

        total_predicted_drain = (base_steps * BATTERY_DRAIN_PER_STEP * payload_factor) + turn_penalty
        return round(total_predicted_drain, 2)

    def calculate_dynamic_charging_threshold(
        self,
        current_pos: Tuple[int, int],
        nearest_charger_pos: Tuple[int, int]
    ) -> float:
        """
        Dynamically calculates the minimum battery threshold needed to safely reach a charger.
        Replaces fixed static thresholds with an adaptive distance-aware safety margin.
        """
        dist_to_charger = abs(current_pos[0] - nearest_charger_pos[0]) + abs(current_pos[1] - nearest_charger_pos[1])
        # Energy required = distance * drain_rate * safety_multiplier (1.5x) + 5% emergency reserve
        dynamic_threshold = (dist_to_charger * BATTERY_DRAIN_PER_STEP * 1.5) + 5.0
        return max(15.0, min(35.0, dynamic_threshold))

    def evaluate_local_congestion_density(
        self,
        my_pos: Tuple[int, int],
        peer_positions: List[Tuple[int, int]],
        sensing_radius: int = 4
    ) -> float:
        """
        Calculates localized traffic density index [0.0 = clear, 1.0 = heavy congestion].
        Uses inverse square distance weighting of neighboring robots.
        """
        if not peer_positions:
            return 0.0

        density_score = 0.0
        for px, py in peer_positions:
            dist = max(1.0, ((my_pos[0] - px)**2 + (my_pos[1] - py)**2)**0.5)
            if dist <= sensing_radius:
                density_score += (1.0 / dist)

        normalized_density = min(1.0, density_score / 2.0)
        return round(normalized_density, 3)

    def compute_adaptive_speed(
        self,
        base_speed: float,
        congestion_index: float,
        upcoming_turn_ahead: bool
    ) -> float:
        """
        Dynamically modulates AMR cruising speed to smooth traffic flow and conserve battery.
        High local congestion -> decelerate smoothly to prevent sudden stops.
        Clear aisle -> maintain nominal cruise speed.
        """
        speed_factor = 1.0 - (0.4 * congestion_index)
        if upcoming_turn_ahead:
            speed_factor *= 0.85

        adapted_speed = max(0.4, base_speed * speed_factor)
        return round(adapted_speed, 2)
