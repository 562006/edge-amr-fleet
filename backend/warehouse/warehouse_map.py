"""
Warehouse Map Layout and Topology Manager.
Defines storage racks, aisles, pickup/dropoff stations, charging bays, and dynamic obstacle tracking.
"""

from typing import List, Tuple, Set, Dict, Optional
from config import GRID_WIDTH, GRID_HEIGHT
from backend.models import Obstacle

class WarehouseMap:
    def __init__(self, width: int = GRID_WIDTH, height: int = GRID_HEIGHT):
        self.width = width
        self.height = height

        # Storage Racks (Non-walkable shelf structures)
        self.racks: Set[Tuple[int, int]] = set()
        self.rack_labels: Dict[Tuple[int, int], str] = {}

        # Stations & Charging Bays
        self.inbound_docks: Dict[str, Tuple[int, int]] = {
            "INBOUND_1": (1, 2),
            "INBOUND_2": (1, 8),
            "INBOUND_3": (1, 13),
        }
        self.outbound_stations: Dict[str, Tuple[int, int]] = {
            "OUTBOUND_1": (22, 2),
            "OUTBOUND_2": (22, 8),
            "OUTBOUND_3": (22, 13),
        }
        self.charging_bays: Dict[str, Tuple[int, int]] = {
            "CHARGER_1": (6, 0),
            "CHARGER_2": (12, 0),
            "CHARGER_3": (18, 0),
        }

        # Static Obstacles (Structural pillars, electrical cabinets)
        self.static_obstacles: Set[Tuple[int, int]] = set()

        # Dynamic Obstacles (e.g., fallen boxes, forklift crossing, temporarily blocked aisles)
        self.dynamic_obstacles: Dict[Tuple[int, int], Obstacle] = {}

        # Build initial layout
        self._init_layout()

    def _init_layout(self):
        """Builds realistic rack blocks and main warehouse pathways."""
        # 4 Dual-Column Rack Blocks
        rack_columns = [
            (4, 5),   # Zone A
            (8, 9),   # Zone B
            (14, 15), # Zone C
            (18, 19)  # Zone D
        ]
        
        # Upper shelves (rows 2 to 5) and Lower shelves (rows 9 to 12)
        upper_rows = list(range(2, 6))
        lower_rows = list(range(9, 13))

        zone_names = ["A", "B", "C", "D"]
        for idx, (col1, col2) in enumerate(rack_columns):
            zone = zone_names[idx]
            # Upper block
            for r in upper_rows:
                for c in (col1, col2):
                    self.racks.add((c, r))
                    self.rack_labels[(c, r)] = f"R-{zone}1-{c}-{r}"
            # Lower block
            for r in lower_rows:
                for c in (col1, col2):
                    self.racks.add((c, r))
                    self.rack_labels[(c, r)] = f"R-{zone}2-{c}-{r}"

        # Structural Pillars
        self.static_obstacles.add((6, 7))
        self.static_obstacles.add((17, 7))
        self.static_obstacles.add((6, 14))
        self.static_obstacles.add((17, 14))

    def is_in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_obstacle(self, x: int, y: int) -> bool:
        if (x, y) in self.racks:
            return True
        if (x, y) in self.static_obstacles:
            return True
        if (x, y) in self.dynamic_obstacles:
            return True
        return False

    def is_walkable(self, x: int, y: int, ignore_dynamic: bool = False) -> bool:
        if not self.is_in_bounds(x, y):
            return False
        if (x, y) in self.racks or (x, y) in self.static_obstacles:
            return False
        if not ignore_dynamic and (x, y) in self.dynamic_obstacles:
            return False
        return True

    def get_neighbors(self, x: int, y: int, ignore_dynamic: bool = False) -> List[Tuple[int, int]]:
        """Returns 4-directional orthogonally adjacent walkable cells."""
        neighbors = []
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = x + dx, y + dy
            if self.is_walkable(nx, ny, ignore_dynamic=ignore_dynamic):
                neighbors.append((nx, ny))
        return neighbors

    def add_dynamic_obstacle(self, x: int, y: int, is_blocked_aisle: bool = False, desc: str = "Obstacle") -> bool:
        if self.is_walkable(x, y):
            obs = Obstacle(id=f"obs_{x}_{y}", x=x, y=y, is_blocked_aisle=is_blocked_aisle, description=desc)
            self.dynamic_obstacles[(x, y)] = obs
            return True
        return False

    def remove_dynamic_obstacle(self, x: int, y: int) -> bool:
        if (x, y) in self.dynamic_obstacles:
            del self.dynamic_obstacles[(x, y)]
            return True
        return False

    def toggle_dynamic_obstacle(self, x: int, y: int) -> bool:
        if (x, y) in self.dynamic_obstacles:
            self.remove_dynamic_obstacle(x, y)
            return False
        else:
            return self.add_dynamic_obstacle(x, y, is_blocked_aisle=True, desc="Blocked Aisle Obstacle")

    def clear_dynamic_obstacles(self):
        self.dynamic_obstacles.clear()

    def get_nearest_charging_bay(self, x: int, y: int) -> Tuple[int, int]:
        best_dist = float('inf')
        best_bay = list(self.charging_bays.values())[0]
        for bay_name, pos in self.charging_bays.items():
            dist = abs(x - pos[0]) + abs(y - pos[1])
            if dist < best_dist:
                best_dist = dist
                best_bay = pos
        return best_bay

    def get_nearest_siding(self, x: int, y: int, occupied_positions: Set[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
        """
        Finds the nearest safe siding/alcove cell to pull aside into during deadlocks or yields.
        Searches in increasing BFS radius around (x, y).
        """
        from collections import deque
        visited = {(x, y)}
        queue = deque([(x, y, 0)])

        while queue:
            cx, cy, dist = queue.popleft()
            if dist > 6:
                break
            
            # Check if this cell is safe: walkable, not occupied by any AMR, and not a central corridor
            if (cx, cy) != (x, y) and self.is_walkable(cx, cy) and (cx, cy) not in occupied_positions:
                # Prefer sidings: cells adjacent to racks or borders
                adjacent_racks = sum(1 for nx, ny in [(cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)] if (nx, ny) in self.racks or not self.is_in_bounds(nx, ny))
                if adjacent_racks >= 1:
                    return (cx, cy)

            for nx, ny in self.get_neighbors(cx, cy):
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))
        
        return None

    def to_dict(self) -> Dict:
        return {
            "width": self.width,
            "height": self.height,
            "racks": list(self.racks),
            "rack_labels": {f"{k[0]}_{k[1]}": v for k, v in self.rack_labels.items()},
            "inbound_docks": self.inbound_docks,
            "outbound_stations": self.outbound_stations,
            "charging_bays": self.charging_bays,
            "static_obstacles": list(self.static_obstacles),
            "dynamic_obstacles": [
                {"id": obs.id, "x": obs.x, "y": obs.y, "is_blocked_aisle": obs.is_blocked_aisle, "description": obs.description}
                for obs in self.dynamic_obstacles.values()
            ]
        }
