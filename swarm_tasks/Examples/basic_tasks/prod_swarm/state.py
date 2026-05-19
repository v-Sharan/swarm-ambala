from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict

@dataclass
class DroneMissionState:
    """
    Represents the complete mission state for a single drone.
    This replaces the brittle parallel arrays (grid_path_array, num_lines, removed_grid_path_array, etc.)
    """
    sysid: int
    active: bool = True
    
    # Grid File Execution Status
    csv_file_path: Optional[str] = None
    total_lines: int = 0
    current_line_index: int = 0
    
    # Split/Reallocation status
    is_reallocated: bool = False
    reassigned_start_index: int = 0
    reassigned_end_index: int = 0
    reassigned_file_path: Optional[str] = None
    
    # Runtime properties
    uncovered_points: List[int] = field(default_factory=list)
    uncovered_filenames: List[str] = field(default_factory=list)
    
    has_reached_goal: bool = False

    def is_mission_complete(self) -> bool:
        if self.is_reallocated:
            return self.current_line_index >= self.reassigned_end_index
        return self.current_line_index >= self.total_lines
    
    def increment_index(self):
        """Safely increments bounds."""
        if not self.is_mission_complete():
            self.current_line_index += 1

@dataclass
class SwarmState:
    """
    Container handling all state associated with the active swarm.
    """
    drones: Dict[int, DroneMissionState] = field(default_factory=dict)
    
    def add_drone(self, sysid: int):
        if sysid not in self.drones:
            self.drones[sysid] = DroneMissionState(sysid=sysid)
        self.drones[sysid].active = True

    def remove_drone(self, sysid: int):
        if sysid in self.drones:
            self.drones[sysid].active = False
            
    def get_active_drones(self) -> List[DroneMissionState]:
        return [drone for drone in self.drones.values() if drone.active]
    
    def all_complete(self) -> bool:
        active_drones = self.get_active_drones()
        if not active_drones:
            return False
        return all(drone.is_mission_complete() for drone in active_drones)
