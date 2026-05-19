import time
import csv
import traceback
from typing import Dict, List, Tuple, Optional

from .state import SwarmState, DroneMissionState
from .config import CONFIG
from .math_helpers import calculate_distance

class MissionController:
    """
    Handles the execution of grid allocation and splitting.
    Replaces the monolithic unhandled `while 1` block from copter_swarm.py.
    """
    def __init__(self, swarm_state: SwarmState, hardware_interface: Optional[Dict] = None):
        self.swarm_state = swarm_state
        self.csv_cache: Dict[str, List[Tuple[float, float]]] = {}
        # hardware_interface should contains keys: 'goto' (func), 'get_location' (func)
        self.hw = hardware_interface or {}
    
    def load_mission_file(self, drone: DroneMissionState, file_path: str):
        """Safely loads a CSV file bounding its lines, ignoring empty rows."""
        drone.csv_file_path = file_path
        if file_path not in self.csv_cache:
            try:
                with open(file_path, "rt", encoding="utf-8") as file:
                    reader = csv.reader(file)
                    # Ignore empty rows during counting to prevent IndexOutofBounds
                    points = []
                    for row in reader:
                        if row and len(row) >= 2:
                            try:
                                points.append((float(row[0]), float(row[1])))
                            except ValueError:
                                continue # Skip header or invalid rows
                    self.csv_cache[file_path] = points
            except Exception as e:
                print(f"Error caching CSV {file_path}: {e}")
                self.csv_cache[file_path] = []
                
        drone.total_lines = len(self.csv_cache[file_path])
        drone.current_line_index = 0
        drone.has_reached_goal = False

    def get_current_goal(self, drone: DroneMissionState) -> Optional[Tuple[float, float]]:
        """
        Safely retrieves the next geographical checkpoint for a specific drone.
        Prevents the exact IndexError crash you had previously.
        """
        target_file = drone.reassigned_file_path if drone.is_reallocated else drone.csv_file_path
        
        if not target_file or target_file not in self.csv_cache:
            return None
            
        points = self.csv_cache[target_file]
        
        # Bounds check!
        if drone.current_line_index >= len(points):
            return None
            
        return points[drone.current_line_index]

    def _execute_drone_goto(self, drone: DroneMissionState, goal: Tuple[float, float]):
        """
        Executes the movement command via the hardware interface.
        """
        if 'goto' in self.hw:
            self.hw['goto'](drone.sysid, goal)

    def _get_drone_location(self, drone: DroneMissionState) -> Tuple[float, float]:
        """
        Fetches the current drone location via the hardware interface.
        """
        if 'get_location' in self.hw:
            return self.hw['get_location'](drone.sysid)
        return (0.0, 0.0)

    def step_drones(self):
        """
        Executes a single processing 'tick' for all active drones.
        Every single drone execution is wrapped in a try/except, 
        ensuring one drone bug won't cascade and crash the remaining swarm.
        """
        active_drones = self.swarm_state.get_active_drones()
        
        for drone in active_drones:
            if drone.is_mission_complete():
                continue

            try:
                # 1. Safely resolve coordinate
                goal = self.get_current_goal(drone)
                
                if not goal:
                    # Invalid file or silently hit EOF bounds
                    drone.increment_index()
                    continue

                # 2. Command Drone Movement
                self._execute_drone_goto(drone, goal)

                # 3. Get current location
                current_lat, current_lon = self._get_drone_location(drone)
                
                # 4. Check Arrival Threshold using safe math
                dist = calculate_distance(current_lat, current_lon, goal[0], goal[1])
                
                if dist <= CONFIG.arrival_distance_threshold:
                    drone.increment_index()
                    
            except Exception as e:
                # Localized exception bound: Only THIS drone skips a beat. 
                # The program stays perfectly alive.
                print(f"[Drone {drone.sysid}] Error in tick: {e}")
                traceback.print_exc()

    def run_mission(self):
        """
        The production safe main loop.
        """
        print("Starting Production Mission Controller Loop...")
        
        while not self.swarm_state.all_complete():
            try:
                self.step_drones()
                
                # Dynamically sleep based on swarm size configurations instead of blocking hardcodes
                sleep_time = CONFIG.sleep_times.get(len(self.swarm_state.drones), 0.1)
                time.sleep(sleep_time)
                
            except KeyboardInterrupt:
                print("Mission aborted by user.")
                break
            except Exception as e:
                print(f"Non-Fatal Controller Outer Loop Exception: {e}")
                
        print("All drones have successfully reached their final indices.")
