from typing import List, Dict
import math
from .state import DroneMissionState
from .math_helpers import safe_divide_points

class ReallocationManager:
    """
    Handles dynamically redistributing search grid points from dropped/removed drones
    to the active swarm participants safely.
    """
    
    @staticmethod
    def calculate_drones_needed(remaining_points: int, points_per_drone: int) -> int:
        if remaining_points <= 0 or points_per_drone <= 0:
            return 0
        # Ceiling division equivalent without math float errors
        return (remaining_points + points_per_drone - 1) // points_per_drone

    @staticmethod
    def redistribute_abandoned_missions(
        abandoned_missions: List[DroneMissionState], 
        active_drones: List[DroneMissionState]
    ) -> bool:
        """
        Takes uncompleted abandoned state boundaries, divides the remaining grid points 
        by the number of active drones, and assigns the reallocated blocks to the active drones.
        
        Returns True if reallocation was successful, False if no drones or no points.
        """
        if not active_drones or not abandoned_missions:
            return False

        # Gather remaining workload
        workload = []
        for abandoned in abandoned_missions:
            remaining = abandoned.total_lines - abandoned.current_line_index
            if remaining > 0:
                workload.append({
                    'sysid': abandoned.sysid,
                    'file': abandoned.csv_file_path,
                    'start_index': abandoned.current_line_index,
                    'end_index': abandoned.total_lines,
                    'points': remaining
                })
        
        if not workload:
            return False
            
        total_active_drones = len(active_drones)
        
        # Sort workload by fewest points remaining first (mimicking original code's ascending sort)
        workload.sort(key=lambda x: x['points'])

        # Pre-calculate points per drone assuming they take half originally (from copter_swarm.py rules)
        points_per_drone_base = [ max(1, original['end_index'] // 2) for original in workload ]
        
        # Determine drone allocation count per workload chunk
        allocations = []
        for idx, job in enumerate(workload):
            needed = ReallocationManager.calculate_drones_needed(job['points'], points_per_drone_base[idx])
            alloc = min(needed, total_active_drones)
            allocations.append(alloc)
            total_active_drones -= alloc
            if total_active_drones <= 0:
                break
                
        # If we ran out of drones but still have workload, clamp the allocations
        while len(allocations) < len(workload):
            allocations.append(0)
            
        drone_pointer = 0
        
        for idx, job in enumerate(workload):
            assigned_drone_count = allocations[idx]
            if assigned_drone_count == 0:
                print(f"[Reallocation] Warning: Uncovered Area points left behind for {job['file']}")
                # Record the dropped points so the controller can flag it
                # In production, this would trigger an Event or Alert.
                continue

            # Divide the points of this job across the assigned_drone_count
            points_distribution = safe_divide_points(job['points'], assigned_drone_count)
            
            current_start = job['start_index']
            for points_for_this_drone in points_distribution:
                if drone_pointer >= len(active_drones):
                    break
                    
                target_drone = active_drones[drone_pointer]
                drone_pointer += 1
                
                # Apply reallocation status
                end = current_start + points_for_this_drone
                
                target_drone.is_reallocated = True
                target_drone.reassigned_file_path = job['file']
                target_drone.reassigned_start_index = current_start
                target_drone.reassigned_end_index = end
                target_drone.current_line_index = current_start
                target_drone.total_lines = end # Caps execution loop to the assigned slice
                
                print(f"[Reallocation] Drone {target_drone.sysid} assigned re-coverage path ({current_start} to {end}) for {job['file']}")
                
                # Shift for the next division chunk
                current_start = end

        return True
