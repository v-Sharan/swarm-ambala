import sys
import os
import time
import argparse
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Add parent directory to path to allow imports from prod_swarm
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from prod_swarm.state import SwarmState, DroneMissionState
from prod_swarm.config import CONFIG
from prod_swarm.mission_controller import MissionController
from prod_swarm.reallocation import ReallocationManager
from prod_swarm.math_helpers import CoordinateConverter

class SwarmHardwareApp:
    def __init__(self, server_ip: str):
        self.server_ip = server_ip
        self.vehicles = {} # sysid -> vehicle object
        self.swarm_state = SwarmState()
        
        # Path to origin file
        origin_path = r"C:\Users\vshar\Documents\swarm_env\rectangles.yaml"
        self.converter = CoordinateConverter(origin_path)
        
        # Define Hardware Interface for MissionController
        self.hw_interface = {
            'goto': self.hardware_goto,
            'get_location': self.hardware_get_location
        }
        
        self.controller = MissionController(self.swarm_state, self.hw_interface)
        self.lock = threading.Lock()
        self.is_running = True

    def hardware_goto(self, sysid: int, goal_cartesian: tuple):
        """Translates Cartesian goal to Geo and sends to vehicle."""
        with self.lock:
            vehicle = self.vehicles.get(sysid)
            if vehicle:
                # Goal in CSV is (x, y). Convert to (lat, lon)
                # Note: original code used x*2, y*2. 
                # Check: grid_1.csv has 1800-1900.
                lat, lon = self.converter.cart_to_geo(goal_cartesian[0]*2, goal_cartesian[1]*2)
                point = LocationGlobalRelative(lat, lon, 10)
                vehicle.simple_goto(point)

    def hardware_get_location(self, sysid: int) -> tuple:
        """Fetches Geo location and translates to Cartesian (x,y) for controller."""
        with self.lock:
            vehicle = self.vehicles.get(sysid)
            if vehicle and vehicle.location.global_relative_frame:
                loc = vehicle.location.global_relative_frame
                # Convert back to Cartesian for MissionController logic
                x, y = self.converter.geo_to_cart(loc.lat, loc.lon)
                return (x/2, y/2)
        return (0.0, 0.0)

    def connect_all(self):
        """Connects to all 8 predefined ports."""
        ports = [14551, 14552, 14553, 14554, 14555, 14556, 14557, 14558]
        print(f"Connecting to swarm at {self.server_ip}...")
        
        for port in ports:
            try:
                conn_str = f"udpin:{self.server_ip}:{port}"
                print(f"Connecting to {conn_str}...")
                vehicle = connect(conn_str, baud=115200, wait_ready=False, heartbeat_timeout=30)
                
                # Assign to state
                sysid = port - 14550 # Simple mapping or use vehicle.parameters['SYSID_THISMAV']
                with self.lock:
                    self.vehicles[sysid] = vehicle
                    self.swarm_state.add_drone(sysid)
                
                # Initialize mission file (example)
                drone_state = self.swarm_state.drones[sysid]
                path = os.path.join(CONFIG.group_split_dir, f"grid_{sysid}.csv")
                self.controller.load_mission_file(drone_state, path)
                
                print(f"Drone {sysid} connected and initialized.")
            except Exception as e:
                print(f"Failed to connect on port {port}: {e}")

    def monitor_health(self):
        """Background thread to handle drop-outs and reallocation."""
        while self.is_running:
            time.sleep(2)
            dropped_drones = []
            
            with self.lock:
                for sysid, vehicle in list(self.vehicles.items()):
                    # Production check: last_heartbeat
                    if vehicle.last_heartbeat is None or vehicle.last_heartbeat > CONFIG.heartbeat_timeout_sec:
                        print(f"CRITICAL: Drone {sysid} lost heartbeat ({vehicle.last_heartbeat}s).")
                        dropped_drones.append(self.swarm_state.drones[sysid])
                        self.swarm_state.remove_drone(sysid)
                        # We don't delete from self.vehicles immediately to allow background reconnection
            
            if dropped_drones:
                print(f"Triggering Reallocation for {len(dropped_drones)} dropped missions...")
                active_drones = self.swarm_state.get_active_drones()
                ReallocationManager.redistribute_abandoned_missions(dropped_drones, active_drones)

    def run(self):
        self.connect_all()
        
        # Start health monitor
        health_thread = threading.Thread(target=self.monitor_health, daemon=True)
        health_thread.start()
        
        print("Starting Main Mission Loop...")
        try:
            self.controller.run_mission()
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.is_running = False
            for v in self.vehicles.values():
                v.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-ip", type=str, default="127.0.0.1")
    args = parser.parse_args()
    
    app = SwarmHardwareApp(args.server_ip)
    app.run()
