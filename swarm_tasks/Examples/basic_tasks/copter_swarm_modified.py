print("Version Swarm Copter V14")
import sys, os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
import swarm_tasks
from time import sleep,time
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
import swarm_tasks.controllers.potential_field as potf
from swarm_tasks.modules.dispersion import disp_field
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules import exploration as exp
from swarm_tasks.tasks import area_coverage as cvg
from math import radians, sin, cos, sqrt, atan2, asin, degrees
from dronekit import connect, VehicleMode, LocationGlobalRelative,APIException
from swarm_tasks.modules.navigate import NavigationGridGenerator
from swarm_tasks.modules.groupsplitauto import AutoSplitMission
from swarm_tasks.modules.search_grid import SearchGridGenerator
from swarm_tasks.modules.search import PolygonSearchGrid
from swarm_tasks.modules.groupsplitspecific import SpecificSplitMission
from swarm_tasks.modules.multipoly_grid import PolygonAutoSplit
from swarm_tasks.modules.multipoly_specificgrid import PolygonSpecificSplit
import socket, json, csv, threading, yaml
import swarm_tasks.modules.locatePosition as locatePosition

# import netifaces, wmi
import math
from pathlib import Path
import argparse

from utils import log, define_log_path,read_origin
from server import SocketServer
from config import *

import logging

# Send dronekit logs to file instead of terminal
file_handler = logging.FileHandler('dronekit.log')
file_handler.setLevel(logging.DEBUG)

for logger_name in ['dronekit', 'mavutil', 'pymavlink']:
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.propagate = False   # prevent printing to terminal
        
class SwarmController:
    def __init__(self,heartbeat_ip,heartbeat_ip_timeout):
        self.vehicles = []
        self.home_pos_lat_lon = []
        self.home_pos = []
        self.pos_array = []
        self.endDistance = 500000
        self.heartbeat_ip = heartbeat_ip
        self.heartbeat_ip_timeout = heartbeat_ip_timeout
        self.num_bots = 8
        self.port_array = [14551, 14552, 14553, 14554, 14555, 14556, 14557, 14558]
        self.robots = [(0, 0)] * 8
        self.uav_homepos_array = []
        self.current_lat_lon = []
        self.csv_cache = {}
        self.bot_speed = 3.0
        self.uav_home_pos = []
        self.different_height = [50, 60, 70, 80, 90, 100, 110, 120]
        self.stop_swarm = StopSwarm()
        self.sock3 = SocketServer(ip, 12002)

        documents_path = os.path.join(os.path.expanduser("~"), "Documents")
        self.file_name = os.path.join(
            documents_path, "swarm_env", "rectangles.yaml"
        )  # Replace 'your_file.yaml' with actual file name
        print("read_origin_path", self.file_name)
        
        self.CHECK_network_connection()
        self.connect_vehicles()
        while not self.all_drones_armed():
            print("Waiting for all drones to be armed...")
            sleep(1)
        self.origin = read_origin(self.file_name)
        print("Origin!!!!", self.origin)
        self.fetch_home_locations()
        while not self.uav_home_pos:
            sleep(0.1)
        threading.Thread(target=self.monitor_socket, daemon=True).start()
        self.num_bots = len(self.pos_array)
        self.specific_goal_pos = [0] * self.num_bots
        self.specific_bot_goal_flag_array = [False] * self.num_bots
        self.specific_goal_xy_index = [0] * self.num_bots
        self.s = sim.Simulation(
            self.uav_home_pos, num_bots=self.num_bots, env_name=self.file_name, speed=self.bot_speed
        )
        
    def monitor_socket(self):
        print("Starting socket monitor thread...")
        while True:
            data, addr = self.sock3.monitor()  # Buffer size is 1024 bytes
            print(f"Received message from {addr}: {data} in sock3")
            if data.lower() == "STOP":
                print("STOP command received. Stopping the swarm...")
                self.stop_swarm._push_command("STOP")
    
    def home_lock(self):
        print("Attempting to lock home positions for all vehicles...")
        self.home_pos = []
        self.home_pos_lat_lon = []
        print("Vehicles to lock:", len(self.vehicles))
        for i, vehicle in enumerate(self.vehicles):
            # Wait until vehicle.home_location is valid or timeout to avoid infinite loop
            timeout = time() + 5  # 30 seconds timeout
            while not vehicle.home_location:
                cmds = vehicle.commands
                cmds.download()
                cmds.wait_ready()
                if not vehicle.home_location:
                    print(" Waiting for home position...")
                    sleep(1)
                if time() > timeout:
                    print(f"Timeout waiting for home_location for vehicle {i}")
                    break

            # Assign home only if available
            home = vehicle.home_location
            if home is None:
                continue  # skip to next vehicle

            x, y = locatePosition.geoToCart(self.origin, self.endDistance, [home.lat, home.lon])
            self.home_pos_lat_lon.append((home.lat, home.lon))
            self.home_pos.append((x / 2, y / 2))
            self.uav_home_pos.append((x / 2, y / 2))
    
    def CHECK_network_connection(self):
        for i, iter_follower in enumerate(self.heartbeat_ip_timeout):
            # Use -n 1 for Windows, and suppress output
            response = os.system("ping -n 1 " + self.heartbeat_ip[i] + " >nul 2>&1")

            if response == 0:
                self.heartbeat_ip_timeout[i] = 30
            else:  # Link is down.
                print("waiting...")
                linkdown_flag = True
                self.heartbeat_ip_timeout[i] = 30

        print("heartbeat_ip_timeout", self.heartbeat_ip_timeout)
        
    def connect_vehicles(self):
        for i in range(self.num_bots):
            connection_string = f"udp:{self.heartbeat_ip[i]}:{self.port_array[i]}"
            print(f"Connecting to vehicle {i+1} at {connection_string}...")
            try:
                vehicle = connect(connection_string, heartbeat_timeout=self.heartbeat_ip_timeout[i])
                self.vehicles.append(vehicle)
                self.pos_array.append(vehicle._master.target_system)
                print(f"Connected to vehicle {i+1} with target system ID: {vehicle._master.target_system}")
            except APIException as e:
                print(f"[APIException] DroneKit error: {e}, while connecting to vehicle {i+1}")
            except Exception as e:
                print(f"[Exception] Error connecting to vehicle {i+1}: {e}")
        print(f"Total vehicles connected: {len(self.vehicles)}")
        
    def calculate_drones_needed(self, remaining_points, points_per_drone):
        """
        Calculate the number of drones required to cover the remaining points.

        Parameters:
        remaining_points (int): Number of points that need to be covered.
        points_per_drone (int): Number of points each drone can cover.

        Returns:
        int: Number of drones needed to cover the remaining points.
        """
        if remaining_points <= 0:
            return 0
        return (
            remaining_points + points_per_drone - 1
        ) // points_per_drone  # Ceiling division
        
    def allocate_drones(self, total_points, covered_points, total_drones):
        """
        Allocate drones to uncovered areas based on the exact number of drones needed.

        Parameters:
        total_points (list of int): Total number of points to cover in each area.
        covered_points (list of int): Points covered by each drone per area.
        total_drones (int): Total number of drones initially available.

        Returns:
        dict: A dictionary where keys are area indices and values are the number of drones allocated to each area.
        """

        # Define the number of points each drone can cover
        # (assuming same rule: half of area's total points per drone)
        points_per_drone = [int(tp / 2) for tp in total_points]

        # Calculate remaining points for each area
        remaining_points_list = [tp - cp for tp, cp in zip(total_points, covered_points)]
        print("remaining_points_list", remaining_points_list)

        # Filter out areas that are already fully covered
        uncovered_areas = [
            (i, points) for i, points in enumerate(remaining_points_list) if points > 0
        ]

        # Calculate the number of drones needed for each uncovered area
        drones_needed = [
            self.calculate_drones_needed(points, points_per_drone[i])
            for i, points in uncovered_areas
        ]
        print("drones_needed", drones_needed, "remaining drones", total_drones)
        # Initialize allocation with zero drones
        allocation = {i: 0 for i, _ in uncovered_areas}

        # Case 1: total_drones <= uncovered areas
        if total_drones <= len(uncovered_areas):
            # Allocate 1 drone per uncovered area until drones run out
            for i, _ in uncovered_areas:
                if total_drones <= 0:
                    break
                allocation[i] = 1
                total_drones -= 1

        # Case 2: total_drones > uncovered areas
        else:
            for idx, (area_index, _) in enumerate(uncovered_areas):
                if total_drones <= 0:
                    break
                # Calculate max drones we can allocate to this area
                required_drones = min(drones_needed[idx], total_drones)
                allocation[area_index] = required_drones
                total_drones -= required_drones

        # Ensure fully covered areas are zeroed out
        all_areas = {i: 0 for i in range(len(covered_points))}
        all_areas.update(allocation)
        return all_areas, remaining_points_list
    
    def fetch_home_locations(self):
        try:
            self.home_lock()
            print("Home positions:", self.home_pos)
        except:
            for i, vehicle in enumerate(self.vehicles):
                lat = vehicle.location.global_relative_frame.lat
                lon = vehicle.location.global_relative_frame.lon
                # print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
                self.home_pos_lat_lon.append((lat, lon))
                x, y = locatePosition.geoToCart(self.origin, self.endDistance, [lat, lon])
                # print("x,y",x/2,y/2)
                self.home_pos.append((x / 2, y / 2))
                if i < len(self.robots):
                    self.robots[i] = (x / 2, y / 2)
                print("home_pos", self.home_pos)
        try:
            for i, vehicle in enumerate(self.vehicles):
                lat = vehicle.location.global_relative_frame.lat
                lon = vehicle.location.global_relative_frame.lon
                self.current_lat_lon.append((lat, lon))
                x, y = locatePosition.geoToCart(self.origin, self.originendDistance, [lat, lon])
                print("x,y", x / 2, y / 2)
                self.uav_home_pos.append((x / 2, y / 2))
        except:
            pass
        
    def all_drones_armed(self):
        """Check if all drones are armed."""
        return all(v.armed for v in self.vehicles)
    
    def _goto(self, i, lat, lon):
            pt = LocationGlobalRelative(lat, lon, self.different_height[i])
            self.vehicles[i].simple_goto(pt)
    
    def handle_different_height(self,data):
        _, height, step = data.split(",")
        height = int(height)
        step   = int(step)
        self.height_difference = step
        self.different_height  = [height + step * i for i in range(self.num_bots)]
        alt_count = [0] * self.num_bots
        diff_done = False
        while not diff_done:
            for i, b in enumerate(self.s.swarm):
                cmd = potf.velocity(b.get_position(), b.sim,
                                    weights=potf.field_weights, order=1, max_dist=10)

                cmd.exec(b)
                lat, lon = locatePosition.cartToGeo(self.origin, self.endDistance, [b.x * 2, b.y * 2])
                self._goto(i, lat, lon)
                alt = self.vehicles[i].location.global_relative_frame.alt
                if abs(alt - self.different_height[i]) <= 1.5:
                    alt_count[i] = 1
                    if all(c == 1 for c in alt_count):
                        diff_done = True
                        break
                if self.stop_swarm and self.stop_swarm._peek_command() == "STOP":
                    print("STOP command detected in handle_different_height. Stopping the swarm...")
                    self.stop_swarm._pop_command()  # Clear the command after processing
                    break
            sleep(0.05)
            
            
    def handle_specific_bot_goal(self,data):
        _, bot_index, loc = data.split("_")
        f = _
        uav_raw = bot_index.strip().replace("'", '"')
        uav_list = json.loads(uav_raw)
        print("uav_list", uav_list)
        goal_array = loc  # All other coordinates
        goal_latlon = json.loads(goal_array)
        print("goal_latlon", goal_latlon)
        goal_xy = []
        for x in goal_latlon:
            x, y = locatePosition.geoToCart(
                self.origin, self.endDistance, [x[1], x[0]]
            )
            goal_xy.append((x / 2, y / 2))

        goal = [0] * len(self.pos_array)
        for uav_id in uav_list:
            # Find matching bot index
            if int(uav_id) in self.pos_array:
                bot_index = self.pos_array.index(int(uav_id))
                self.specific_goal_pos[bot_index] = goal_xy
                self.specific_bot_goal_flag_array[bot_index] = True
                self.specific_goal_xy_index[bot_index] = 0

            else:
                print("UAV", uav_id, "not found in pos_array")
                continue
        print(
            "specific_bot_goal_flag_array",
            self.specific_bot_goal_flag_array,
            self.specific_goal_pos,
        )
        
        self.uav_home_pos = []
        for vehicle in self.vehicles:
            lat = vehicle.location.global_relative_frame.lat
            lon = vehicle.location.global_relative_frame.lon
            x, y = locatePosition.geoToCart(self.origin, self.endDistance, [lat, lon])
            self.uav_home_pos.append((x / 2, y / 2))

        self.origin = read_origin(self.file_name)
        s = sim.Simulation(
            self.uav_home_pos,
            num_bots=self.num_bots,
            env_name=self.file_name,
            speed=self.bot_speed,
        )
        while True:
            sleep(sleep_times.get(self.num_bots, 0.1))
            # if specific_bot_goal_flag:
            #     specific_bot_goal_flag = False
            #     previous_task_flag = False
            #     break

            for i, b in enumerate(s.swarm):
                if self.specific_bot_goal_flag_array[i]:
                    goal[i] = self.specific_goal_pos[i][self.specific_goal_xy_index[i]]
                    # current_position = [b.x, b.y]
                    if self.specific_bot_goal_flag_array[i]:
                        dx = abs(goal[i][0] - b.x)
                        dy = abs(goal[i][1] - b.y)

                        if dx <= 5 and dy <= 5:
                            # if dist <= 5:
                            self.specific_goal_xy_index[i] = (
                                self.specific_goal_xy_index[i] + 1
                            )
                            print(
                                "specific_goal_xy_index", self.specific_goal_xy_index
                            )
                            if self.specific_goal_xy_index[i] == len(
                                self.specific_goal_pos[i]
                            ):
                                self.specific_bot_goal_flag_array[i] = False
                                self.specific_goal_pos[i] = 0
                                print(
                                    "specific_bot_goal_flag_array",
                                    self.specific_bot_goal_flag_array,
                                    self.specific_goal_pos,
                                )
                        print("self.specific_bot_goal_flag_array",self.specific_bot_goal_flag_array)
                        if all(
                            flag == False
                            for flag in self.specific_bot_goal_flag_array
                        ):
                            print("self.specific_bot_goal_flag = True")
                            break
                        else:
                            if self.specific_bot_goal_flag_array[i]:
                                b.set_goal(goal[i][0], goal[i][1])
                                cmd = cvg.goal_area_cvg(i, b, goal[i])
                                
                                cmd.exec(b)
                        if master_flag:
                            current_position = (b.x * 2, b.y * 2)
                            lat, lon = locatePosition.cartToGeo(
                                self.origin, self.endDistance, current_position
                            )
                            self._goto(i, lat, lon)
            
            if self.stop_swarm and self.stop_swarm._peek_command() == "STOP":
                print("STOP command detected in handle_specific_bot_goal. Stopping the swarm...")
                self.stop_swarm._pop_command()  # Clear the command after processing
                break
                
class StopSwarm:
    def __init__(self):
        self._cmd_queue = []

    def _push_command(self, data):
        self._cmd_queue.append(data)

    def _peek_command(self):
        return self._cmd_queue[0] if self._cmd_queue else None

    def _pop_command(self):
        return self._cmd_queue.pop(0) if self._cmd_queue else None

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Swarm Server Controller")

    parser.add_argument(
        "--server-address",
        type=str,
        default="127.0.0.1",
        help="inital communication ip address",
    )
    parser.add_argument("--sim-enable", action="store_true", help="Enable simulation mode")
    parser.add_argument("--log-path", default="logs", help="Enable Logging Support System")

    args = parser.parse_args()

    log_path = args.log_path
    print("log_path", log_path)
    
    path = define_log_path(log_path)
    
    log(f"SERVER STARTED", LOG_FILE=path)
    
    ip = args.server_address  # "192.168.6.220"
    print("ip", ip)

    if args.sim_enable:
        heartbeat_ip = [ip] * 8
        heartbeat_ip_timeout = [30] * 8
    else:
        heartbeat_ip = [  # modem_ip
            "192.168.6.101",
            "192.168.6.102",
            "192.168.6.103",
            "192.168.6.104",
            "192.168.6.105",
            "192.168.6.106",
            "192.168.6.107",
            "192.168.6.108",
        ]
        heartbeat_ip_timeout = [3] * 8
            
    sock2 = SocketServer(ip, 12008,isBlocking=True)
    
    swarm_controller = SwarmController(heartbeat_ip, heartbeat_ip_timeout)

    while True:
        print("Main loop running...")
        
        try:
            data,add = sock2.monitor()
            print(f"Received message from {add}: {data} in sock2")
            
            if data is None:
                print("No data received. Continuing main loop...")
            elif data.startswith("different"):
                swarm_controller.handle_different_height(data)
                
            elif data.startswith("specificbotgoal"):
                swarm_controller.handle_specific_bot_goal(data)
                
            else:
                print("No recognized command received. Continuing main loop...")
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Error in main loop: {e}")
            log(f"Error in main loop: {e}", LOG_FILE=path)
    
    