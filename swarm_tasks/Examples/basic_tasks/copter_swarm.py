print("Version Swarm Copter V13")
import sys, os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
import swarm_tasks
import time
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
import swarm_tasks.controllers.potential_field as potf
from swarm_tasks.modules.dispersion import disp_field
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules import exploration as exp
from swarm_tasks.tasks import area_coverage as cvg
from math import radians, sin, cos, sqrt, atan2, asin, degrees
from dronekit import connect, VehicleMode, LocationGlobalRelative
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
cwd = os.getcwd()
print("cwd", cwd)
log_path = args.log_path
print("log_path", log_path)
if log_path is not None:
    # remove leading / or \ so it becomes relative
    clean_log_path = log_path.lstrip("/\\")

    LOG_DIR = Path(cwd) / clean_log_path
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    LOG_FILE = LOG_DIR / "swarm_server.log"
    log_path = str(LOG_FILE)

    print("Final log file path:", log_path)


def log(msg, LOG_FILE=None):

    if LOG_FILE is not None:

        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(msg + "\n")
            f.flush()


log(f"SERVER STARTED", LOG_FILE=LOG_FILE)

ip = args.server_address  # "192.168.6.220"
print("ip", ip)

sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address2 = ("", 12008)  # receive from .....rx.py
sock2.bind(server_address2)

sock3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address3 = ("", 12002)  # receive from .....rx.py
sock3.bind(server_address3)

goal_table = []
file_name = ""
master_num = -1
master_flag = False

different_height = [50, 60, 70, 80, 90, 100, 110, 120]

sock2.setblocking(0)


def read_origin(filepath):
    print("Reading YAML from:", filepath)
    with open(filepath) as f:
        data = yaml.safe_load(f)

    origin = data.get("origin")

    # If the origin is provided as a string "(lat, lon)"
    if isinstance(origin, str):
        origin = origin.strip("()")
        lat, lon = origin.split(",")
        origin = (float(lat), float(lon))

    return origin


documents_path = os.path.join(os.path.expanduser("~"), "Documents")
file_name = os.path.join(
    documents_path, "swarm_env", "rectangles.yaml"
)  # Replace 'your_file.yaml' with actual file name
print("read_origin_path", file_name)


disperse_multiple_goals = []
start_multiple_goals = []
return_multiple_goals = []
goal_points = []
agg_goal_point = []
origin = []
removed_uav_homepos_array = []
bot_speed = 3.0

master_num = 0
master_flag = True


origin = read_origin(file_name)
print("Origin!!!!", origin)
nextwaypoint = 0


num_bots = 8
vehicles = []
port_array = [14551, 14552, 14553, 14554, 14555, 14556, 14557, 14558]

port_dict = {
    1: 14551,
    2: 14552,
    3: 14553,
    4: 14554,
    5: 14555,
    6: 14556,
    7: 14557,
    8: 14558,
}

# Print the dictionary to verify
print(port_dict)

pos_array = []
active_bot_count = len(pos_array)
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


goal_path_csv_array = []
goal_path_csv_array_flag = False
skip_wp_flag = False
next_wp = 0


def socket_monitor():
    global index, origin
    global vehicles
    global master_flag, master_num, pos_array, home_pos, uav_home_pos, skip_wp_flag, next_wp
    while 1:
        index, address = sock3.recvfrom(1024)
        print("msg!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", index)
        # decoded_index = index.decode("utf-8")
        # if decoded_index.startswith("master"):
        #     m, master_num = decoded_index.split("-")
        #     print("m,master_num", m, master_num)
        #     msg = "Drone 0 master_num " + str(master_num) + " data received"
        #     if int(master_num) == 0:
        #         if master_flag:
        #             pass
        #         else:
        #             master_flag = True
        #             CHECK_network_connection()
        #             vehicle_connection()
        #             fetch_location()
        #             home_lock()
        #             origin = read_origin(file_name)
        #             s = sim.Simulation(
        #                 uav_home_pos,
        #                 num_bots=len(vehicles),
        #                 env_name=file_name,
        #                 speed=bot_speed,
        #             )
        #             print("Simulation initialized")
        #     else:
        #         if master_flag:
        #             for vehicle in vehicles:
        #                 vehicle.close()

        #         master_flag = False

        #     index = "data"
        #     data = "data"
        #     msg = "master_num " + str(master_num)
        #     print("master_flag", master_flag)

        # if decoded_index.startswith("pos_array"):
        #     message = decoded_index[:9]  # Assuming "home_pos" is 8 characters long
        #     array_data = decoded_index[9:]
        #     print("pos_array", pos_array)
        #     pos_array = json.loads(array_data)
        #     print("pos_array", pos_array)
        #     index = "data"
        #     msg = "UAV 3 connected with " + str(len(pos_array)) + " vehicles"
        #     print("master_flag", master_flag)
        # if decoded_index.startswith("home_pos"):
        #     message = decoded_index[:8]  # Assuming "home_pos" is 8 characters long
        #     home_pos = decoded_index[8:]
        #     print("home_pos", home_pos)
        #     home_pos = json.loads(home_pos)
        #     print("home_pos", home_pos)

        # if decoded_index.startswith("uav_home_pos"):
        #     if master_flag:
        #         pass
        #     else:
        #         message = decoded_index[:12]  # Assuming "home_pos" is 8 characters long
        #         uav_home_pos = decoded_index[12:]
        #         print("uav_home_pos", uav_home_pos)
        #         uav_home_pos = json.loads(uav_home_pos)
        #         print("uav_home_pos", uav_home_pos)


socket_thread = threading.Thread(target=socket_monitor, daemon=True)
socket_thread.start()


def home_lock():
    global vehicles, home_pos_lat_lon, home_pos
    home_pos = []
    home_pos_lat_lon = []
    for i, vehicle in enumerate(vehicles):
        # Wait until vehicle.home_location is valid or timeout to avoid infinite loop
        timeout = time.time() + 30  # 30 seconds timeout
        while not vehicle.home_location:
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not vehicle.home_location:
                print(" Waiting for home position...")
                time.sleep(1)
            if time.time() > timeout:
                print(f"Timeout waiting for home_location for vehicle {i}")
                break

        # Assign home only if available
        home = vehicle.home_location
        if home is None:
            continue  # skip to next vehicle

        x, y = locatePosition.geoToCart(origin, endDistance, [home.lat, home.lon])
        home_pos_lat_lon.append((home.lat, home.lon))
        home_pos.append((x / 2, y / 2))
    return 1


def CHECK_network_connection():
    global heartbeat_ip_timeout, heartbeat_ip
    for i, iter_follower in enumerate(heartbeat_ip_timeout):
        # Use -n 1 for Windows, and suppress output
        response = os.system("ping -n 1 " + heartbeat_ip[i] + " >nul 2>&1")

        if response == 0:
            heartbeat_ip_timeout[i] = 30
        else:  # Link is down.
            print("waiting...")
            linkdown_flag = True
            heartbeat_ip_timeout[i] = 30

    print("heartbeat_ip_timeout", heartbeat_ip_timeout)


def vehicle_connection():
    global vehicles, pos_array, num_bots, heartbeat_ip_timeout
    pos_array = []
    vehicles = []
    num_bots = 0

    try:
        vehicle1 = connect(
            "udpin:{}:14551".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[0],
        )
        print("Drone1")
        vehicles.append(vehicle1)
        pos_array.append(vehicle1._master.target_system)
        num_bots += 1
        msg = "Drone1 Connected"

    except:
        pass
        print("Vehicle 1 is lost")
    try:
        vehicle2 = connect(
            "udpin:{}:14552".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[1],
        )
        print("Drone2")
        num_bots += 1
        vehicles.append(vehicle2)
        pos_array.append(vehicle2._master.target_system)
        msg = "Drone2 Connected"

    except:
        pass
        print("Vehicle 2 is lost")

    try:
        vehicle3 = connect(
            "udpin:{}:14553".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[2],
        )
        print("Drone3")
        num_bots += 1
        vehicles.append(vehicle3)
        pos_array.append(vehicle3._master.target_system)
        msg = "Drone3 Connected"

    except:
        pass
        print("Vehicle 3 is lost")

    try:
        vehicle4 = connect(
            "udpin:{}:14554".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[3],
        )
        print("Drone4")
        num_bots += 1
        vehicles.append(vehicle4)
        pos_array.append(vehicle4._master.target_system)
        msg = "Drone4 Connected"
    except:
        pass
        print("Vehicle 4 is lost")
    try:
        vehicle5 = connect(
            "udpin:{}:14555".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[4],
        )
        print("Drone5")
        num_bots += 1
        vehicles.append(vehicle5)
        pos_array.append(vehicle5._master.target_system)
        msg = "Drone5 Connected"
    except:
        pass
        print("Vehicle 5 is lost")

    try:
        vehicle6 = connect(
            "udpin:{}:14556".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[5],
        )
        print("Drone6")
        num_bots += 1
        vehicles.append(vehicle6)
        pos_array.append(vehicle6._master.target_system)
        msg = "Drone6 Connected"
    except:
        pass
        print("Vehicle 6 is lost")

    try:
        vehicle7 = connect(
            "udpin:{}:14557".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[6],
        )
        print("Drone7")
        num_bots += 1
        vehicles.append(vehicle7)
        pos_array.append(vehicle7._master.target_system)
        msg = "Drone7 Connected"
    except:
        pass
        print("Vehicle 7 is lost")

    try:
        vehicle8 = connect(
            "udpin:{}:14558".format(ip),
            baud=115200,
            heartbeat_timeout=heartbeat_ip_timeout[7],
        )
        print("Drone8")
        num_bots += 1
        vehicles.append(vehicle8)
        pos_array.append(vehicle8._master.target_system)
        msg = "Drone8 Connected"
    except:
        pass
        print("Vehicle 8 is lost")

    print(len(vehicles))


# Track currently active reconnection threads to avoid duplicates
reconnecting_ports = set()

# TODO
def reconnection_worker(index, connection_str, sys_id):
    global vehicles, reconnecting_ports
    try:
        # We use wait_ready=True to ensure the vehicle is fully ready before it's used
        new_v = connect(connection_str, baud=115200, heartbeat_timeout=30)
        new_v.last_reconnect_attempt = 0

        # Atomically swap the old dead vehicle with the new live one
        # Safely update the vehicles list
        vehicles[index] = new_v
        print(
            f"\n[RECONNECT] SUCCESS: Drone {index} (SysID {sys_id}) reconnected and ready."
        )
    except Exception as e:
        print(
            f"\n[RECONNECT] FAILED: Drone {index} (SysID {sys_id}) still offline: {e}"
        )
    finally:
        reconnecting_ports.discard(connection_str)

# TODO
def check_reconnection():
    global vehicles, pos_array, ip, port_dict, reconnecting_ports
    now = time.time()

    # Periodic Status (log every 5 seconds)
    if not hasattr(check_reconnection, "last_status_report"):
        check_reconnection.last_status_report = 0
    if now - check_reconnection.last_status_report > 5:
        check_reconnection.last_status_report = now
        hbs = []
        for i, v in enumerate(vehicles):
            try:
                hb = (
                    round(v.last_heartbeat, 1)
                    if v.last_heartbeat is not None
                    else "DEAD"
                )
            except:
                hb = "DEAD"
            hbs.append(f"D{i}:{hb}s")
        # print(f"[RECONNECT MONITOR] {time.strftime('%H:%M:%S')} | Heartbeats: " + " | ".join(hbs))

    for i in range(len(vehicles)):
        v = vehicles[i]
        is_dead = False
        try:
            hb = v.last_heartbeat
            # DroneKit times out at 30s. Trigger background fix at 28s.
            if hb is None or hb >= 28:
                is_dead = True
        except:
            is_dead = True

        if is_dead:
            sys_id = pos_array[i]
            port = port_dict.get(sys_id)
            if not port:
                continue

            connection_str = f"udpin:{ip}:{port}"
            last_attempt = getattr(v, "last_reconnect_attempt", 0)

            # Start reconnection in background if not already trying
            if (now - last_attempt > 10) and (connection_str not in reconnecting_ports):
                v.last_reconnect_attempt = now
                reconnecting_ports.add(connection_str)

                print(
                    f"CRITICAL: Drone {i} (SysID {sys_id}) link lost. Starting background recovery..."
                )
                try:
                    v.close()
                except:
                    pass

                t = threading.Thread(
                    target=reconnection_worker,
                    args=(i, connection_str, sys_id),
                    daemon=True,
                )
                t.start()


def calculate_drones_needed(remaining_points, points_per_drone):
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


def allocate_drones(total_points, covered_points, total_drones):
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
        calculate_drones_needed(points, points_per_drone[i])
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


count = 0
endDistance = 500000
home_pos = []
home_pos_lat_lon = []
uav_home_pos = []
current_lat_lon = []
height_difference = 5
home_flag = False
home_flag1 = False
search_flag = False
home_goto_flag = False
lost_vehicle_num = 0
aggregate_flag = False
disperse_flag = False
closing_flag = False
landing_flag = False
# Iterate over the list of vehicles
robots = [(0, 0)] * 8

heartbeat = [0] * num_bots

vehicle_uav_heartbeat_flag = False

all_uav_csv_grid_array = [0] * active_bot_count
robot_positions = [([0, 0]) for _ in range(8)]
print("origin#########", origin)

sleep_times = {
    8: 0.0000001,
    7: 0.0000001,
    6: 0.0000001,
    5: 0.0000001,
    4: 0.0000001,
    3: 0.0000001,
    2: 0.0000001,
    1: 0.0000001,
}


def fetch_location():
    global vehicles, home_pos_lat_lon, home_pos, uav_home_pos
    global robots
    uav_home_pos = []
    current_lat_lon = []
    print("#####")
    if master_flag:
        try:
            home_lock()
        except:
            for i, vehicle in enumerate(vehicles):
                lat = vehicle.location.global_relative_frame.lat
                lon = vehicle.location.global_relative_frame.lon
                # print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
                home_pos_lat_lon.append((lat, lon))
                x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                # print("x,y",x/2,y/2)
                home_pos.append((x / 2, y / 2))
                if i < len(robots):
                    robots[i] = (x / 2, y / 2)
                print("home_pos", home_pos)
        try:
            for i, vehicle in enumerate(vehicles):
                lat = vehicle.location.global_relative_frame.lat
                lon = vehicle.location.global_relative_frame.lon
                current_lat_lon.append((lat, lon))
                x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                print("x,y", x / 2, y / 2)
                uav_home_pos.append((x / 2, y / 2))
        except:
            pass


if master_flag:
    CHECK_network_connection()
    vehicle_connection()
    while True:
        all_armed = [False] * len(vehicles)  # Assume all vehicles are armed initially
        for i, vehicle in enumerate(vehicles):
            if vehicle.armed:
                all_armed[i] = True  # Set the flag to False
        if all(all_armed):
            fetch_location()
            break
        time.sleep(0.1)


csv_cache = {}


def read_specific_line(csv_file_path, line_number):
    global csv_cache
    goal = []

    # Check if we have already read this entire CSV into memory
    if csv_file_path not in csv_cache:
        try:
            with open(csv_file_path, "rt") as file:
                reader = csv.reader(file)
                # Read all lines into cache at once: [[float(x), float(y)], ...]
                csv_cache[csv_file_path] = [
                    [float(row[0]), float(row[1])] for row in reader if row
                ]
        except Exception as e:
            print(f"Error caching CSV {csv_file_path}: {e}")
            return []

    # Safely return the requested line from the cache (0-indexed)
    try:
        # The original code used a 0-based index but skipped `line_number` rows, effectively making it 0-indexed memory access
        target_line = csv_cache[csv_file_path][line_number]
        goal.append((target_line[0], target_line[1]))
    except IndexError:
        print(f"Error: Line {line_number} not found in {csv_file_path}")

    return goal


def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checcks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    time.sleep(3)
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


data = ""
index = 0
pop_flag_arr = [1] * num_bots
pop_flag = False
specific_bot_goal_flag = False
pop_bot_index = None
goal_bot_num = None
start_flag = False
start_return_csv_flag = False
radius_of_earth = 6378100.0  # in meters
uav_home_flag = False
remove_flag = False
group_goal_flag = False
circle_formation_count = 0
uav_removed = True
grid_path_array = [1] * num_bots
remove_bot_flag = False
remove_bot_index = 0
search_step = 1
percentage = 0
removed_uav_grid = []
removed_grid_path_length = []
removed_numlines = []
mid_mission_data_cache = {}

removed_grid_path_array = [0] * len(pos_array)
removed_grid_path_array_start_val = [0] * len(pos_array)
checkall_removed_grid_path_array_start_val = [0] * len(pos_array)
active_bot_count = len(pos_array)
removed_grid_filename = [0] * num_bots
removed_grid_path_array_flag = False
remove_bot_flag = False
remove_bot_index = []
remove_bot_array = []
grid_completed_bot = [-1] * num_bots
uncovered_area_filename = []
uncovered_area_points = []
grid_completed_bot = [-1] * num_bots
remove_bot_num_array = []
group_split_goal_pos = [0] * num_bots
group_split_flag_array = [False] * num_bots
specific_goal_pos = [0] * num_bots
specific_bot_goal_flag_array = [False] * num_bots
specific_goal_xy_index = [0] * num_bots
group_split_flag = False
search_flag_val = 0
split_flag_val = 0
split_flag = False
previous_task = b""
previous_task_flag = False
include_uav_flag = False
include_uav_index = []
search_loop_running = False
# Initialize Simulation and GUI
while True:
    if uav_home_pos != []:
        print("num_bots", num_bots, uav_home_pos)
        origin = read_origin(file_name)
        s = sim.Simulation(
            uav_home_pos, num_bots=len(pos_array), env_name=file_name, speed=bot_speed
        )
        break
    else:
        time.sleep(0.1)

# TODO
def home_monitor_thread():
    global home_pos, pos_array
    while True:
        try:
            # Only refresh if home_pos is missing or out of sync
            if not home_pos or len(pos_array) != len(home_pos):
                print("[Home Monitor] Updating home positions...")
                home_lock()
        except Exception as e:
            print("[Home Monitor] Error while updating home positions:", e)
        time.sleep(5)  # Check every 5 seconds (tune as needed)

# TODO
home_monitor_thread = threading.Thread(target=home_monitor_thread, daemon=True)
home_monitor_thread.daemon = True
home_monitor_thread.start()


vehicles_thread = []
while 1:
    check_reconnection()
    if master_flag:
        num_bots = len(vehicles)
    else:
        num_bots = len(pos_array)
    try:
        data, address = sock2.recvfrom(1050)
        print("!!msg.......", data)

        try:
            origin = read_origin(file_name)
        except Exception as e:
            print("Error reading updated origin:", e)

        if data.startswith(b"takeoff"):
            decoded_index = data.decode("utf-8")
            print("decoded_index", decoded_index)
            data, takeoff_height = decoded_index.split(",")
            print("data,takeoff_height", data, takeoff_height)
            for i, vehicle in enumerate(vehicles):
                print(i)
                thread = threading.Thread(
                    target=arm_and_takeoff, args=(vehicle, int(takeoff_height))
                )
                vehicles_thread.append(thread)
                thread.start()

            for thread in vehicles_thread:
                thread.join()
            home_pos = []
            for i, vehicle in enumerate(vehicles):
                lat = vehicle.location.global_relative_frame.lat
                lon = vehicle.location.global_relative_frame.lon
                # print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
                home_pos_lat_lon.append((lat, lon))
                x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                # print("x,y",x/2,y/2)
                home_pos.append((x / 2, y / 2))
                if i < len(robots):
                    robots[i] = (x / 2, y / 2)
                msg = ",".join([f"{robot[0]},{robot[1]}" for robot in robots])

        if data.startswith(b"different"):
            decoded_index = data.decode(
                "utf-8"
            )  # Assuming utf-8 encoding, adjust if needed
            data1, height, step = decoded_index.split(",")

            height = int(height)
            step = int(step)

            height_difference = step

            different_height = [height + step * i for i in range(num_bots)]

            print("different_height", different_height, height_difference)
            alt_count = [0] * num_bots
            print("alt_count", alt_count)
            alt = [0] * num_bots
            diff_height_flag = False
            alt_count1 = 0
            while True:
                if diff_height_flag:
                    diff_height_flag = False
                    break
                for i, b in enumerate(s.swarm):
                    cmd = potf.velocity(
                        b.get_position(),
                        b.sim,
                        weights=potf.field_weights,
                        order=1,
                        max_dist=10,
                    )
                    # --- Proximity Monitor ---
                    for other_idx, other_b in enumerate(s.swarm):
                        if other_idx > i:  # Only check each pair once
                            dist = math.sqrt(
                                (b.x - other_b.x) ** 2 + (b.y - other_b.y) ** 2
                            )
                            if dist < 9.0:
                                print(
                                    # f"[ALERT] Drone {i+1} and {other_idx+1} are {dist:.2f}m apart!"
                                )

                    cmd.exec(b)
                    if master_flag:
                        value = [b.x * 2, b.y * 2]
                        lat, lon = locatePosition.cartToGeo(origin, endDistance, value)
                        point1 = LocationGlobalRelative(lat, lon, different_height[i])
                        vehicles[i].simple_goto(point1)
                        alt[i] = vehicles[i].location.global_relative_frame.alt
                        if (
                            different_height[i] - 1.5
                            <= alt[i]
                            <= different_height[i] + 1.5
                        ):
                            alt_count[i] = 1
                            if all(count == 1 for count in alt_count):
                                print("Reached target altitude")
                                index = "data"
                                data = "data"
                                diff_height_flag = True
                                break
                    else:
                        data = "data"
                        diff_height_flag = True
                        break

                if index == b"stop":
                    index = "data"
                    data = "index"
                    break
                time.sleep(0.05)
            data = previous_task
            if previous_task != b"":
                previous_task_flag = True
            else:
                previous_task_flag = False
            print("!!!!!!!", data, previous_task_flag)

        if (data.startswith(b"remove")) or (remove_flag):
            decoded_index = data.decode(
                "utf-8"
            )  # Assuming utf-8 encoding, adjust if needed
            f, remove_bot_num = decoded_index.split(",")
            print(f, remove_bot_num)
            print("remove_bot_num", remove_bot_num, pos_array)
            remove_bot_num_array.append(int(remove_bot_num))

            remove_bot_flag = True
            print("msg", index)
            for l in range(0, len(pos_array)):
                if int(remove_bot_num) == pos_array[l]:
                    pop_bot_index = l
                    print(l)
                    break
            if pop_bot_index != None:
                remove_bot_index = pop_bot_index
                remove_bot_num = pos_array[pop_bot_index]
                pos_array.pop(pop_bot_index)
                remove_bot_array.append((pop_bot_index, int(remove_bot_num)))
                print(pos_array)
                print("vehicles[pop_bot_index]", vehicles[pop_bot_index])
                v = vehicles[pop_bot_index]
                v.close()
                print("vehicles!!!", vehicles)
                vehicles.pop(pop_bot_index)
                s.remove_bot(pop_bot_index)
                home_pos.pop(pop_bot_index)
                print("home_pos", home_pos)
                different_height.pop(pop_bot_index)
                pop_flag_arr.pop(pop_bot_index)
                print(num_bots, "LLLLLL")
                print("!!!!!!!!!!!!pop_flag_arr!!!!!!!!!!!", pop_flag_arr, vehicles)
                print("pop index", pop_bot_index)
                print(f"len(pos_array): {len(pos_array)}")
                print(f"len(all_uav_csv_grid_array): {len(all_uav_csv_grid_array)}")
                print(f"len(grid_path_array): {len(grid_path_array)}")
                print(f"len(csv_file_paths): {len(csv_file_paths)}")
                print(f"len(pop_flag_arr): {len(pop_flag_arr)}")
                print(f"len(s.swarm): {len(s.swarm)}")
                print(f"len(vehicles): {len(vehicles)}")
                specific_goal_pos.pop(pop_bot_index)
                specific_bot_goal_flag_array.pop(pop_bot_index)
                specific_goal_xy_index.pop(pop_bot_index)
                uav_home_pos = []
                for vehicle in vehicles:
                    lat = vehicle.location.global_relative_frame.lat
                    lon = vehicle.location.global_relative_frame.lon
                    x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                    uav_home_pos.append((x / 2, y / 2))
                print("uav_home_pos", uav_home_pos)
                remove_flag = False
                uav_removed = True
                pop_bot_index = None
                num_bots = len(pos_array)
                print("num_bots", num_bots)
            else:
                print("Not found")
            data = previous_task
            if previous_task != b"":
                previous_task_flag = True
            else:
                previous_task_flag = False

        if data.startswith(b"add"):
            try:
                decoded_index = data.decode("utf-8")
                f, sys_id = decoded_index.split(",")
                if int(sys_id) not in pos_array:
                    # print("sys id not found in pos_array")
                    if int(sys_id) in port_dict:
                        connection_str = f"udpin:{ip}:{port_dict[int(sys_id)]}"
                        print(
                            f"Connection string for sys_id {int(sys_id)}: {connection_str}"
                        )
                        vehicle = connect(
                            connection_str,
                            baud=115200,
                            heartbeat_timeout=30,
                            wait_ready=True,
                        )

                        lat = vehicle.location.global_relative_frame.lat
                        lon = vehicle.location.global_relative_frame.lon
                        timeout = time.time() + 15
                        while (lat is None or lon is None) and time.time() < timeout:
                            print("Waiting for GPS for added drone...")
                            time.sleep(0.5)
                            lat = vehicle.location.global_relative_frame.lat
                            lon = vehicle.location.global_relative_frame.lon

                        if lat is None or lon is None:
                            vehicle.close()
                            raise Exception("Could not get GPS for added drone")

                        vehicles.append(vehicle)
                        num_bots = num_bots + 1
                        print("num_bots", num_bots)

                        x, y = locatePosition.geoToCart(
                            origin, endDistance, [float(lat), float(lon)]
                        )
                        s.add_bot(len(pos_array), (x / 2, y / 2))
                        pos_array.append(int(sys_id))

                        # Initialize arrays with defaults to maintain consistency
                        specific_goal_pos.append(0)
                        specific_bot_goal_flag_array.append(False)
                        specific_goal_xy_index.append(0)
                        pop_flag_arr.append(1)

                        # Handle different_height safely
                        if not different_height:
                            different_height.append(50)  # Default start height
                        else:
                            # Use existing num_bots which was incremented at 3631
                            # but safer to use different_height[-1]
                            different_height.append(
                                different_height[-1] + height_difference
                            )

                        num_bots = len(pos_array)
                        home_lock()
                        print("vehicles", vehicles, home_pos)
                        print("remove_bot_num_array", remove_bot_num_array)
                        print(
                            "sys_id",
                            sys_id,
                            "remove_bot_num_array",
                            remove_bot_num_array,
                            "remove_bot_array",
                            remove_bot_array,
                            previous_task,
                        )
                        if int(sys_id) in remove_bot_num_array:
                            remove_bot_flag = False
                            previous_task_flag = False
                            remove_bot_num_array.remove(int(sys_id))
                        if previous_task in [
                            b"search",
                            b"split",
                            b"navigate",
                            b"specificsplit",
                        ]:
                            include_uav_flag = True
                            previous_task_flag = True
                            data = previous_task
                    else:
                        print(f"sys_id {int(sys_id)} not found in port_dict.")

                else:
                    print("sys id already in pos_array")
                    # try:
                    #     system_id = [int(id) for id in pos_array].index(int(sys_id))
                    #     print("system_id", system_id)
                    #     if int(sys_id) in port_dict:
                    #         connection_str = f"udpin:{ip}:{port_dict[int(sys_id)]}"
                    #         print(
                    #             f"Connection string for sys_id {int(sys_id)}: {connection_str}"
                    #         )
                    #     else:
                    #         print(f"sys_id {int(sys_id)} not found in port_dict.")

                    #     vehicles[system_id] = connect(
                    #         connection_str, baud=115200, heartbeat_timeout=30
                    #     )
                    #     print("vehicles[system_id]", vehicles[system_id], vehicles)
                    # except Exception as e:
                    #     print("Exception reconnecting to existing vehicle:", e)
            except Exception as e:
                pass
                print("System array not found ", e)

        if data.startswith(b"specificbotgoal"):
            index = "data"
            try:
                if not previous_task_flag:
                    decoded_index = data.decode(
                        "utf-8"
                    )  # Assuming utf-8 encoding, adjust if needed
                    msg_parts = decoded_index.split("_")
                    f = msg_parts[0]
                    uav_raw = msg_parts[1].strip().replace("'", '"')
                    uav_list = json.loads(uav_raw)
                    print("uav_list", uav_list)
                    goal_array = msg_parts[2]  # All other coordinates
                    goal_latlon = json.loads(goal_array)
                    goal_xy = []
                    bot_reached = [0] * len(pos_array)
                    for x in goal_latlon:
                        x, y = locatePosition.geoToCart(
                            origin, endDistance, [x[1], x[0]]
                        )
                        goal_xy.append((x / 2, y / 2))
                    
                    goal = [0] * len(pos_array)
                    for uav_id in uav_list:
                        # Find matching bot index
                        if int(uav_id) in pos_array:
                            bot_index = pos_array.index(int(uav_id))
                            specific_goal_pos[bot_index] = goal_xy
                            specific_bot_goal_flag_array[bot_index] = True
                            specific_goal_xy_index[bot_index] = 0

                        else:
                            print("UAV", uav_id, "not found in pos_array")
                            continue
                    print(
                        "specific_bot_goal_flag_array",
                        specific_bot_goal_flag_array,
                        specific_goal_pos,
                    )
                if master_flag:
                    uav_home_pos = []
                    index = "data"
                    for vehicle in vehicles:
                        lat = vehicle.location.global_relative_frame.lat
                        lon = vehicle.location.global_relative_frame.lon
                        x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                        uav_home_pos.append((x / 2, y / 2))
                    print("uav_home_pos", uav_home_pos)
                    origin = read_origin(file_name)
                    s = sim.Simulation(
                        uav_home_pos,
                        num_bots=len(pos_array),
                        env_name=file_name,
                        speed=bot_speed,
                    )
                while 1:
                    time.sleep(sleep_times.get(num_bots, 0.1))
                    if specific_bot_goal_flag:
                        specific_bot_goal_flag = False
                        previous_task_flag = False
                        break

                    for i, b in enumerate(s.swarm):
                        if specific_bot_goal_flag_array[i]:
                            goal[i] = specific_goal_pos[i][specific_goal_xy_index[i]]
                            # current_position = [b.x, b.y]
                            if specific_bot_goal_flag_array[i]:
                                dx = abs(goal[i][0] - b.x)
                                dy = abs(goal[i][1] - b.y)

                                if dx <= 5 and dy <= 5:
                                    # if dist <= 5:
                                    specific_goal_xy_index[i] = (
                                        specific_goal_xy_index[i] + 1
                                    )
                                    print(
                                        "specific_goal_xy_index", specific_goal_xy_index
                                    )
                                    if specific_goal_xy_index[i] == len(
                                        specific_goal_pos[i]
                                    ):
                                        specific_bot_goal_flag_array[i] = False
                                        specific_goal_pos[i] = 0
                                        print(
                                            "specific_bot_goal_flag_array",
                                            specific_bot_goal_flag_array,
                                            specific_goal_pos,
                                        )
                                if all(
                                    flag == False
                                    for flag in specific_bot_goal_flag_array
                                ):
                                    specific_bot_goal_flag = True
                                    break
                                else:
                                    if specific_bot_goal_flag_array[i]:
                                        b.set_goal(goal[i][0], goal[i][1])
                                        cmd = cvg.goal_area_cvg(i, b, goal[i])
                                        # --- Proximity Monitor ---
                                        for other_idx, other_b in enumerate(s.swarm):
                                            if (
                                                other_idx > i
                                            ):  # Only check each pair once
                                                dist = math.sqrt(
                                                    (b.x - other_b.x) ** 2
                                                    + (b.y - other_b.y) ** 2
                                                )
                                                if dist < 5.0:
                                                    print(
                                                        f"[ALERT] Drone {i+1} and {other_idx+1} are {dist:.2f}m apart!"
                                                    )
                                        cmd.exec(b)
                                if master_flag:
                                    current_position = (b.x * 2, b.y * 2)
                                    lat, lon = locatePosition.cartToGeo(
                                        origin, endDistance, current_position
                                    )
                                    point1 = LocationGlobalRelative(
                                        lat, lon, different_height[i]
                                    )
                                    vehicles[i].simple_goto(point1)

                    if index == b"stop":
                        specific_bot_goal_flag = False
                        previous_task_flag = False
                        previous_task = b"specificbotgoal"
                        break
            except Exception as e:
                print("exceptiiiooonnn", e)
                pass

        if data.startswith(b"goal"):
            index = "data"
            try:
                if previous_task == b"specificbotgoal":
                    specific_bot_goal_flag_array = [False] * num_bots

                if not previous_task_flag:
                    decoded_index = data.decode(
                        "utf-8"
                    )  # Assuming utf-8 encoding, adjust if needed
                    msg_parts = decoded_index.split("_")
                    print("msg_parts", msg_parts, len(msg_parts))
                    f = msg_parts[0]  # First coordinate pair
                    goal_array = msg_parts[1]  # All other coordinates
                    goal_latlon = json.loads(goal_array)
                    goal_xy = []
                    bot_reached = [0] * num_bots
                    for x in goal_latlon:
                        x, y = locatePosition.geoToCart(
                            origin, endDistance, [x[0], x[1]]
                        )
                        goal_xy.append((x / 2, y / 2))
                        print(goal_xy, "goal_xy")
                    print(goal_xy, "goal")
                    goal_xy_index = 0
                    if master_flag:
                        uav_home_pos = []
                        index = "data"
                        for vehicle in vehicles:
                            lat = vehicle.location.global_relative_frame.lat
                            lon = vehicle.location.global_relative_frame.lon
                            x, y = locatePosition.geoToCart(
                                origin, endDistance, [lat, lon]
                            )
                            uav_home_pos.append((x / 2, y / 2))
                        print("uav_home_pos", uav_home_pos)
                        origin = read_origin(file_name)
                        s = sim.Simulation(
                            uav_home_pos,
                            num_bots=len(pos_array),
                            env_name=file_name,
                            speed=bot_speed,
                        )

                previous_task_flag = False
                while 1:
                    time.sleep(sleep_times.get(num_bots, 0.1))
                    if group_goal_flag:
                        group_goal_flag = False
                        previous_task_flag = False
                        break
                    goal_position = goal_xy[goal_xy_index]
                    for i, b in enumerate(s.swarm):
                        current_position = [b.x, b.y]
                        dx = abs(goal_position[0] - current_position[0])
                        dy = abs(goal_position[1] - current_position[1])

                        if dx <= 5 and dy <= 5:
                            bot_reached[i] = 1
                            if (
                                any(element == 1 for element in bot_reached)
                                and goal_xy_index != len(goal_xy) - 1
                            ):
                                print("One reached", bot_reached)
                                bot_reached = [0] * num_bots
                                if goal_xy_index == len(goal_xy) - 1:
                                    print("group_goal_flag", group_goal_flag)
                                    group_goal_flag = True
                                    break
                                else:
                                    goal_xy_index += 1
                            elif (
                                all(element == 1 for element in bot_reached)
                                and goal_xy_index == len(goal_xy) - 1
                            ):
                                bot_reached = [0] * num_bots
                                if goal_xy_index == len(goal_xy) - 1:
                                    print("group_goal_flag", group_goal_flag)
                                    group_goal_flag = True
                                    break
                                else:
                                    goal_xy_index += 1
                        else:
                            b.set_goal(goal_position[0], goal_position[1])
                            cmd = cvg.goal_area_cvg(i, b, goal_position)
                            # --- Proximity Monitor ---
                            for other_idx, other_b in enumerate(s.swarm):
                                if other_idx > i:  # Only check each pair once
                                    dist = math.sqrt(
                                        (b.x - other_b.x) ** 2 + (b.y - other_b.y) ** 2
                                    )
                                    if dist < 5.0:
                                        print(
                                            f"[ALERT] Drone {i+1} and {other_idx+1} are {dist:.2f}m apart!"
                                        )
                            cmd.exec(b)
                        if master_flag:
                            current_position = (b.x * 2, b.y * 2)
                            lat, lon = locatePosition.cartToGeo(
                                origin, endDistance, current_position
                            )
                            point1 = LocationGlobalRelative(
                                lat, lon, different_height[i]
                            )
                            vehicles[i].simple_goto(point1)

                    if index == b"stop":
                        print("Data", data)
                        group_goal_flag = False
                        previous_task = b"goal"
                        previous_task_flag = False
                        break
            except Exception as e:
                print("exception", e)
                pass

        if (data.startswith(b"navigate")) or (start_flag):
            print("data", data)
            if previous_task == b"specificbotgoal":
                specific_bot_goal_flag_array = [False] * num_bots

            if not previous_task_flag:
                decoded_index = data.decode(
                    "utf-8"
                )  # Assuming utf-8 encoding, adjust if needed
                f, center_lat, center_lon, num_uavs, grid_space, coverage_area = (
                    decoded_index.split(",")
                )
                curve = NavigationGridGenerator(
                    origin,
                    float(center_lat),
                    float(center_lon),
                    int(num_uavs),
                    int(grid_space),
                    int(coverage_area),
                )
                path = curve.navigate_grid()
                multiple_goals = path
                if master_flag:
                    start_flag = True
                    uav_home_pos = []
                    index = "data"
                    for vehicle in vehicles:
                        lat = vehicle.location.global_relative_frame.lat
                        lon = vehicle.location.global_relative_frame.lon
                        x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                        uav_home_pos.append((x / 2, y / 2))
                    origin = read_origin(file_name)
                    s = sim.Simulation(
                        uav_home_pos,
                        num_bots=len(pos_array),
                        env_name=file_name,
                        speed=bot_speed,
                    )
            previous_task_flag = False
            index = "data"

            for b in s.swarm:
                search_flag = False
                all_bot_reach_flag = False
                # bot_array = [0] * num_bots
                ind = 0
                while 1:
                    if not start_flag:
                        start_flag = False
                        break
                    time.sleep(sleep_times.get(num_bots))
                    for i, b in enumerate(s.swarm):
                        current_position = [b.x, b.y]
                        goal = multiple_goals[0][ind]
                        x, y = locatePosition.geoToCart(origin, endDistance, goal)
                        goal = (x / 2, y / 2)
                        cmd = cvg.goal_area_cvg(i, b, goal)
                        # --- Proximity Monitor ---
                        for other_idx, other_b in enumerate(s.swarm):
                            if other_idx > i:  # Only check each pair once
                                dist = math.sqrt(
                                    (b.x - other_b.x) ** 2 + (b.y - other_b.y) ** 2
                                )
                                if dist < 5.0:
                                    print(
                                        # f"[ALERT] Drone {i+1} and {other_idx+1} are {dist:.2f}m apart!"
                                    )
                        cmd.exec(b)
                        dx = abs(goal[0] - current_position[0])
                        dy = abs(goal[1] - current_position[1])
                        if dx <= 3 and dy <= 3:
                            # bot_array[i] = ind
                            ind += 1
                            print("Bot reached goal", i, len(multiple_goals[0]))
                            if ind >= len(multiple_goals[0]):
                                print("All reached goal")
                                start_flag = False
                                log("MISSION COMPLETED - navigation", LOG_FILE=LOG_FILE)
                                break

                        if master_flag:
                            current_position = [b.x * 2, b.y * 2]
                            lat, lon = locatePosition.cartToGeo(
                                origin, endDistance, current_position
                            )
                            point1 = LocationGlobalRelative(
                                lat, lon, different_height[i]
                            )
                            vehicles[i].simple_goto(point1)

                    if index == b"stop":
                        print(
                            "start_flag",
                            start_flag,
                        )
                        start_flag = False
                        previous_task = b"navigate"
                        previous_task_flag = False
                        break

        if (data.startswith(b"disperse")) or (disperse_flag):
            disperse_goal = []
            index = "data"
            print("Disperse!!!!!")
            disperse_flag = True
            if previous_task == b"specificbotgoal":
                specific_bot_goal_flag_array = [False] * num_bots

            if data.startswith(b"dispersegoal"):
                decoded_index = data.decode("utf-8")
                f, disperse_latlon = decoded_index.split(",")
                disperse_latlon = json.loads(disperse_goal)
                disperse_goal_index = 0
                disperse_goal = []
                for x in disperse_latlon:
                    x, y = locatePosition.geoToCart(origin, endDistance, [x[0], x[1]])
                    disperse_goal.append((x / 2, y / 2))
                    print(disperse_goal, "disperse_goal")
                print(disperse_goal, "goal")
            if master_flag:
                index = "data"
                uav_home_pos = []
                for vehicle in vehicles:
                    lat = vehicle.location.global_relative_frame.lat
                    lon = vehicle.location.global_relative_frame.lon
                    x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                    uav_home_pos.append((x / 2, y / 2))
                origin = read_origin(file_name)
                s = sim.Simulation(
                    uav_home_pos, num_bots=num_bots, env_name=file_name, speed=bot_speed
                )

            disperse_start_time = time.time()
            print("disperse_start_time", disperse_start_time)
            while True:
                time.sleep(sleep_times.get(num_bots, 0.1))
                if not disperse_flag:
                    disperse_flag = False
                    print("disperse_flag", disperse_flag)
                    break
                try:
                    data, address = sock2.recvfrom(1024)
                    decoded_index = data.decode(
                        "utf-8"
                    )  # Assuming utf-8 encoding, adjust if needed
                    if (data == b"search") or search_flag:
                        search_flag = True
                        break
                except:
                    for i, b in enumerate(s.swarm):
                        # print("disperse_start_time", disperse_start_time)
                        if disperse_goal == []:
                            cmd = base_control.exp_control(b)
                            cmd += disp_field(b) * 15
                            cmd += base_control.exp_obstacle_avoidance(b) * 30
                        else:
                            current_position = [b.x, b.y]
                            cmd = base_control.base_control(
                                i, b, disperse_goal[disperse_goal_index]
                            )
                            cmd = disp_field(b) * 2
                            cmd += base_control.obstacle_avoidance(
                                i, b, disperse_goal[disperse_goal_index]
                            )
                            dx = abs(
                                disperse_goal[disperse_goal_index][0]
                                - current_position[0]
                            )
                            dy = abs(
                                disperse_goal[disperse_goal_index][1]
                                - current_position[1]
                            )
                            if dx <= 1 and dy <= 1:
                                new_length_arr[i] = new_length_arr[i] - 1
                                disperse_bot_goal[i] = 1

                            if all(goal == 1 for goal in disperse_bot_goal):
                                disperse_flag = False
                                search_flag = True
                                print(search_flag, "search_flag")
                                break
                        elapsed_time = time.time() - disperse_start_time
                        print("Disperse elapsed_time", elapsed_time)
                        # --- Proximity Monitor ---
                        for other_idx, other_b in enumerate(s.swarm):
                            if other_idx > i:  # Only check each pair once
                                dist = math.sqrt(
                                    (b.x - other_b.x) ** 2 + (b.y - other_b.y) ** 2
                                )
                                if dist < 5.0:
                                    print(
                                        # f"[ALERT] Drone {i+1} and {other_idx+1} are {dist:.2f}m apart!"
                                    )
                        cmd.exec(b)
                        if master_flag:
                            current_position = [b.x * 2, b.y * 2]
                            lat, lon = locatePosition.cartToGeo(
                                origin, endDistance, current_position
                            )
                            point1 = LocationGlobalRelative(
                                lat, lon, different_height[i]
                            )
                            vehicles[i].simple_goto(point1)
                    if (disperse_goal == []) and (elapsed_time > 10):
                        disperse_flag = False
                        search_flag = True
                        break
                    if (index == b"search") or (search_flag):
                        disperse_flag = False
                        search_flag = True
                        break

                    if index == b"stop":
                        print("Disperse Stoped!!!")
                        disperse_flag = False
                        index = "data"

        if (data.startswith(b"search")) or (search_flag):
            print("data,previous_task_flag", data, previous_task_flag)
            index = "data"
            search_flag = True
            if previous_task == b"specificbotgoal":
                specific_bot_goal_flag_array = [False] * num_bots
            if not previous_task_flag:
                decoded_index = data.decode(
                    "utf-8"
                )  # Assuming utf-8 encoding, adjust if needed
                print("decoded_index", decoded_index)

                if decoded_index.startswith("searchpolygon_"):
                    print("search polygon entered")
                    obstacles_latlon = []
                    rotation_angle = 90
                    f, polygon_latlon_array, num_uavs, grid_spacing, uid = (
                        decoded_index.split("_")
                    )
                    print(
                        "f,polygon_latlon_array,num_uavs,grid_spacing",
                        f,
                        polygon_latlon_array,
                        num_uavs,
                        grid_spacing,
                    )
                    polygon_latlon = json.loads(polygon_latlon_array)
                    print(
                        f"Initializing PolygonSearchGrid for {len(pos_array)} drones..."
                    )
                    planner = PolygonSearchGrid(
                        polygon_latlon=polygon_latlon,
                        origin_gps=origin,
                        endDistance=endDistance,
                        drone_list=pos_array,
                        grid_spacing=int(grid_spacing),
                        rotation_angle=rotation_angle,
                        obstacles_latlon=obstacles_latlon,
                    )
                    planner.generate_paths()
                    planner.save_paths()
                    # Clear cache when new search paths are generated
                    csv_cache.clear()
                    print("PolygonSearchGrid: Paths generated and saved successfully.")
                    print("num_uavs from command:", num_uavs)
                    print("len(pos_array):", len(pos_array))
                    print("active_bot_count:", active_bot_count)

                else:
                    (
                        f,
                        center_lat,
                        center_lon,
                        num_uavs,
                        grid_space,
                        coverage_area,
                        uid,
                    ) = decoded_index.split(",")
                    curve = SearchGridGenerator(
                        origin,
                        float(center_lat),
                        float(center_lon),
                        pos_array,
                        int(grid_space),
                        int(coverage_area),
                    )
                    val = curve.generate_grids()
                    # Clear cache so drones read the newly generated grids
                    csv_cache.clear()
                    print(num_uavs, "num_uavs")
                search_step = 1
                active_bot_count = len(pos_array)
                all_uav_csv_grid_array = [0] * active_bot_count
                # FIX: Use len(pos_array) not active_bot_count, which may be stale
                # if a drone was removed before this fresh search start.
                pop_flag_arr = [1] * len(pos_array)
                if master_flag:
                    uav_home_pos = []
                    for vehicle in vehicles:
                        lat = vehicle.location.global_relative_frame.lat
                        lon = vehicle.location.global_relative_frame.lon
                        x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                        uav_home_pos.append((x / 2, y / 2))
                    origin = read_origin(file_name)
                    s = sim.Simulation(
                        uav_home_pos,
                        num_bots=num_bots,
                        env_name=file_name,
                        speed=bot_speed,
                    )

                active_bot_count = len(s.swarm)

                print("Search Started")
                search_flag_val = 0
                f = ""
                # num_lines = [0] * active_bot_count
                goal_position = []
                cwd = os.getcwd()
                print("search_flag_val", search_flag_val)
                grid_path_array = [0] * len(pos_array)
                all_uav_csv_grid_array = [0] * len(pos_array)
                if search_flag_val == 0:
                    search_flag_val += 1
                    # csv_file_paths = [None] * num_bots
                    # for i in range(1, len(pos_array) + 1):
                    #     path = os.path.join(cwd, "searchgrid", f"d{i}.csv")
                    #     csv_file_paths[i - 1] = path
                    #     reader = csv.reader(open(csv_file_paths[i - 1]))
                    csv_file_paths = [None] * len(pos_array)
                    num_lines = [0] * len(pos_array)

                    for i, e in enumerate(pos_array):
                        print("Checking CSV for drone:", e)
                        path = os.path.join(cwd, "searchgrid", f"d{e}.csv")
                        print("Checking:", path)

                        if not os.path.exists(path):
                            print(
                                f"WARNING: Grid missing for drone {e}, initializing as idle"
                            )
                            csv_file_paths[i] = None
                            num_lines[i] = 0
                            continue

                        csv_file_paths[i] = path

                        with open(path) as f_in:
                            reader = csv.reader(f_in)
                            num_lines[i] = len(list(reader))
                # TODO
                removed_grid_path_array_index = 0
                removed_uav_grid = []
                removed_grid_path_length = []
                removed_numlines = []
                mid_mission_data_cache = {}

                uncovered_area_points = []
                uncovered_area_filename = []
                removed_grid_path_array_flag = False
                removed_grid_path_array = [0] * len(pos_array)
                removed_grid_filename = [0] * len(pos_array)
                removed_grid_path_array_start_val = [0] * len(pos_array)
                checkall_removed_grid_path_array_start_val = [0] * len(pos_array)
                # Reset bot state for the new mission
                remove_bot_flag = False
                remove_bot_array = []
                include_uav_flag = False
                include_uav_index = []
            previous_task_flag = False
            print("len(s.swarm):", len(s.swarm))
            print("len(csv_file_paths):", len(csv_file_paths))
            print("active_bot_count:", active_bot_count)
            search_loop_running = True

            # ── Pre-loop sync: handle drones added AFTER mission completed ──
            if include_uav_flag and not search_flag:
                print(
                    "[PRE-LOOP] Mission already finished — syncing arrays for added drone"
                )
                while len(all_uav_csv_grid_array) < len(pos_array):
                    added_sysid = pos_array[len(all_uav_csv_grid_array)]
                    if added_sysid in mid_mission_data_cache:
                        m_data, _, _ = mid_mission_data_cache.pop(added_sysid)
                        if m_data in removed_uav_grid:
                            r_idx = removed_uav_grid.index(m_data)
                            removed_uav_grid.pop(r_idx)
                            removed_grid_path_length.pop(r_idx)
                            removed_numlines.pop(r_idx)
                    all_uav_csv_grid_array.append(0)
                    grid_path_array.append(0)
                    num_lines.append(0)
                    removed_grid_filename.append(0)
                    removed_grid_path_array.append(0)
                    removed_grid_path_array_start_val.append(0)
                    checkall_removed_grid_path_array_start_val.append(0)

                    _cwd = os.getcwd()
                    _path = os.path.join(_cwd, "searchgrid", f"d{added_sysid}.csv")
                    if os.path.exists(_path):
                        csv_file_paths.append(_path)
                    else:
                        csv_file_paths.append(None)
                include_uav_flag = False
                include_uav_index = []
                print("[PRE-LOOP] Arrays synced, drone is idle. No mission restart.")

            while 1:
                check_reconnection()
                # print("WHILEwww")
                time.sleep(sleep_times.get(len(vehicles), 0.1))
                if not search_flag:
                    search_loop_running = False
                    search_flag = False
                    break
                # TODO
                if include_uav_flag:
                    # Grow mission arrays to match the current swarm size in pos_array
                    while len(all_uav_csv_grid_array) < len(pos_array):
                        added_sysid = pos_array[len(all_uav_csv_grid_array)]

                        # LOGIC: Resume if data exists AND reallocation hasn't reassigned it yet
                        if (
                            added_sysid in mid_mission_data_cache
                            and not removed_grid_path_array_flag
                        ):
                            # Resume: Found saved mission data for this drone
                            m_data, length, lines = mid_mission_data_cache.pop(
                                added_sysid
                            )
                            all_uav_csv_grid_array.append(m_data)
                            grid_path_array.append(length)
                            num_lines.append(lines)
                            print(
                                f"[RESUME] Drone (SysID {added_sysid}) restored with saved mission data"
                            )

                            # CRITICAL: If this drone's data was in the reallocation pool,
                            # we MUST remove it so other drones don't try to cover it.
                            if m_data in removed_uav_grid:
                                r_idx = removed_uav_grid.index(m_data)
                                removed_uav_grid.pop(r_idx)
                                removed_grid_path_length.pop(r_idx)
                                removed_numlines.pop(r_idx)
                                print(
                                    f"[RESUME] Removed SysID {added_sysid} data from reallocation pool to prevent duplication"
                                )
                        else:
                            # Join as Idle: Either no saved data OR reallocation is already re-covering the area
                            if added_sysid in mid_mission_data_cache:
                                cached_data = mid_mission_data_cache.pop(added_sysid)[0]
                                # If it was joining as idle because reallocation IS active,
                                # the data stays in removed_uav_grid (already being re-covered).
                                print(
                                    f"[JOIN] Drone (SysID {added_sysid}) ignored saved data (reallocation already active)"
                                )

                            all_uav_csv_grid_array.append(0)
                            grid_path_array.append(0)
                            num_lines.append(0)
                            print(
                                f"[JOIN] Drone (SysID {added_sysid}) added as idle (Reallocation active or No saved data)"
                            )

                        # ALWAYS grow helper arrays so indexing by 'i' remains safe
                        removed_grid_filename.append(0)
                        removed_grid_path_array.append(0)
                        removed_grid_path_array_start_val.append(0)
                        checkall_removed_grid_path_array_start_val.append(0)

                        _cwd = os.getcwd()
                        _path = os.path.join(_cwd, "searchgrid", f"d{added_sysid}.csv")
                        if os.path.exists(_path):
                            csv_file_paths.append(_path)
                        else:
                            csv_file_paths.append(None)

                    include_uav_flag = False
                    include_uav_index = []
                    remove_bot_flag = False
                    if remove_bot_array == [] and removed_grid_path_length == []:
                        removed_grid_path_array_flag = False
                    print(
                        "include_uav_flag,remove_bot_array",
                        include_uav_flag,
                        remove_bot_array,
                        all_uav_csv_grid_array,
                        grid_path_array,
                        num_lines,
                    )

                # TODO
                if remove_bot_flag:
                    print("remove_bot_flag", remove_bot_flag, remove_bot_array)
                    # Process removals in the exact order they occurred to keep arrays aligned
                    for m, m_sysid in remove_bot_array:
                        # Save current progress before popping
                        m_data = all_uav_csv_grid_array.pop(m)
                        length = grid_path_array.pop(m)
                        lines = num_lines.pop(m)

                        # Cache mission progress by SysID for resumption
                        mid_mission_data_cache[m_sysid] = (m_data, length, lines)

                        # Add to removal pool specifically for reallocation logic
                        removed_uav_grid.append(m_data)
                        removed_grid_path_length.append(length)
                        removed_numlines.append(lines)

                        # Pop helper arrays to keep them the same length as vehicles
                        removed_grid_path_array.pop(m)
                        removed_grid_path_array_start_val.pop(m)
                        removed_grid_filename.pop(m)
                        checkall_removed_grid_path_array_start_val.pop(m)
                        # NOTE: pop_flag_arr is NOT popped here.
                        # It was already popped in the outer remove handler before search resumed.
                        # Popping again here caused: IndexError: list index out of range at pop_flag_arr[i]
                    remove_bot_array = []
                    remove_bot_flag = False
                    # Sync active_bot_count after removal
                    active_bot_count = len(pos_array)
                    # FIX: Rebuild csv_file_paths to match current pos_array.
                    # Without this, csv_file_paths still has 8 entries and if search_step
                    # resets to 1, it would assign wrong CSVs to remaining drones.
                    csv_file_paths = [None] * len(pos_array)
                    _cwd = os.getcwd()
                    for _i, _sysid in enumerate(pos_array):
                        _path = os.path.join(_cwd, "searchgrid", f"d{_sysid}.csv")
                        if os.path.exists(_path):
                            csv_file_paths[_i] = _path
                    print("csv_file_paths rebuilt after removal:", csv_file_paths)
                    print(
                        "removed_uav_grid,removed_grid_path_length,search_step,all_uav_csv_grid_array,grid_path_array",
                        removed_uav_grid,
                        removed_grid_path_length,
                        search_step,
                        all_uav_csv_grid_array,
                        grid_path_array,
                    )
                if search_step == 1:
                    for i, sysid in enumerate(pos_array):
                        all_uav_csv_grid_array[i] = csv_file_paths[i]
                    search_step += 1
                for m_index, sysid in enumerate(pos_array):
                    i = m_index
                    # FIX: Use m_index as b_index, not sysid-1.
                    # After removal, vehicles[] is compacted so sysid-1 points to the wrong vehicle.
                    # m_index is always the correct index into vehicles[] and s.swarm[].
                    b_index = m_index
                    try:
                        b = s.swarm[b_index]
                    except IndexError:
                        continue

                    # SAFETY GUARD: Skip if vehicle is dead or reconnecting
                    try:
                        if (
                            vehicles[b_index].last_heartbeat is None
                            or vehicles[b_index].last_heartbeat >= 28
                        ):
                            continue
                    except:
                        continue

                    if len(checkall_removed_grid_path_array_start_val) == len(
                        pos_array
                    ):
                        if all(
                            c == 1 for c in checkall_removed_grid_path_array_start_val
                        ):
                            previous_task_flag = False
                            removed_uav_grid = []
                            removed_grid_path_length = []
                            removed_numlines = []
                            uncovered_area_points = []
                            uncovered_area_filename = []
                            removed_grid_path_array_flag = False
                            removed_grid_path_array_start_val = [0] * len(pos_array)
                            checkall_removed_grid_path_array_start_val = [0] * len(
                                pos_array
                            )
                            landing_flag = True
                            log(f"MISSION COMPLETED111 {uid}", LOG_FILE=LOG_FILE)
                            search_flag = False
                            previous_task = b""
                            break

                    if (
                        any(
                            c >= int(num_lines[a])
                            for a, c in enumerate(grid_path_array)
                        )
                        and removed_grid_path_length != []
                        and not removed_grid_path_array_flag
                    ):
                        allocation, remaining_points_list = allocate_drones(
                            removed_numlines, removed_grid_path_length, len(pos_array)
                        )
                        remaining = [n - g for n, g in zip(num_lines, grid_path_array)]
                        # Sort areas by remaining points (ascending)
                        min_sorted = sorted(enumerate(remaining), key=lambda x: x[1])
                        print("min_sorted", min_sorted)
                        min_sorted_ptr = 0
                        # --- Assign start/end indices based on allocation ---
                        for x, v in enumerate(remaining_points_list):
                            print("x", x, min_sorted[min_sorted_ptr][0])
                            area_idx = min_sorted[min_sorted_ptr][0]
                            # Initial start index
                            start_index = (
                                abs(removed_grid_path_length[x])
                                if removed_grid_path_length[x] == 1
                                else abs(removed_grid_path_length[x] - 1)
                            )
                            print(
                                "start_index",
                                start_index,
                                area_idx,
                                removed_grid_path_length[x],
                            )
                            print(
                                "JJJ",
                                allocation[x],
                                removed_grid_path_length[x],
                                int(removed_numlines[x]),
                            )

                            # If no allocation
                            if allocation[x] == 0:
                                if removed_grid_path_length[x] != int(
                                    removed_numlines[x]
                                ):
                                    uncovered_area_points.append(
                                        removed_grid_path_length[x]
                                    )
                                    uncovered_area_filename.append(removed_uav_grid[x])
                                    print(
                                        "uncovered_area_points",
                                        x,
                                        uncovered_area_points,
                                        uncovered_area_filename,
                                    )
                                continue

                            # If only one drone → full area
                            if allocation[x] == 1:
                                end_index = int(removed_numlines[x])
                                print(
                                    "Only one drone end_index",
                                    end_index,
                                    removed_grid_path_array,
                                )
                                removed_grid_path_array[area_idx] = (
                                    start_index,
                                    end_index,
                                )
                                removed_grid_path_array_start_val[area_idx] = (
                                    start_index
                                )
                                removed_grid_filename[area_idx] = removed_uav_grid[x]
                                print(
                                    "Drone",
                                    m,
                                    "→",
                                    removed_grid_path_array[area_idx],
                                    removed_grid_path_array_start_val[area_idx],
                                    removed_grid_filename[area_idx],
                                )
                                min_sorted_ptr += 1
                            # If multiple drones → split area
                            else:
                                add_points = math.ceil(
                                    remaining_points_list[x] / allocation[x]
                                )
                                print("add_points", add_points)

                                for m in range(allocation[x]):
                                    area_idx = min_sorted[min_sorted_ptr][0]
                                    min_sorted_ptr += 1
                                    # area_idx = min_sorted[m][0]
                                    print("area_inx", area_idx)
                                    if m == 0:
                                        if start_index + add_points < int(
                                            removed_numlines[x]
                                        ):
                                            end_index = start_index + add_points
                                        else:
                                            end_index = int(removed_numlines[x])
                                    else:
                                        start_index = end_index
                                        if start_index + add_points < int(
                                            removed_numlines[x]
                                        ):
                                            end_index = start_index + add_points
                                        else:
                                            end_index = int(removed_numlines[x])
                                    # ✅ Assign to this same area index
                                    removed_grid_path_array[area_idx] = (
                                        start_index,
                                        end_index,
                                    )
                                    removed_grid_path_array_start_val[area_idx] = (
                                        start_index
                                    )
                                    removed_grid_filename[area_idx] = removed_uav_grid[
                                        x
                                    ]

                                    print(
                                        "Drone",
                                        m,
                                        "→",
                                        removed_grid_path_array[area_idx],
                                        removed_grid_path_array_start_val[area_idx],
                                        removed_grid_filename[area_idx],
                                    )

                        print(
                            "removed_grid_path_array%%%%",
                            removed_grid_path_array,
                            removed_grid_path_array_start_val,
                            removed_grid_filename,
                        )
                        removed_grid_path_array_flag = True

                    if (
                        all(
                            c >= int(num_lines[a])
                            for a, c in enumerate(grid_path_array)
                        )
                        and not removed_grid_path_length != []
                    ):
                        previous_task_flag = False
                        removed_uav_grid = []
                        removed_grid_path_length = []
                        uncovered_area_points = []
                        uncovered_area_filename = []
                        removed_grid_path_array_start_val = [0] * len(pos_array)
                        removed_grid_path_array_flag = False
                        removed_grid_path_array = [0] * len(pos_array)
                        checkall_removed_grid_path_array_start_val = [0] * len(
                            pos_array
                        )
                        landing_flag = True
                        log(f"MISSION COMPLETED22 {uid}", LOG_FILE=LOG_FILE)
                        search_flag = False
                        previous_task = b""
                        break
                    if (
                        removed_grid_path_array_flag
                        and all(
                            c >= int(num_lines[a])
                            for a, c in enumerate(grid_path_array)
                        )
                        and all(
                            removed_grid_path_array_start_val[a]
                            >= removed_grid_path_array[a][1]
                            for a in range(len(removed_grid_path_array))
                            if removed_grid_path_array[a] != 0
                        )
                    ):

                        previous_task_flag = False
                        removed_uav_grid = []
                        removed_grid_path_length = []
                        uncovered_area_points = []
                        uncovered_area_filename = []
                        removed_grid_path_array_start_val = [0] * len(pos_array)
                        removed_grid_path_array_flag = False
                        removed_grid_path_array = [0] * len(pos_array)
                        checkall_removed_grid_path_array_start_val = [0] * len(
                            pos_array
                        )
                        landing_flag = True
                        log(f"MISSION COMPLETED@@@@ {uid}", LOG_FILE=LOG_FILE)
                        search_flag = False
                        previous_task = b""
                        break

                    if (removed_grid_path_array_flag) and grid_path_array[i] >= int(
                        num_lines[i]
                    ):
                        if removed_grid_path_array_start_val[i] == 0:
                            checkall_removed_grid_path_array_start_val[i] = 1
                            continue
                        if (
                            removed_grid_path_array_start_val[i]
                            >= removed_grid_path_array[i][1]
                        ):
                            checkall_removed_grid_path_array_start_val[i] = 1
                            if uncovered_area_points != []:
                                print("uncovered_area_points", uncovered_area_points)
                                for u, uncovered_area_point in enumerate(
                                    uncovered_area_points
                                ):
                                    removed_grid_path_array[i] = (
                                        uncovered_area_point,
                                        int(num_lines[i]) + 1,
                                    )
                                    print(
                                        "removed_grid_path_array!!!!",
                                        removed_grid_path_array,
                                    )
                                    removed_grid_path_array_start_val[i] = (
                                        uncovered_area_points[u]
                                    )
                                    removed_grid_filename[i] = uncovered_area_filename[
                                        u
                                    ]
                                    removed_grid_path_array[i] = (
                                        uncovered_area_points[u],
                                        int(num_lines[i]),
                                    )
                                    print(
                                        "removed_grid_path_array_start_val@@@@",
                                        removed_grid_path_array_start_val,
                                        removed_grid_filename,
                                    )
                                    checkall_removed_grid_path_array_start_val[i] = 0
                                    uncovered_area_points.pop(u)
                                    uncovered_area_filename.pop(u)
                            else:
                                continue
                    if (
                        grid_path_array[i] >= int(num_lines[i])
                        and not removed_grid_path_array_flag
                    ):
                        continue
                    if removed_grid_path_array_flag:
                        if grid_path_array[i] < num_lines[i]:

                            goal_lat_lon = read_specific_line(
                                all_uav_csv_grid_array[i], grid_path_array[i]
                            )
                        else:
                            # Guard: skip if already past end of reassigned area
                            if (
                                removed_grid_path_array[i] != 0
                                and removed_grid_path_array_start_val[i]
                                >= removed_grid_path_array[i][1]
                            ):
                                continue
                            goal_lat_lon = read_specific_line(
                                removed_grid_filename[i],
                                removed_grid_path_array_start_val[i],
                            )
                    else:
                        goal_lat_lon = read_specific_line(
                            all_uav_csv_grid_array[i], grid_path_array[i]
                        )
                    x, y = goal_lat_lon[0][0], goal_lat_lon[0][1]
                    goal = (x, y)
                    goal_coord = locatePosition.cartToGeo(
                        origin, endDistance, [x * 2, y * 2]
                    )
                    cmd = cvg.goal_area_cvg(b_index, b, goal)
                    value = [b.x * 2, b.y * 2]
                    current_position = [b.x, b.y]
                    dx = abs(goal[0] - current_position[0])
                    dy = abs(goal[1] - current_position[1])
                    if dx <= 0.5 and dy <= 0.5:
                        distance = locatePosition.distance_bearing(
                            vehicles[b_index].location.global_relative_frame.lat,
                            vehicles[b_index].location.global_relative_frame.lon,
                            goal_coord[0],
                            goal_coord[1],
                        )
                        if (
                            grid_path_array[i] >= int(num_lines[i])
                            and not removed_grid_path_array_flag
                        ):
                            continue
                        if (
                            grid_path_array[i] >= int(num_lines[i])
                            and removed_grid_path_array_flag
                            and distance < 5
                        ):
                            removed_grid_path_array_start_val[i] += 1
                            print(
                                "removed_grid_path_array_start_val",
                                removed_grid_path_array_start_val,
                            )

                        else:
                            if distance < 5:
                                grid_path_array[i] += 1
                                print("grid_path_array", grid_path_array)
                    # --- Proximity Monitor ---
                    for other_idx, other_b in enumerate(s.swarm):
                        if other_idx > i:  # Only check each pair once
                            dist = math.sqrt(
                                (b.x - other_b.x) ** 2 + (b.y - other_b.y) ** 2
                            )
                            if dist < 5.0:
                                print(
                                    # f"[ALERT] Drone {i+1} and {other_idx+1} are {dist:.2f}m apart!"
                                )

                    cmd.exec(b)
                    if master_flag:
                        if pop_flag_arr[i] == 1:
                            lat, lon = locatePosition.cartToGeo(
                                origin, endDistance, value
                            )
                            point1 = LocationGlobalRelative(
                                lat, lon, different_height[i]
                            )
                            vehicles[i].simple_goto(point1)

                s.time_elapsed += 1
                if index == b"stop":
                    search_flag = False
                    print("Search Stoped!!!")
                    previous_task = b"search"
                    previous_task_flag = False
                    log(f"MISSION PAUSED {uid}", LOG_FILE=LOG_FILE)
                    search_flag = False
                    break
                if data.startswith(b"split"):
                    print("AutoSplitMission Started")

        if (
            data.startswith(
                (b"split", b"specificsplit", b"polyspecificsplit", b"polyautosplit")
            )
            or split_flag
        ):
            index = "data"
            print("previous_task_flag", previous_task_flag)
            split_flag = True
            if previous_task == b"specificbotgoal":
                specific_bot_goal_flag_array = [False] * num_bots

            if (data.startswith(b"specificsplit")) and not previous_task_flag:
                try:
                    decoded_index = data.decode(
                        "utf-8"
                    )  # Assuming utf-8 encoding, adjust if needed
                    msg_parts = decoded_index.split("_")
                    print("msg_parts", msg_parts)
                    f = msg_parts[0]  # First coordinate pair
                    print("f", f)
                    center_lat_lon_array = msg_parts[1]  # All other coordinates
                    center_lat_lon_array = json.loads(center_lat_lon_array)
                    print("center_lat_lon_array", center_lat_lon_array)
                    uav_array = msg_parts[2]
                    uav_array = json.loads(uav_array)
                    grid_space = msg_parts[3]
                    grid_space = json.loads(grid_space)
                    print("grid_space", grid_space)
                    coverage_area = msg_parts[4]
                    coverage_area = json.loads(coverage_area)
                    print("coverage_area", coverage_area)
                    uid = msg_parts[5]
                    print("uid", uid)
                    split = SpecificSplitMission(
                        origin=origin,
                        center_lat_lons=center_lat_lon_array,
                        drone_array=uav_array,
                        grid_spacing=grid_space,
                        coverage_area=coverage_area,
                    )
                    isDone = split.GroupSplitting(
                        center_lat_lons=center_lat_lon_array,
                        drone_array=uav_array,
                        grid_spacing=grid_space,
                        coverage_area=coverage_area,
                    )
                    # Clear cache when new specificsplit paths are generated
                    csv_cache.clear()
                    previous_task = b"specificsplit"
                    previous_task_flag = False
                except Exception as e:
                    print("Exception", e)

            if (data.startswith(b"polyautosplit")) and not previous_task_flag:
                print("PolyAutoSplit")
                decoded_index = data.decode(
                    "utf-8"
                )  # Assuming utf-8 encoding, adjust if needed
                msg_parts = decoded_index.split("_")
                print("msg_parts", msg_parts, len(msg_parts))
                f = msg_parts[0]  # First coordinate pair
                num_uavs = msg_parts[2]
                grid_space = msg_parts[3]
                grid_space = json.loads(grid_space)
                coverage_area = msg_parts[4]
                coverage_area = json.loads(coverage_area)
                polygon_array = msg_parts[1]  # All other coordinates
                polygon_array = json.loads(polygon_array)
                uid = msg_parts[5]
                print(f"Initializing PolygonAutoSplit for {len(pos_array)} drones...")
                split = PolygonAutoSplit(
                    polygon_latlon_list=polygon_array,
                    origin_gps=origin,
                    endDistance=endDistance,  # as in your original code
                    drone_list=pos_array,
                    grid_spacing=int(grid_space),
                    rotation_angle=90,  # or -1 to auto-detect angle
                    obstacles_latlon_list=[],
                )
                print("split", split)
                split.generate_paths()
                split.save_paths()
                # Clear cache when new autosplit paths are generated
                csv_cache.clear()
                print("PolygonAutoSplit: Paths generated and saved successfully.")
                previous_task = b"split"
                previous_task_flag = False
                print("num_uavs from command:", num_uavs)
                print("len(pos_array):", len(pos_array))
                print("active_bot_count:", active_bot_count)

            if (data.startswith(b"polyspecificsplit")) and not previous_task_flag:
                decoded_index = data.decode(
                    "utf-8"
                )  # Assuming utf-8 encoding, adjust if needed
                msg_parts = decoded_index.split("_")
                print("msg_parts", msg_parts, len(msg_parts))
                f = msg_parts[0]  # First coordinate pair
                uav_array = msg_parts[2]
                uav_array = json.loads(uav_array)
                grid_space = msg_parts[3]
                grid_space = json.loads(grid_space)
                coverage_area = msg_parts[4]
                coverage_area = json.loads(coverage_area)
                polygon_array = msg_parts[1]  # All other coordinates
                polygon_array = json.loads(polygon_array)
                uid = msg_parts[5]
                split = PolygonSpecificSplit(
                    polygon_latlon_list=polygon_array,
                    origin_gps=origin,
                    endDistance=endDistance,
                    drone_list=pos_array,
                    grid_spacing=grid_space,
                    rotation_angle=90,
                    obstacles_latlon_list=None,
                    drone_assignments=uav_array,
                )

                paths = split.generate_paths()
                split.save_paths()

                # Clear cache when new specificsplit paths are generated
                csv_cache.clear()
                print("PolygonSpecificSplit: Paths generated and saved successfully.")

                previous_task = b"split"
                previous_task_flag = False

            if (data.startswith(b"split")) and not previous_task_flag:
                decoded_index = data.decode(
                    "utf-8"
                )  # Assuming utf-8 encoding, adjust if needed
                msg_parts = decoded_index.split("_")
                print("msg_parts", msg_parts, len(msg_parts))
                f = msg_parts[0]  # First coordinate pair
                num_uavs = msg_parts[2]
                grid_space = msg_parts[3]
                grid_space = json.loads(grid_space)
                coverage_area = msg_parts[4]
                coverage_area = json.loads(coverage_area)
                center_lat_lon_array = msg_parts[1]  # All other coordinates
                center_lat_lon_array = json.loads(center_lat_lon_array)
                # Flatten extra nesting if the frontend sends [[[lat, lon]], [[lat, lon]]]
                flattened_array = []
                for item in center_lat_lon_array:
                    if len(item) == 1 and isinstance(item[0], list):
                        flattened_array.append(item[0])
                    else:
                        flattened_array.append(item)
                center_lat_lon_array = flattened_array
                uid = msg_parts[5]
                split = AutoSplitMission(
                    origin=origin,
                    center_lat_lons=center_lat_lon_array,
                    drone_list=pos_array,
                    grid_spacing=int(grid_space),
                    coverage_area=int(coverage_area),
                )
                isDone = split.GroupSplitting(
                    center_lat_lons=center_lat_lon_array,
                    num_of_drones=len(pos_array),
                    grid_spacing=int(grid_space),
                    coverage_area=int(coverage_area),
                )

                # Clear cache when new split paths are generated
                csv_cache.clear()
                print("AutoSplitMission: Paths generated and saved successfully.")

                previous_task = b"split"
                previous_task_flag = False
            if not previous_task_flag:
                previous_task_flag = False
                split_flag_val = 0
                search_step = 1
                all_uav_csv_grid_array = [0] * len(pos_array)
                pop_flag_arr = [1] * len(pos_array)
                grid_path_array = [0] * len(pos_array)
                if master_flag:
                    index = "data"
                    uav_home_pos = []
                    for vehicle in vehicles:
                        lat = vehicle.location.global_relative_frame.lat
                        lon = vehicle.location.global_relative_frame.lon
                        x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                        uav_home_pos.append((x / 2, y / 2))
                    origin = read_origin(file_name)
                    s = sim.Simulation(
                        uav_home_pos,
                        num_bots=num_bots,
                        env_name=file_name,
                        speed=bot_speed,
                    )

                print("Group Splitting Started")
                print("num_bots", num_bots)
                print("s.swarm", s.swarm)
                print("active_bot_count", active_bot_count)
                f = ""
                goal_bot_num = 0
                goal_position = []
                num_lines = [0] * len(pos_array)
                cwd = os.getcwd()
                grid_path_array = [0] * len(pos_array)
                print("split_flag_val", split_flag_val)

                if split_flag_val == 0 and (
                    data.startswith(b"split") or data.startswith(b"polyautosplit")
                ):
                    print("AutoSplitMission Started")
                    print("active_bot_count", active_bot_count)
                    split_flag_val += 1
                    csv_file_paths = [0] * len(pos_array)
                    num_lines = [0] * len(pos_array)
                    print("pos_array", pos_array)
                    for i in range(len(pos_array)):
                        print("i", i)
                        e = pos_array[i]
                        path = os.path.join(cwd, "group_split", f"grid_{e}.csv")
                        print("pos_array[i]", pos_array[i], path)

                        csv_file_paths[i] = path if os.path.exists(path) else None

                        if os.path.exists(path):
                            with open(path) as f:
                                reader = csv.reader(f)
                                num_lines[i] = sum(1 for _ in reader)

                if split_flag_val == 0 and (
                    data.startswith(b"specificsplit")
                    or data.startswith(b"polyspecificsplit")
                ):
                    print("SpecificSplitMission Started !!!!!")

                    split_flag_val += 1
                    csv_file_paths = [None] * len(pos_array)
                    num_lines = [0] * len(pos_array)

                    for i in range(len(pos_array)):
                        e = pos_array[i]
                        path = os.path.join(cwd, "group_split", f"grid_{e}.csv")
                        print("path specificsplit mission", path)

                        csv_file_paths[i] = path if os.path.exists(path) else None
                        print("csv_file_paths[i]", csv_file_paths[i])

                        if os.path.exists(path):
                            with open(path) as f:
                                reader = csv.reader(f)
                                num_lines[i] = sum(1 for _ in reader)
                removed_uav_grid = []
                mid_mission_data_cache = {}

                removed_grid_path_length = []
                removed_numlines = []
                uncovered_area_points = []
                uncovered_area_filename = []
                removed_grid_path_array_flag = False
                removed_grid_path_array = [0] * len(pos_array)
                removed_grid_filename = [0] * len(pos_array)
                removed_grid_path_array_start_val = [0] * len(pos_array)
                checkall_removed_grid_path_array_start_val = [0] * len(pos_array)
                # Arrays are freshly built for current pos_array; any prior
                # bot state is already accounted for — reset so the
                # while loop doesn't try to pop from them again.
                remove_bot_flag = False
                remove_bot_array = []
                include_uav_flag = False
                include_uav_index = []
            previous_task_flag = False

            # ── Pre-loop sync: handle drones added AFTER split mission completed ──
            if include_uav_flag and not split_flag:
                print(
                    "[PRE-LOOP-SPLIT] Mission already finished — syncing arrays for added drone"
                )
                while len(all_uav_csv_grid_array) < len(pos_array):
                    added_sysid = pos_array[len(all_uav_csv_grid_array)]
                    if added_sysid in mid_mission_data_cache:
                        m_data, _, _ = mid_mission_data_cache.pop(added_sysid)
                        if m_data in removed_uav_grid:
                            r_idx = removed_uav_grid.index(m_data)
                            removed_uav_grid.pop(r_idx)
                            removed_grid_path_length.pop(r_idx)
                            removed_numlines.pop(r_idx)
                    all_uav_csv_grid_array.append(0)
                    grid_path_array.append(0)
                    num_lines.append(0)
                    removed_grid_filename.append(0)
                    removed_grid_path_array.append(0)
                    removed_grid_path_array_start_val.append(0)
                    checkall_removed_grid_path_array_start_val.append(0)
                    pop_flag_arr.append(1)
                include_uav_flag = False
                include_uav_index = []
                print(
                    "[PRE-LOOP-SPLIT] Arrays synced, drone is idle. No mission restart."
                )

            while 1:
                check_reconnection()
                time.sleep(sleep_times.get(num_bots, 0.1))
                if not split_flag:
                    split_flag = False
                    break
                if include_uav_flag:
                    # Grow mission arrays to match the current swarm size in pos_array
                    while len(all_uav_csv_grid_array) < len(pos_array):
                        added_sysid = pos_array[len(all_uav_csv_grid_array)]

                        # LOGIC: Resume if data exists AND reallocation hasn't reassigned it yet
                        if (
                            added_sysid in mid_mission_data_cache
                            and not removed_grid_path_array_flag
                        ):
                            # Resume: Found saved mission data for this drone
                            m_data, length, lines = mid_mission_data_cache.pop(
                                added_sysid
                            )
                            all_uav_csv_grid_array.append(m_data)
                            grid_path_array.append(length)
                            num_lines.append(lines)
                            print(
                                f"[RESUME-SPLIT] Drone (SysID {added_sysid}) restored with saved mission data"
                            )

                            # CRITICAL: Prevent duplication by removing from reallocation pool
                            if m_data in removed_uav_grid:
                                r_idx = removed_uav_grid.index(m_data)
                                removed_uav_grid.pop(r_idx)
                                removed_grid_path_length.pop(r_idx)
                                removed_numlines.pop(r_idx)
                                print(
                                    f"[RESUME-SPLIT] Removed SysID {added_sysid} from pool"
                                )
                        else:
                            # Join as Idle: Either no saved data OR reallocation is already re-covering the area
                            if added_sysid in mid_mission_data_cache:
                                mid_mission_data_cache.pop(added_sysid)  # Discard

                            all_uav_csv_grid_array.append(0)
                            grid_path_array.append(0)
                            num_lines.append(0)
                            print(
                                f"[JOIN-SPLIT] Drone (SysID {added_sysid}) added as idle (Reallocation active or No saved data)"
                            )

                        # ALWAYS grow helper arrays so indexing by 'i' remains safe
                        removed_grid_filename.append(0)
                        removed_grid_path_array.append(0)
                        removed_grid_path_array_start_val.append(0)
                        checkall_removed_grid_path_array_start_val.append(0)
                    pop_flag_arr.append(1)

                    include_uav_flag = False
                    remove_bot_flag = False
                    if remove_bot_array == [] and removed_grid_path_length == []:
                        removed_grid_path_array_flag = False
                    print(
                        "include_uav_flag,remove_bot_array",
                        include_uav_flag,
                        remove_bot_array,
                        all_uav_csv_grid_array,
                        grid_path_array,
                        num_lines,
                    )
                if remove_bot_flag:
                    print("remove_bot_flag", remove_bot_flag, remove_bot_array)
                    # Process removals in the exact order they occurred
                    for m, m_sysid in remove_bot_array:
                        m_data = all_uav_csv_grid_array.pop(m)
                        length = grid_path_array.pop(m)
                        lines = num_lines.pop(m)

                        # Cache mission progress
                        mid_mission_data_cache[m_sysid] = (m_data, length, lines)

                        # Pool for reallocation
                        removed_uav_grid.append(m_data)
                        removed_grid_path_length.append(length)
                        removed_numlines.append(lines)

                        # Helper arrays
                        removed_grid_path_array.pop(m)
                        removed_grid_path_array_start_val.pop(m)
                        removed_grid_filename.pop(m)
                        checkall_removed_grid_path_array_start_val.pop(m)
                        # FIX: pop_flag_arr already popped in outer remove handler — do NOT pop again here.
                    remove_bot_array = []
                    remove_bot_flag = False
                    # FIX: Sync active_bot_count and rebuild csv_file_paths after removal.
                    active_bot_count = len(pos_array)
                    csv_file_paths = [None] * len(pos_array)
                    _cwd = os.getcwd()
                    for _i, _sysid in enumerate(pos_array):
                        _path = os.path.join(_cwd, "searchgrid", f"d{_sysid}.csv")
                        if os.path.exists(_path):
                            csv_file_paths[_i] = _path
                    print(
                        "csv_file_paths rebuilt after removal (split):", csv_file_paths
                    )
                if search_step == 1:
                    for i, sysid in enumerate(pos_array):
                        all_uav_csv_grid_array[i] = csv_file_paths[i]
                    search_step += 1
                for m_index, sysid in enumerate(pos_array):
                    i = m_index
                    # FIX: Use m_index as b_index — vehicles[] is compacted after removal, sysid-1 is wrong.
                    b_index = m_index
                    try:
                        b = s.swarm[b_index]
                    except IndexError:
                        continue

                    # SAFETY GUARD: Skip if vehicle is dead or reconnecting
                    try:
                        if (
                            vehicles[b_index].last_heartbeat is None
                            or vehicles[b_index].last_heartbeat >= 28
                        ):
                            continue
                    except:
                        continue

                    if len(checkall_removed_grid_path_array_start_val) == len(
                        pos_array
                    ):
                        if all(
                            c == 1 for c in checkall_removed_grid_path_array_start_val
                        ):
                            previous_task_flag = False
                            removed_uav_grid = []
                            removed_grid_path_length = []
                            removed_numlines = []
                            uncovered_area_points = []
                            uncovered_area_filename = []
                            removed_grid_path_array_start_val = [0] * len(pos_array)
                            checkall_removed_grid_path_array_start_val = [0] * len(
                                pos_array
                            )
                            removed_grid_path_array_flag = False
                            landing_flag = True
                            log(f"MISSION COMPLETED {uid}", LOG_FILE=LOG_FILE)
                            split_flag = False
                            previous_task = b""
                            break

                    if (
                        any(
                            c >= int(num_lines[a])
                            for a, c in enumerate(grid_path_array)
                        )
                        and removed_grid_path_length != []
                        and not removed_grid_path_array_flag
                    ):

                        print(
                            "removed_grid_path_length",
                            removed_grid_path_length,
                            removed_numlines,
                        )
                        allocation, remaining_points_list = allocate_drones(
                            removed_numlines, removed_grid_path_length, len(pos_array)
                        )
                        print(
                            "allocation,remaining_points_list",
                            allocation,
                            remaining_points_list,
                            num_lines,
                            grid_path_array,
                        )
                        remaining = [n - g for n, g in zip(num_lines, grid_path_array)]
                        # Sort areas by remaining points (ascending)
                        min_sorted = sorted(enumerate(remaining), key=lambda x: x[1])
                        min_sorted_ptr = 0
                        # --- Assign start/end indices based on allocation ---
                        for x, v in enumerate(remaining_points_list):
                            area_idx = min_sorted[min_sorted_ptr][0]
                            # Initial start index
                            start_index = (
                                abs(removed_grid_path_length[x])
                                if removed_grid_path_length[x] == 1
                                else abs(removed_grid_path_length[x] - 1)
                            )
                            print(
                                "start_index", start_index, removed_grid_path_length[x]
                            )
                            print(
                                "JJJ",
                                allocation[x],
                                removed_grid_path_length[x],
                                int(removed_numlines[x]),
                            )

                            # If no allocation
                            if allocation[x] == 0:
                                if removed_grid_path_length[x] != int(
                                    removed_numlines[x]
                                ):
                                    uncovered_area_points.append(
                                        removed_grid_path_length[x]
                                    )
                                    uncovered_area_filename.append(removed_uav_grid[x])
                                    print(
                                        "uncovered_area_points",
                                        x,
                                        uncovered_area_points,
                                        uncovered_area_filename,
                                    )
                                continue

                            # If only one drone → full area
                            if allocation[x] == 1:
                                end_index = int(removed_numlines[x])
                                print("Only one drone end_index", end_index)
                                removed_grid_path_array[area_idx] = (
                                    start_index,
                                    end_index,
                                )
                                removed_grid_path_array_start_val[area_idx] = (
                                    start_index
                                )
                                removed_grid_filename[area_idx] = removed_uav_grid[x]
                                print(
                                    "Drone",
                                    m,
                                    "→",
                                    removed_grid_path_array[area_idx],
                                    removed_grid_path_array_start_val[area_idx],
                                    removed_grid_filename[area_idx],
                                )
                                min_sorted_ptr += 1

                            # If multiple drones → split area
                            else:
                                add_points = math.ceil(
                                    remaining_points_list[x] / allocation[x]
                                )
                                print("add_points", add_points)

                                for m in range(allocation[x]):
                                    area_idx = min_sorted[min_sorted_ptr][0]
                                    min_sorted_ptr += 1
                                    if m == 0:
                                        if start_index + add_points < int(
                                            removed_numlines[x]
                                        ):
                                            end_index = start_index + add_points
                                        else:
                                            end_index = int(removed_numlines[x])
                                    else:
                                        start_index = end_index
                                        if start_index + add_points < int(
                                            removed_numlines[x]
                                        ):
                                            end_index = start_index + add_points
                                        else:
                                            end_index = int(removed_numlines[x])
                                    # ✅ Assign to this same area index
                                    removed_grid_path_array[area_idx] = (
                                        start_index,
                                        end_index,
                                    )
                                    removed_grid_path_array_start_val[area_idx] = (
                                        start_index
                                    )
                                    removed_grid_filename[area_idx] = removed_uav_grid[
                                        x
                                    ]

                                    print(
                                        "Drone",
                                        m,
                                        "→",
                                        removed_grid_path_array[area_idx],
                                        removed_grid_path_array_start_val[area_idx],
                                        removed_grid_filename[area_idx],
                                    )

                        print(
                            "removed_grid_path_array!!!!!",
                            removed_grid_path_array,
                            removed_grid_path_array_start_val,
                            removed_grid_filename,
                        )
                        removed_grid_path_array_flag = True

                    finished_drones = [
                        i
                        for i, c in enumerate(grid_path_array)
                        if c >= int(num_lines[i])
                    ]
                    if (
                        finished_drones
                        and removed_grid_path_length != []
                        and not removed_grid_path_array_flag
                    ):

                        previous_task_flag = False
                        removed_uav_grid = []
                        removed_grid_path_length = []
                        uncovered_area_points = []
                        uncovered_area_filename = []
                        removed_grid_path_array_flag = False
                        removed_grid_path_array_start_val = [0] * len(pos_array)
                        removed_grid_path_array = [0] * len(pos_array)
                        checkall_removed_grid_path_array_start_val = [0] * len(
                            pos_array
                        )
                        landing_flag = True
                        log(f"MISSION COMPLETED {uid}", LOG_FILE=LOG_FILE)
                        split_flag = False
                        previous_task = b""
                        break

                    if (
                        removed_grid_path_array_flag
                        and all(
                            c >= int(num_lines[a])
                            for a, c in enumerate(grid_path_array)
                        )
                        and all(
                            removed_grid_path_array_start_val[a]
                            >= removed_grid_path_array[a][1]
                            for a in range(len(removed_grid_path_array))
                            if removed_grid_path_array[a] != 0
                        )
                    ):

                        previous_task_flag = False
                        removed_uav_grid = []
                        removed_grid_path_length = []
                        uncovered_area_points = []
                        uncovered_area_filename = []
                        removed_grid_path_array_start_val = [0] * len(pos_array)
                        removed_grid_path_array_flag = False
                        removed_grid_path_array = [0] * len(pos_array)
                        checkall_removed_grid_path_array_start_val = [0] * len(
                            pos_array
                        )
                        landing_flag = True
                        log(f"MISSION COMPLETED#### {uid}", LOG_FILE=LOG_FILE)
                        search_flag = False
                        previous_task = b""
                        break
                    if (removed_grid_path_array_flag) and grid_path_array[i] >= int(
                        num_lines[i]
                    ):
                        if removed_grid_path_array_start_val[i] == 0:
                            checkall_removed_grid_path_array_start_val[i] = 1
                            continue
                        if (
                            removed_grid_path_array_start_val[i]
                            >= removed_grid_path_array[i][1]
                        ):
                            checkall_removed_grid_path_array_start_val[i] = 1
                            if uncovered_area_points != []:
                                print("uncovered_area_points", uncovered_area_points)
                                for u, uncovered_area_point in enumerate(
                                    uncovered_area_points
                                ):
                                    removed_grid_path_array[i] = (
                                        uncovered_area_point,
                                        int(num_lines[i]) + 1,
                                    )
                                    print(
                                        "removed_grid_path_array",
                                        removed_grid_path_array,
                                    )
                                    removed_grid_path_array_start_val[i] = (
                                        uncovered_area_points[u]
                                    )
                                    removed_grid_filename[i] = uncovered_area_filename[
                                        u
                                    ]
                                    removed_grid_path_array[i] = (
                                        uncovered_area_points[u],
                                        int(num_lines[i]),
                                    )
                                    print(
                                        "removed_grid_path_array_start_val",
                                        removed_grid_path_array_start_val,
                                        removed_grid_filename,
                                    )
                                    checkall_removed_grid_path_array_start_val[i] = 0
                                    uncovered_area_points.pop(u)
                                    uncovered_area_filename.pop(u)
                            else:
                                continue
                    if (
                        grid_path_array[i] >= int(num_lines[i])
                        and not removed_grid_path_array_flag
                    ):
                        continue
                    if removed_grid_path_array_flag:
                        if grid_path_array[i] < num_lines[i]:
                            goal_lat_lon = read_specific_line(
                                all_uav_csv_grid_array[i], grid_path_array[i]
                            )
                        else:
                            # Guard: skip if already past end of reassigned area
                            if (
                                removed_grid_path_array[i] != 0
                                and removed_grid_path_array_start_val[i]
                                >= removed_grid_path_array[i][1]
                            ):
                                continue
                            goal_lat_lon = read_specific_line(
                                removed_grid_filename[i],
                                removed_grid_path_array_start_val[i],
                            )
                    else:
                        goal_lat_lon = read_specific_line(
                            all_uav_csv_grid_array[i], grid_path_array[i]
                        )
                    if not goal_lat_lon:
                        if removed_grid_path_array_flag and grid_path_array[i] >= int(
                            num_lines[i]
                        ):
                            removed_grid_path_array_start_val[i] += 1
                        else:
                            grid_path_array[i] += 1
                        continue
                    x, y = goal_lat_lon[0][0], goal_lat_lon[0][1]
                    goal = (x, y)
                    goal_coord = locatePosition.cartToGeo(
                        origin, endDistance, [x * 2, y * 2]
                    )
                    cmd = cvg.goal_area_cvg(b_index, b, goal)
                    value = [b.x * 2, b.y * 2]
                    current_position = [b.x, b.y]
                    dx = abs(goal[0] - current_position[0])
                    dy = abs(goal[1] - current_position[1])
                    if dx <= 1 and dy <= 1:
                        distance = locatePosition.distance_bearing(
                            vehicles[b_index].location.global_relative_frame.lat,
                            vehicles[b_index].location.global_relative_frame.lon,
                            goal_coord[0],
                            goal_coord[1],
                        )
                        if (
                            grid_path_array[i] >= int(num_lines[i])
                            and not removed_grid_path_array_flag
                        ):
                            continue
                        if (
                            grid_path_array[i] < int(num_lines[i])
                            and removed_grid_path_array_flag
                        ):
                            grid_path_array[i] += 1
                            print("grid_path_array@@@", grid_path_array)

                        elif (
                            grid_path_array[i] >= int(num_lines[i])
                            and removed_grid_path_array_flag
                            and distance < 5
                        ):
                            removed_grid_path_array_start_val[i] += 1
                            print(
                                "removed_grid_path_array_start_val",
                                removed_grid_path_array_start_val,
                            )

                        else:
                            if distance < 5:
                                grid_path_array[i] += 1
                                print("grid_path_array", grid_path_array)
                    # --- Proximity Monitor ---
                    for other_idx, other_b in enumerate(s.swarm):
                        if other_idx > i:  # Only check each pair once
                            dist = math.sqrt(
                                (b.x - other_b.x) ** 2 + (b.y - other_b.y) ** 2
                            )
                            if dist < 5.0:
                                print(
                                    # f"[ALERT] Drone {i+1} and {other_idx+1} are {dist:.2f}m apart!"
                                )

                    cmd.exec(b)
                    if master_flag:
                        if pop_flag_arr[i] == 1:
                            lat, lon = locatePosition.cartToGeo(
                                origin, endDistance, value
                            )
                            point1 = LocationGlobalRelative(
                                lat, lon, different_height[i]
                            )
                            vehicles[i].simple_goto(point1)

                s.time_elapsed += 1
                if index == b"stop":
                    previous_task = b"split"
                    previous_task_flag = False
                    log(f"MISSION PAUSED {uid}", LOG_FILE=LOG_FILE)
                    split_flag = False
                    break

        if (data.startswith(b"aggregate")) or (aggregate_flag):
            decoded_index = data.decode(
                "utf-8"
            )  # Assuming utf-8 encoding, adjust if needed
            print("decoded_index", decoded_index)
            f, agg_lat, agg_lon = decoded_index.split(",")
            print("agg_lat,agg_lon", agg_lat, agg_lon)
            search_flag = False
            aggregate_flag = True
            bot_reached = [0] * len(pos_array)
            x, y = locatePosition.geoToCart(
                origin, endDistance, [float(agg_lat), float(agg_lon)]
            )
            agg_goal_point = (x / 2, y / 2)
            index = "data"
            if master_flag:
                index = "data"
                uav_home_pos = []
                for vehicle in vehicles:
                    lat = vehicle.location.global_relative_frame.lat
                    lon = vehicle.location.global_relative_frame.lon
                    x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                    uav_home_pos.append((x / 2, y / 2))

            if previous_task == b"specificbotgoal":
                specific_bot_goal_flag_array = [False] * num_bots

            if not start_return_csv_flag:
                origin = read_origin(file_name)
                s = sim.Simulation(
                    uav_home_pos,
                    num_bots=len(pos_array),
                    env_name=file_name,
                    speed=bot_speed,
                )
            print("Aggregate!!!!")
            while True:
                time.sleep(sleep_times.get(num_bots, 0.1))
                if not aggregate_flag:
                    index = "data"
                    break
                if aggregate_flag:
                    velocity_flag = True
                    time.sleep(0.01)
                    for i, b in enumerate(s.swarm):
                        goal_position = agg_goal_point
                        current_position = [b.x, b.y]
                        dx = abs(goal_position[0] - current_position[0])
                        dy = abs(goal_position[1] - current_position[1])
                        if dx <= 1 and dy <= 1:
                            bot_reached[i] = 1
                            b.cancel_goal()
                            if all(element == 1 for element in bot_reached):
                                aggregate_flag = False
                                break
                        else:
                            current_position = (b.x, b.y)
                            b.set_goal(goal_position[0], goal_position[1])
                            cmd = cvg.goal_area_cvg(i, b, goal_position)
                            cmd.exec(b)

                        if master_flag:
                            value = [b.x * 2, b.y * 2]
                            lat, lon = locatePosition.cartToGeo(
                                origin, endDistance, value
                            )
                            point1 = LocationGlobalRelative(
                                lat, lon, different_height[i]
                            )
                            vehicles[i].simple_goto(point1)
                    if index == b"stop":
                        index = "data"
                        aggregate_flag = False
                        break

        if (data == b"home") or (home_flag):
            if master_flag:
                uav_home_pos = []
                for vehicle in vehicles:
                    lat = vehicle.location.global_relative_frame.lat
                    lon = vehicle.location.global_relative_frame.lon

                    # Process the lat and lon as needed
                    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
                    x, y = locatePosition.geoToCart(origin, endDistance, [lat, lon])
                    # print("x,y",x/2,y/2)
                    uav_home_pos.append((x / 2, y / 2))
                origin = read_origin(file_name)
                s = sim.Simulation(
                    uav_home_pos,
                    num_bots=len(pos_array),
                    env_name=file_name,
                    speed=bot_speed,
                )
                home_flag = True

            if previous_task == b"specificbotgoal":
                specific_bot_goal_flag_array = [False] * len(s.swarm)

            # FIX: Use len(s.swarm) not num_bots, which can be stale after a drone removal.
            bot_array_home = [0] * len(s.swarm)
            all_bot_reach_flag_home = False
            print("Home.......", home_pos)
            if home_pos == []:
                home_lock()
            index = "data"
            while 1:
                time.sleep(sleep_times.get(len(s.swarm), 0.1))
                if home_flag1:
                    home_flag1 = False
                    home_goto_flag = False
                    break
                for i, b in enumerate(s.swarm):
                    current_position = [b.x, b.y]
                    cmd = cvg.home_area_cvg(i, b, home_pos[i])
                    goal = home_pos[i]
                    cmd.exec(b)
                    dx = abs(goal[0] - current_position[0])
                    dy = abs(goal[1] - current_position[1])
                    if dx <= 0.5 and dy <= 0.5:
                        bot_array_home[i] = 1
                    count_home = 0
                    for j in range(len(s.swarm)):
                        if bot_array_home[j] == 1:
                            count_home += 1
                        if count_home == len(s.swarm):
                            count_home = 0
                            bot_array_home = [0] * len(s.swarm)
                            all_bot_reach_flag_home = True

                    # if all_bot_reach_flag_home == True:
                    #     with open(csv_path, "a") as csvfile:
                    #         fieldnames = ["waypoint"]
                    #         writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    #         writer.writerow({"waypoint": -1})
                    #     home_flag1 = True

                    if master_flag:
                        current_position = [b.x * 2, b.y * 2]
                        lat, lon = locatePosition.cartToGeo(
                            origin, endDistance, current_position
                        )
                        point1 = LocationGlobalRelative(lat, lon, different_height[i])
                        vehicles[i].simple_goto(point1)

                if index == b"stop":
                    home_flag1 = True
                    home_flag = False
                    previous_task = b""
                    previous_task_flag = False

        if data == b"land":
            for i, b in enumerate(s.swarm):
                if pop_flag_arr[i] == 0:
                    i += 1
                vehicles[i].mode = VehicleMode("LAND")
                vehicles[i].close()
            break

        if (data == b"home_goto") or (home_goto_flag):
            bot_array_home = [0] * num_bots
            all_bot_reach_flag_home = False
            origin = read_origin(file_name)
            s = sim.Simulation(
                home_pos, num_bots=num_bots, env_name=file_name, speed=bot_speed
            )

            if previous_task == b"specificbotgoal":
                specific_bot_goal_flag_array = [False] * num_bots

            for i, b in enumerate(s.swarm):
                if master_flag:
                    for i, b in enumerate(s.swarm):
                        lat, lon = locatePosition.cartToGeo(
                            origin,
                            endDistance,
                            [home_pos[i][0] * 2, home_pos[i][1] * 2],
                        )
                        point1 = LocationGlobalRelative(lat, lon, different_height[i])
                        vehicles[i].simple_goto(point1)

            if index == b"stop":
                home_goto_flag = False
                start_flag = False
                home_flag = False
                closing_flag = True
                # break

        if data == b"close" or (closing_flag):
            for i, b in enumerate(s.swarm):
                vehicles[i].close()
                # print("Closing vehicle", i)
            break
    except BlockingIOError:
        time.sleep(0.1)
        pass
    except Exception as e:
        print("Global Error Caught in main loop:", e)
        import traceback

        traceback.print_exc()
        if search_flag:
            search_flag = False
        if split_flag:
            split_flag = False
        if start_flag:
            start_flag = False
        if home_flag:
            home_flag = False
        if home_goto_flag:
            home_goto_flag = False
        if disperse_flag:
            disperse_flag = False
        if aggregate_flag:
            aggregate_flag = False
        if closing_flag:
            closing_flag = False
        pass
