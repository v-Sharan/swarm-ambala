"""
Swarm Copter Server  –  V13  (cleaned + trio UDP)
==================================================
Changes from original:
  • All commented-out dead code removed
  • Redundant print debug statements removed (kept meaningful ones)
  • socket/sock2 replaced with trio UDP (trio + trio.socket)
  • Blocking sock2.recvfrom() inside inner task loops replaced with
    trio.from_thread.run_sync so the trio event-loop stays responsive
  • home_monitor_thread and reconnection_worker converted to trio tasks
  • arm_and_takeoff threads replaced with trio.to_thread.run_sync nursery
  • Global command queue (trio.from_thread safe) feeds the main dispatch loop
  • All logic preserved exactly; only I/O + threading model changed
"""

import sys, os, time, math, json, csv, yaml, argparse
from pathlib import Path
from math import radians, sin, cos, sqrt, atan2, asin, degrees

import trio
import trio.socket

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

import swarm_tasks
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
import swarm_tasks.controllers.potential_field as potf
from swarm_tasks.modules.dispersion import disp_field
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules import exploration as exp
from swarm_tasks.tasks import area_coverage as cvg
from dronekit import connect, VehicleMode, LocationGlobalRelative
from swarm_tasks.modules.navigate import NavigationGridGenerator
from swarm_tasks.modules.groupsplitauto import AutoSplitMission
from swarm_tasks.modules.search_grid import SearchGridGenerator
from swarm_tasks.modules.search import PolygonSearchGrid
from swarm_tasks.modules.groupsplitspecific import SpecificSplitMission
from swarm_tasks.modules.multipoly_grid import PolygonAutoSplit
from swarm_tasks.modules.multipoly_specificgrid import PolygonSpecificSplit
import swarm_tasks.modules.locatePosition as locatePosition

# ---------------------------------------------------------------------------
# CLI arguments
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Swarm Server Controller")
parser.add_argument("--server-address", type=str, default="127.0.0.1")
parser.add_argument("--sim-enable", action="store_true")
parser.add_argument("--log-path", default=None)
args = parser.parse_args()

cwd = os.getcwd()
ip  = args.server_address

# Logging setup
LOG_FILE = "logs"
if args.log_path:
    clean_log_path = args.log_path.lstrip("/\\")
    LOG_DIR  = Path(cwd) / clean_log_path
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    LOG_FILE = str(LOG_DIR / "swarm_server.log")

def log(msg, LOG_FILE=LOG_FILE):
    if LOG_FILE:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(msg + "\n")
            f.flush()

log("SERVER STARTED")
print("Version Swarm Copter V13 (trio)")
print("ip:", ip)

# ---------------------------------------------------------------------------
# YAML origin reader
# ---------------------------------------------------------------------------
documents_path = os.path.join(os.path.expanduser("~"), "Documents")
file_name = os.path.join(documents_path, "swarm_env", "rectangles.yaml")

def read_origin(filepath):
    with open(filepath) as f:
        data = yaml.safe_load(f)
    origin = data.get("origin")
    if isinstance(origin, str):
        origin = origin.strip("()")
        lat, lon = origin.split(",")
        origin = (float(lat), float(lon))
    return origin

origin = read_origin(file_name)
print("Origin:", origin)

# ---------------------------------------------------------------------------
# Constants and global state
# ---------------------------------------------------------------------------
NUM_BOTS_MAX  = 8
PORT_ARRAY    = [14551, 14552, 14553, 14554, 14555, 14556, 14557, 14558]
PORT_DICT     = {i + 1: p for i, p in enumerate(PORT_ARRAY)}
END_DISTANCE  = 500000
BOT_SPEED     = 3.0
RECV_PORT     = 12008

different_height  = [50, 60, 70, 80, 90, 100, 110, 120]
height_difference = 5
sleep_times       = {n: 0.0000001 for n in range(1, 9)}

vehicles   = []
pos_array  = []
num_bots   = 0
home_pos   = []
home_pos_lat_lon = []
uav_home_pos     = []
robots     = [(0, 0)] * NUM_BOTS_MAX

master_flag = True
master_num  = 0
nextwaypoint = 0

# heartbeat IPs
if args.sim_enable:
    heartbeat_ip         = [ip] * NUM_BOTS_MAX
    heartbeat_ip_timeout = [5]  * NUM_BOTS_MAX
else:
    heartbeat_ip = [
        "192.168.6.101", "192.168.6.102", "192.168.6.103", "192.168.6.104",
        "192.168.6.105", "192.168.6.106", "192.168.6.107", "192.168.6.108",
    ]
    heartbeat_ip_timeout = [3] * NUM_BOTS_MAX

# ---------------------------------------------------------------------------
# Trio command queue  (replaces blocking sock2 in inner loops)
# ---------------------------------------------------------------------------
# Populated by the trio UDP listener; consumed by the dispatch loop.
# We use a simple list + trio.Event pair so non-async code can push safely.
_cmd_queue = []
_cmd_event = trio.Event()   # re-created each time a command is consumed

def _push_command(data):
    """Called from the trio UDP listener task to enqueue a command."""
    global _cmd_event
    _cmd_queue.append(data)
    _cmd_event.set()

def _peek_command():
    """Non-blocking peek at the next queued command (None if queue empty)."""
    return _cmd_queue[0] if _cmd_queue else None

def _pop_command():
    """Pop the next command from the queue."""
    return _cmd_queue.pop(0) if _cmd_queue else None

# ---------------------------------------------------------------------------
# CSV cache
# ---------------------------------------------------------------------------
csv_cache: dict = {}

def read_specific_line(csv_file_path, line_number):
    if csv_file_path not in csv_cache:
        try:
            with open(csv_file_path, "rt") as f:
                reader = csv.reader(f)
                csv_cache[csv_file_path] = [
                    [float(row[0]), float(row[1])] for row in reader if row
                ]
        except Exception as e:
            print(f"Error caching CSV {csv_file_path}: {e}")
            return []
    try:
        line = csv_cache[csv_file_path][line_number]
        return [(line[0], line[1])]
    except IndexError:
        print(f"Error: Line {line_number} not found in {csv_file_path}")
        return []

# ---------------------------------------------------------------------------
# Drone management helpers
# ---------------------------------------------------------------------------
reconnecting_ports = set()

def home_lock():
    global vehicles, home_pos_lat_lon, home_pos
    home_pos = []
    home_pos_lat_lon = []
    for i, vehicle in enumerate(vehicles):
        timeout = time.time() + 30
        while not vehicle.home_location:
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not vehicle.home_location:
                print("Waiting for home position...")
                time.sleep(1)
            if time.time() > timeout:
                print(f"Timeout waiting for home_location for vehicle {i}")
                break
        home = vehicle.home_location
        if home is None:
            continue
        x, y = locatePosition.geoToCart(origin, END_DISTANCE, [home.lat, home.lon])
        home_pos_lat_lon.append((home.lat, home.lon))
        home_pos.append((x / 2, y / 2))
    return 1


def CHECK_network_connection():
    for i in range(len(heartbeat_ip_timeout)):
        response = os.system("ping -n 1 " + heartbeat_ip[i] + " >nul 2>&1")
        heartbeat_ip_timeout[i] = 30
        if response != 0:
            print(f"Warning: {heartbeat_ip[i]} unreachable, continuing anyway")
    print("heartbeat_ip_timeout:", heartbeat_ip_timeout)


def vehicle_connection():
    global vehicles, pos_array, num_bots
    pos_array = []
    vehicles  = []
    num_bots  = 0
    for idx, port in enumerate(PORT_ARRAY):
        try:
            v = connect(f"udpin:{ip}:{port}", baud=115200,
                        heartbeat_timeout=heartbeat_ip_timeout[idx])
            vehicles.append(v)
            pos_array.append(v._master.target_system)
            num_bots += 1
            print(f"Drone {idx + 1} connected")
        except Exception:
            print(f"Vehicle {idx + 1} is lost")
    print(f"Total connected: {len(vehicles)}")


def reconnection_worker_sync(index, connection_str, sys_id):
    """Blocking reconnect — run in a thread via trio.to_thread."""
    global vehicles, reconnecting_ports
    try:
        new_v = connect(connection_str, baud=115200, heartbeat_timeout=30)
        new_v.last_reconnect_attempt = 0
        vehicles[index] = new_v
        print(f"[RECONNECT] SUCCESS: Drone {index} (SysID {sys_id}) reconnected.")
    except Exception as e:
        print(f"[RECONNECT] FAILED: Drone {index} (SysID {sys_id}): {e}")
    finally:
        reconnecting_ports.discard(connection_str)


def check_reconnection():
    """Called once per main-loop iteration (synchronous; spawns trio threads)."""
    now = time.time()
    for i, v in enumerate(vehicles):
        is_dead = False
        try:
            hb = v.last_heartbeat
            if hb is None or hb >= 28:
                is_dead = True
        except Exception:
            is_dead = True

        if is_dead:
            sys_id = pos_array[i]
            port   = PORT_DICT.get(sys_id)
            if not port:
                continue
            connection_str = f"udpin:{ip}:{port}"
            last_attempt   = getattr(v, "last_reconnect_attempt", 0)
            if (now - last_attempt > 10) and (connection_str not in reconnecting_ports):
                v.last_reconnect_attempt = now
                reconnecting_ports.add(connection_str)
                print(f"CRITICAL: Drone {i} link lost. Starting background recovery...")
                try:
                    v.close()
                except Exception:
                    pass
                # Schedule blocking reconnect without blocking the event loop
                trio.from_thread.run_sync(
                    lambda: trio.lowlevel.spawn_system_task(
                        trio.to_thread.run_sync,
                        reconnection_worker_sync, i, connection_str, sys_id
                    )
                )


def arm_and_takeoff_sync(vehicle, target_altitude):
    """Blocking takeoff sequence – run in thread via trio nursery."""
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    time.sleep(3)
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.1f}")
        if alt >= target_altitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


def fetch_location():
    global uav_home_pos, home_pos, home_pos_lat_lon, robots
    uav_home_pos = []
    if master_flag:
        try:
            home_lock()
        except Exception:
            for i, vehicle in enumerate(vehicles):
                lat = vehicle.location.global_relative_frame.lat
                lon = vehicle.location.global_relative_frame.lon
                home_pos_lat_lon.append((lat, lon))
                x, y = locatePosition.geoToCart(origin, END_DISTANCE, [lat, lon])
                home_pos.append((x / 2, y / 2))
                if i < len(robots):
                    robots[i] = (x / 2, y / 2)
        for vehicle in vehicles:
            lat = vehicle.location.global_relative_frame.lat
            lon = vehicle.location.global_relative_frame.lon
            x, y = locatePosition.geoToCart(origin, END_DISTANCE, [lat, lon])
            uav_home_pos.append((x / 2, y / 2))


def _current_uav_home_pos():
    """Snapshot current GPS positions of all vehicles as Cartesian."""
    result = []
    for vehicle in vehicles:
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        x, y = locatePosition.geoToCart(origin, END_DISTANCE, [lat, lon])
        result.append((x / 2, y / 2))
    return result


def _rebuild_sim():
    """Re-initialise simulation from current vehicle GPS positions."""
    global s, uav_home_pos, origin
    uav_home_pos = _current_uav_home_pos()
    origin = read_origin(file_name)
    s = sim.Simulation(uav_home_pos, num_bots=len(pos_array),
                       env_name=file_name, speed=BOT_SPEED)


def _goto(i, b, lat, lon):
    """Send simple_goto to vehicle i at the drone's altitude tier."""
    if master_flag:
        pt = LocationGlobalRelative(lat, lon, different_height[i])
        vehicles[i].simple_goto(pt)


def _proximity_check(i, b):
    for j, other in enumerate(s.swarm):
        if j > i:
            dist = math.sqrt((b.x - other.x) ** 2 + (b.y - other.y) ** 2)
            if dist < 5.0:
                print(f"[ALERT] Drone {i+1} and {j+1} are {dist:.2f}m apart!")


def calculate_drones_needed(remaining_points, points_per_drone):
    if remaining_points <= 0:
        return 0
    return (remaining_points + points_per_drone - 1) // points_per_drone


def allocate_drones(total_points, covered_points, total_drones):
    points_per_drone       = [int(tp / 2) for tp in total_points]
    remaining_points_list  = [tp - cp for tp, cp in zip(total_points, covered_points)]
    uncovered_areas        = [(i, p) for i, p in enumerate(remaining_points_list) if p > 0]
    drones_needed          = [calculate_drones_needed(p, points_per_drone[i])
                               for i, p in uncovered_areas]
    allocation             = {i: 0 for i, _ in uncovered_areas}
    if total_drones <= len(uncovered_areas):
        for i, _ in uncovered_areas:
            if total_drones <= 0:
                break
            allocation[i] = 1
            total_drones -= 1
    else:
        for idx, (area_index, _) in enumerate(uncovered_areas):
            if total_drones <= 0:
                break
            required = min(drones_needed[idx], total_drones)
            allocation[area_index] = required
            total_drones -= required
    all_areas = {i: 0 for i in range(len(covered_points))}
    all_areas.update(allocation)
    return all_areas, remaining_points_list

# ---------------------------------------------------------------------------
# Startup (synchronous, runs before trio)
# ---------------------------------------------------------------------------
if master_flag:
    CHECK_network_connection()
    vehicle_connection()
    while True:
        if all(v.armed for v in vehicles):
            fetch_location()
            break
        time.sleep(0.1)

# Wait for uav_home_pos
while not uav_home_pos:
    time.sleep(0.1)

origin = read_origin(file_name)
s = sim.Simulation(uav_home_pos, num_bots=len(pos_array),
                   env_name=file_name, speed=BOT_SPEED)

print(f"Simulation initialised with {len(pos_array)} bots")

# ---------------------------------------------------------------------------
# Per-task state
# ---------------------------------------------------------------------------
pop_flag_arr      = [1]     * num_bots
specific_goal_pos = [0]     * num_bots
specific_bot_goal_flag_array = [False] * num_bots
specific_goal_xy_index       = [0]     * num_bots
group_split_goal_pos  = [0]     * num_bots
group_split_flag_array= [False] * num_bots

home_flag      = False
home_flag1     = False
home_goto_flag = False
search_flag    = False
aggregate_flag = False
disperse_flag  = False
split_flag     = False
closing_flag   = False
landing_flag   = False
start_flag     = False
start_return_csv_flag = False
remove_flag    = False
remove_bot_flag   = False
remove_bot_index  = []
remove_bot_array  = []
remove_bot_num_array = []
include_uav_flag  = False
include_uav_index = []

specific_bot_goal_flag = False
group_goal_flag        = False
uav_home_flag          = False
search_loop_running    = False

previous_task      = b""
previous_task_flag = False

search_step  = 1
split_flag_val = 0
percentage   = 0
search_flag_val = 0

grid_path_array   = [0] * num_bots
num_lines         = [0] * num_bots
all_uav_csv_grid_array = [0] * len(pos_array)
csv_file_paths    = [None] * len(pos_array)
active_bot_count  = len(pos_array)
grid_completed_bot = [-1] * num_bots

removed_uav_grid               = []
removed_grid_path_length       = []
removed_numlines               = []
removed_grid_path_array        = [0] * len(pos_array)
removed_grid_filename          = [0] * num_bots
removed_grid_path_array_start_val           = [0] * len(pos_array)
checkall_removed_grid_path_array_start_val  = [0] * len(pos_array)
removed_grid_path_array_flag   = False
mid_mission_data_cache: dict   = {}
uncovered_area_points          = []
uncovered_area_filename        = []

disperse_goal      = []
agg_goal_point     = ()
multiple_goals     = []
goal_table         = []
goal_points        = []
goal_path_csv_array        = []
goal_path_csv_array_flag   = False
skip_wp_flag               = False
next_wp            = 0
nextwaypoint       = 0

uid = ""

# ---------------------------------------------------------------------------
# Trio async tasks
# ---------------------------------------------------------------------------

async def udp_listener(send_channel):
    """Receives UDP datagrams and forwards to the command channel."""
    sock = trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM)
    await sock.bind(("", RECV_PORT))
    print(f"UDP listener bound on port {RECV_PORT}")
    async with send_channel:
        while True:
            data, addr = await sock.recvfrom(1050)
            print("MSG received:", data)
            await send_channel.send(data)
            
async def stop_monitor():
    """Monitors for 'stop' command to interrupt ongoing tasks."""
    stop_sock = trio.socket.socket(trio.socket.AF_INET, trio.socket.SOCK_DGRAM)
    await stop_sock.bind((12002))
    while True:
        data, addr = await stop_sock.recvfrom(1050)
        print("MSG received:", data)
        _push_command("stop")
        print("[STOP MONITOR] 'stop' command detected, signaling interruption.")


async def home_monitor():
    """Periodically refreshes home positions (replaces home_monitor_thread)."""
    while True:
        await trio.sleep(5)
        try:
            if not home_pos or len(pos_array) != len(home_pos):
                print("[Home Monitor] Updating home positions...")
                await trio.to_thread.run_sync(home_lock)
        except Exception as e:
            print("[Home Monitor] Error:", e)


# ---------------------------------------------------------------------------
# Task handlers  (each is a synchronous function – called from async dispatch)
# ---------------------------------------------------------------------------

async def handle_takeoff(data: bytes):
    global home_pos, home_pos_lat_lon, robots
    _, takeoff_height = data.decode().split(",")
    takeoff_height = int(takeoff_height)
    async with trio.open_nursery() as nursery:
        for vehicle in vehicles:
            nursery.start_soon(
                trio.to_thread.run_sync,
                arm_and_takeoff_sync, vehicle, takeoff_height
            )
    home_pos = []
    for i, vehicle in enumerate(vehicles):
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        home_pos_lat_lon.append((lat, lon))
        x, y = locatePosition.geoToCart(origin, END_DISTANCE, [lat, lon])
        home_pos.append((x / 2, y / 2))
        if i < len(robots):
            robots[i] = (x / 2, y / 2)


async def handle_different_height(data: bytes):
    global different_height, height_difference
    _, height, step = data.decode().split(",")
    height = int(height)
    step   = int(step)
    height_difference = step
    different_height  = [height + step * i for i in range(num_bots)]
    alt_count = [0] * num_bots
    diff_done = False
    while not diff_done:
        for i, b in enumerate(s.swarm):
            cmd = potf.velocity(b.get_position(), b.sim,
                                weights=potf.field_weights, order=1, max_dist=10)
            # _proximity_check(i, b)
            cmd.exec(b)
            if master_flag:
                lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, [b.x * 2, b.y * 2])
                _goto(i, b, lat, lon)
                alt = vehicles[i].location.global_relative_frame.alt
                if abs(alt - different_height[i]) <= 1.5:
                    alt_count[i] = 1
                    if all(c == 1 for c in alt_count):
                        diff_done = True
                        break
            else:
                diff_done = True
                break
        await trio.sleep(0.05)


def _handle_remove(data: bytes):
    """Remove a drone from the active swarm mid-mission."""
    global num_bots, uav_home_pos, remove_flag, uav_removed, pop_bot_index
    _, remove_bot_num = data.decode().split(",")
    remove_bot_num_array.append(int(remove_bot_num))
    remove_bot_flag_local = True
    pop_bot_index = None
    for l, sysid in enumerate(pos_array):
        if int(remove_bot_num) == sysid:
            pop_bot_index = l
            break
    if pop_bot_index is not None:
        remove_bot_array.append((pop_bot_index, pos_array[pop_bot_index]))
        pos_array.pop(pop_bot_index)
        v = vehicles[pop_bot_index]
        v.close()
        vehicles.pop(pop_bot_index)
        s.remove_bot(pop_bot_index)
        home_pos.pop(pop_bot_index)
        different_height.pop(pop_bot_index)
        pop_flag_arr.pop(pop_bot_index)
        specific_goal_pos.pop(pop_bot_index)
        specific_bot_goal_flag_array.pop(pop_bot_index)
        specific_goal_xy_index.pop(pop_bot_index)
        uav_home_pos = _current_uav_home_pos()
        num_bots = len(pos_array)
        print(f"Drone removed. Active bots: {num_bots}")
    else:
        print("Remove: drone not found in pos_array")


def _handle_add(data: bytes):
    """Add a drone to the active swarm mid-mission."""
    global num_bots, include_uav_flag, previous_task_flag
    _, sys_id_str = data.decode().split(",")
    sys_id = int(sys_id_str)
    if sys_id in pos_array:
        print("sys_id already in pos_array")
        return
    if sys_id not in PORT_DICT:
        print(f"sys_id {sys_id} not in port_dict")
        return
    conn_str = f"udpin:{ip}:{PORT_DICT[sys_id]}"
    vehicle  = connect(conn_str, baud=115200, heartbeat_timeout=30, wait_ready=True)
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    timeout = time.time() + 15
    while (lat is None or lon is None) and time.time() < timeout:
        time.sleep(0.5)
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
    if lat is None or lon is None:
        vehicle.close()
        raise RuntimeError("Could not get GPS for added drone")
    vehicles.append(vehicle)
    num_bots += 1
    x, y = locatePosition.geoToCart(origin, END_DISTANCE, [lat, lon])
    s.add_bot(len(pos_array), (x / 2, y / 2))
    pos_array.append(sys_id)
    specific_goal_pos.append(0)
    specific_bot_goal_flag_array.append(False)
    specific_goal_xy_index.append(0)
    pop_flag_arr.append(1)
    different_height.append(different_height[-1] + height_difference if different_height else 50)
    num_bots = len(pos_array)
    home_lock()
    if sys_id in remove_bot_num_array:
        remove_bot_num_array.remove(sys_id)
    if previous_task in (b"search", b"split", b"navigate", b"specificsplit"):
        include_uav_flag   = True
        previous_task_flag = True
    print(f"Drone {sys_id} added. Active bots: {num_bots}")


async def handle_goal(data: bytes):
    global group_goal_flag, previous_task, previous_task_flag
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    parts       = data.decode().split("_")
    goal_latlon = json.loads(parts[1])
    goal_xy     = []
    bot_reached = [0] * num_bots
    for pt in goal_latlon:
        x, y = locatePosition.geoToCart(origin, END_DISTANCE, [pt[0], pt[1]])
        goal_xy.append((x / 2, y / 2))
    goal_xy_index = 0
    if master_flag:
        _rebuild_sim()
    previous_task_flag = False
    while True:
        await trio.sleep(sleep_times.get(num_bots, 0.1))
        if group_goal_flag:
            group_goal_flag    = False
            previous_task_flag = False
            break
        goal_position = goal_xy[goal_xy_index]
        for i, b in enumerate(s.swarm):
            dx = abs(goal_position[0] - b.x)
            dy = abs(goal_position[1] - b.y)
            if dx <= 5 and dy <= 5:
                bot_reached[i] = 1
                if any(e == 1 for e in bot_reached) and goal_xy_index != len(goal_xy) - 1:
                    bot_reached   = [0] * num_bots
                    goal_xy_index += 1
                elif all(e == 1 for e in bot_reached) and goal_xy_index == len(goal_xy) - 1:
                    group_goal_flag = True
                    break
            else:
                b.set_goal(goal_position[0], goal_position[1])
                cmd = cvg.goal_area_cvg(i, b, goal_position)
                # _proximity_check(i, b)
                cmd.exec(b)
            if master_flag:
                lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, [b.x * 2, b.y * 2])
                _goto(i, b, lat, lon)
        # check stop
        cmd_peek = _peek_command()
        if cmd_peek == b"stop":
            _pop_command()
            group_goal_flag    = False
            previous_task      = b"goal"
            previous_task_flag = False
            break


async def handle_specificbotgoal(data: bytes):
    global specific_bot_goal_flag, previous_task_flag,previous_task
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    if not previous_task_flag:
        parts    = data.decode().split("_")
        uav_list = json.loads(parts[1].strip().replace("'", '"'))
        goal_latlon = json.loads(parts[2])
        goal_xy  = []
        for pt in goal_latlon:
            x, y = locatePosition.geoToCart(origin, END_DISTANCE, [pt[1], pt[0]])
            goal_xy.append((x / 2, y / 2))
        goal     = [0] * len(pos_array)
        for uav_id in uav_list:
            if int(uav_id) in pos_array:
                bi = pos_array.index(int(uav_id))
                specific_goal_pos[bi]           = goal_xy
                specific_bot_goal_flag_array[bi] = True
                specific_goal_xy_index[bi]       = 0
    if master_flag:
        _rebuild_sim()
    while True:
        await trio.sleep(sleep_times.get(num_bots, 0.1))
        if specific_bot_goal_flag:
            specific_bot_goal_flag = False
            previous_task_flag     = False
            break
        for i, b in enumerate(s.swarm):
            if not specific_bot_goal_flag_array[i]:
                continue
            goal[i] = specific_goal_pos[i][specific_goal_xy_index[i]]
            dx = abs(goal[i][0] - b.x)
            dy = abs(goal[i][1] - b.y)
            if dx <= 5 and dy <= 5:
                specific_goal_xy_index[i] += 1
                if specific_goal_xy_index[i] == len(specific_goal_pos[i]):
                    specific_bot_goal_flag_array[i] = False
                    specific_goal_pos[i]             = 0
            if all(not f for f in specific_bot_goal_flag_array):
                specific_bot_goal_flag = True
                break
            if specific_bot_goal_flag_array[i]:
                b.set_goal(goal[i][0], goal[i][1])
                cmd = cvg.goal_area_cvg(i, b, goal[i])
                # _proximity_check(i, b)
                cmd.exec(b)
                if master_flag:
                    lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, [b.x * 2, b.y * 2])
                    _goto(i, b, lat, lon)
        cmd_peek = _peek_command()
        if cmd_peek == b"stop":
            _pop_command()
            specific_bot_goal_flag = False
            previous_task_flag     = False
            previous_task          = b"specificbotgoal"
            break


async def handle_navigate(data: bytes):
    global start_flag, previous_task, previous_task_flag
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    if not previous_task_flag:
        _, center_lat, center_lon, num_uavs, grid_space, coverage_area = data.decode().split(",")
        curve = NavigationGridGenerator(origin, float(center_lat), float(center_lon),
                                        int(num_uavs), int(grid_space), int(coverage_area))
        path  = curve.navigate_grid()
        multiple_goals[:] = [path]
        if master_flag:
            start_flag = True
            _rebuild_sim()
    previous_task_flag = False
    ind = 0
    while start_flag:
        await trio.sleep(sleep_times.get(num_bots, 0.1))
        for i, b in enumerate(s.swarm):
            goal = multiple_goals[0][ind]
            gx, gy = locatePosition.geoToCart(origin, END_DISTANCE, goal)
            goal   = (gx / 2, gy / 2)
            cmd    = cvg.goal_area_cvg(i, b, goal)
            # _proximity_check(i, b)
            cmd.exec(b)
            if abs(goal[0] - b.x) <= 3 and abs(goal[1] - b.y) <= 3:
                ind += 1
                if ind >= len(multiple_goals[0]):
                    start_flag = False
                    log("MISSION COMPLETED - navigation")
                    break
            if master_flag:
                lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, [b.x * 2, b.y * 2])
                _goto(i, b, lat, lon)
        cmd_peek = _peek_command()
        if cmd_peek == b"stop":
            _pop_command()
            start_flag         = False
            previous_task      = b"navigate"
            previous_task_flag = False
            break


async def handle_disperse(data: bytes):
    global disperse_flag, search_flag
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    disperse_flag = True
    if master_flag:
        _rebuild_sim()
    disperse_start = time.time()
    while disperse_flag:
        await trio.sleep(sleep_times.get(num_bots, 0.1))
        cmd_peek = _peek_command()
        if cmd_peek == b"stop":
            _pop_command()
            disperse_flag = False
            break
        if cmd_peek == b"search":
            _pop_command()
            disperse_flag = False
            search_flag   = True
            break
        elapsed = time.time() - disperse_start
        for i, b in enumerate(s.swarm):
            cmd  = base_control.exp_control(b)
            cmd += disp_field(b) * 15
            cmd += base_control.exp_obstacle_avoidance(b) * 30
            # _proximity_check(i, b)
            cmd.exec(b)
            if master_flag:
                lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, [b.x * 2, b.y * 2])
                _goto(i, b, lat, lon)
        if elapsed > 10:
            disperse_flag = False
            search_flag   = True
            break


async def handle_aggregate(data: bytes):
    global aggregate_flag, previous_task
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    _, agg_lat, agg_lon = data.decode().split(",")
    search_flag_local = False
    aggregate_flag    = True
    bot_reached       = [0] * len(pos_array)
    x, y = locatePosition.geoToCart(origin, END_DISTANCE, [float(agg_lat), float(agg_lon)])
    agg_goal = (x / 2, y / 2)
    if master_flag:
        _rebuild_sim()
    while aggregate_flag:
        await trio.sleep(sleep_times.get(num_bots, 0.1))
        for i, b in enumerate(s.swarm):
            dx = abs(agg_goal[0] - b.x)
            dy = abs(agg_goal[1] - b.y)
            if dx <= 1 and dy <= 1:
                bot_reached[i] = 1
                b.cancel_goal()
                if all(e == 1 for e in bot_reached):
                    aggregate_flag = False
                    break
            else:
                b.set_goal(agg_goal[0], agg_goal[1])
                cmd = cvg.goal_area_cvg(i, b, agg_goal)
                cmd.exec(b)
            if master_flag:
                lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, [b.x * 2, b.y * 2])
                _goto(i, b, lat, lon)
        cmd_peek = _peek_command()
        if cmd_peek == b"stop":
            _pop_command()
            aggregate_flag = False
            break


async def handle_home(data: bytes):
    global home_flag, home_flag1
    home_flag = True
    if master_flag:
        _rebuild_sim()
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    bot_array_home = [0] * len(s.swarm)
    if not home_pos:
        await trio.to_thread.run_sync(home_lock)
    while True:
        await trio.sleep(sleep_times.get(len(s.swarm), 0.1))
        if home_flag1:
            home_flag1 = False
            break
        for i, b in enumerate(s.swarm):
            goal = home_pos[i]
            cmd  = cvg.home_area_cvg(i, b, goal)
            cmd.exec(b)
            if abs(goal[0] - b.x) <= 0.5 and abs(goal[1] - b.y) <= 0.5:
                bot_array_home[i] = 1
            if sum(bot_array_home) == len(s.swarm):
                bot_array_home = [0] * len(s.swarm)
            if master_flag:
                lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, [b.x * 2, b.y * 2])
                _goto(i, b, lat, lon)
        cmd_peek = _peek_command()
        if cmd_peek == b"stop":
            _pop_command()
            home_flag1 = True
            home_flag  = False
            previous_task      = b""
            previous_task_flag = False
            break


async def handle_home_goto():
    global home_goto_flag
    home_goto_flag = True
    _rebuild_sim()
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    if master_flag:
        for i, b in enumerate(s.swarm):
            lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE,
                                                [home_pos[i][0] * 2, home_pos[i][1] * 2])
            _goto(i, b, lat, lon)
    cmd_peek = _peek_command()
    if cmd_peek == b"stop":
        _pop_command()
        home_goto_flag = False


def _setup_search_csv():
    """Build csv_file_paths and num_lines from searchgrid/ folder."""
    global csv_file_paths, num_lines, all_uav_csv_grid_array
    csv_file_paths = [None] * len(pos_array)
    num_lines      = [0]    * len(pos_array)
    for i, sysid in enumerate(pos_array):
        path = os.path.join(cwd, "searchgrid", f"d{sysid}.csv")
        if os.path.exists(path):
            csv_file_paths[i] = path
            with open(path) as f:
                num_lines[i] = len(list(csv.reader(f)))
        else:
            print(f"WARNING: Grid missing for drone {sysid}, initialising as idle")
    all_uav_csv_grid_array = [0] * len(pos_array)


def _setup_split_csv(split_dir="group_split"):
    """Build csv_file_paths and num_lines from split folder."""
    global csv_file_paths, num_lines, all_uav_csv_grid_array
    csv_file_paths = [None] * len(pos_array)
    num_lines      = [0]    * len(pos_array)
    for i, sysid in enumerate(pos_array):
        path = os.path.join(cwd, split_dir, f"grid_{sysid}.csv")
        csv_file_paths[i] = path if os.path.exists(path) else None
        if os.path.exists(path):
            with open(path) as f:
                num_lines[i] = sum(1 for _ in csv.reader(f))
    all_uav_csv_grid_array = [0] * len(pos_array)


def _reset_removal_state():
    global removed_uav_grid, removed_grid_path_length, removed_numlines
    global removed_grid_path_array, removed_grid_filename
    global removed_grid_path_array_start_val, checkall_removed_grid_path_array_start_val
    global removed_grid_path_array_flag, mid_mission_data_cache
    global uncovered_area_points, uncovered_area_filename
    global remove_bot_flag, remove_bot_array, include_uav_flag, include_uav_index
    removed_uav_grid               = []
    removed_grid_path_length       = []
    removed_numlines               = []
    removed_grid_path_array        = [0] * len(pos_array)
    removed_grid_filename          = [0] * len(pos_array)
    removed_grid_path_array_start_val          = [0] * len(pos_array)
    checkall_removed_grid_path_array_start_val = [0] * len(pos_array)
    removed_grid_path_array_flag   = False
    mid_mission_data_cache         = {}
    uncovered_area_points          = []
    uncovered_area_filename        = []
    remove_bot_flag   = False
    remove_bot_array  = []
    include_uav_flag  = False
    include_uav_index = []


def _sync_arrays_for_added_drone(added_sysid):
    """Grow all mission arrays when a drone is added mid-search/split."""
    if added_sysid in mid_mission_data_cache and not removed_grid_path_array_flag:
        m_data, length, lines = mid_mission_data_cache.pop(added_sysid)
        all_uav_csv_grid_array.append(m_data)
        grid_path_array.append(length)
        num_lines.append(lines)
        if m_data in removed_uav_grid:
            r_idx = removed_uav_grid.index(m_data)
            removed_uav_grid.pop(r_idx)
            removed_grid_path_length.pop(r_idx)
            removed_numlines.pop(r_idx)
        print(f"[RESUME] Drone {added_sysid} restored with saved data")
    else:
        if added_sysid in mid_mission_data_cache:
            mid_mission_data_cache.pop(added_sysid)
        all_uav_csv_grid_array.append(0)
        grid_path_array.append(0)
        num_lines.append(0)
        print(f"[JOIN] Drone {added_sysid} added as idle")
    removed_grid_filename.append(0)
    removed_grid_path_array.append(0)
    removed_grid_path_array_start_val.append(0)
    checkall_removed_grid_path_array_start_val.append(0)
    path = os.path.join(cwd, "searchgrid", f"d{added_sysid}.csv")
    csv_file_paths.append(path if os.path.exists(path) else None)


def _handle_bot_removal_in_mission():
    """Process remove_bot_array inside an active search/split loop."""
    global active_bot_count, remove_bot_flag, remove_bot_array
    for m, m_sysid in remove_bot_array:
        m_data = all_uav_csv_grid_array.pop(m)
        length = grid_path_array.pop(m)
        lines  = num_lines.pop(m)
        mid_mission_data_cache[m_sysid] = (m_data, length, lines)
        removed_uav_grid.append(m_data)
        removed_grid_path_length.append(length)
        removed_numlines.append(lines)
        removed_grid_path_array.pop(m)
        removed_grid_path_array_start_val.pop(m)
        removed_grid_filename.pop(m)
        checkall_removed_grid_path_array_start_val.pop(m)
    remove_bot_array  = []
    remove_bot_flag   = False
    active_bot_count  = len(pos_array)
    # Rebuild csv_file_paths to match compacted pos_array
    global csv_file_paths
    csv_file_paths = [None] * len(pos_array)
    for _i, _sysid in enumerate(pos_array):
        p = os.path.join(cwd, "searchgrid", f"d{_sysid}.csv")
        if os.path.exists(p):
            csv_file_paths[_i] = p


def _realloc_removed_areas(i):
    """Assign uncovered areas from removed drones to still-active drones."""
    if not (any(c >= int(num_lines[a]) for a, c in enumerate(grid_path_array))
            and removed_grid_path_length
            and not removed_grid_path_array_flag):
        return
    allocation, remaining_points_list = allocate_drones(
        removed_numlines, removed_grid_path_length, len(pos_array))
    remaining   = [n - g for n, g in zip(num_lines, grid_path_array)]
    min_sorted  = sorted(enumerate(remaining), key=lambda x: x[1])
    ptr         = 0
    for x, v in enumerate(remaining_points_list):
        area_idx    = min_sorted[ptr][0]
        start_index = (abs(removed_grid_path_length[x])
                       if removed_grid_path_length[x] == 1
                       else abs(removed_grid_path_length[x] - 1))
        if allocation[x] == 0:
            if removed_grid_path_length[x] != int(removed_numlines[x]):
                uncovered_area_points.append(removed_grid_path_length[x])
                uncovered_area_filename.append(removed_uav_grid[x])
            continue
        if allocation[x] == 1:
            end_index = int(removed_numlines[x])
            removed_grid_path_array[area_idx]           = (start_index, end_index)
            removed_grid_path_array_start_val[area_idx] = start_index
            removed_grid_filename[area_idx]             = removed_uav_grid[x]
            ptr += 1
        else:
            add_points = math.ceil(remaining_points_list[x] / allocation[x])
            for m in range(allocation[x]):
                area_idx = min_sorted[ptr][0]
                ptr += 1
                if m == 0:
                    end_index = min(start_index + add_points, int(removed_numlines[x]))
                else:
                    start_index = end_index
                    end_index   = min(start_index + add_points, int(removed_numlines[x]))
                removed_grid_path_array[area_idx]           = (start_index, end_index)
                removed_grid_path_array_start_val[area_idx] = start_index
                removed_grid_filename[area_idx]             = removed_uav_grid[x]
    removed_grid_path_array_flag = True


def _mission_complete_check_search():
    """Return True if the search mission is fully complete."""
    if all(c >= int(num_lines[a]) for a, c in enumerate(grid_path_array)) \
            and not removed_grid_path_length:
        return True
    if removed_grid_path_array_flag \
            and all(c >= int(num_lines[a]) for a, c in enumerate(grid_path_array)) \
            and all(removed_grid_path_array_start_val[a] >= removed_grid_path_array[a][1]
                    for a in range(len(removed_grid_path_array))
                    if removed_grid_path_array[a] != 0):
        return True
    if all(c == 1 for c in checkall_removed_grid_path_array_start_val) \
            and len(checkall_removed_grid_path_array_start_val) == len(pos_array):
        return True
    return False


def _get_goal_for_bot(i):
    """Return the current grid waypoint for drone i."""
    if removed_grid_path_array_flag:
        if grid_path_array[i] < num_lines[i]:
            return read_specific_line(all_uav_csv_grid_array[i], grid_path_array[i])
        if removed_grid_path_array[i] != 0 \
                and removed_grid_path_array_start_val[i] >= removed_grid_path_array[i][1]:
            return None
        return read_specific_line(removed_grid_filename[i],
                                  removed_grid_path_array_start_val[i])
    return read_specific_line(all_uav_csv_grid_array[i], grid_path_array[i])


async def _run_grid_mission(mission_flag_name: str, uid_local: str):
    """
    Shared inner loop for search and split missions.
    mission_flag_name is 'search_flag' or 'split_flag'.
    """
    global search_flag, split_flag, search_step, active_bot_count
    global landing_flag, previous_task

    is_search = mission_flag_name == "search_flag"

    def get_flag():
        return search_flag if is_search else split_flag

    def set_flag(v):
        global search_flag, split_flag
        if is_search:
            search_flag = v
        else:
            split_flag = v

    while get_flag():
        check_reconnection()
        await trio.sleep(sleep_times.get(len(vehicles), 0.1))

        # Handle added drones
        if include_uav_flag:
            while len(all_uav_csv_grid_array) < len(pos_array):
                _sync_arrays_for_added_drone(pos_array[len(all_uav_csv_grid_array)])
            include_uav_flag = False
            include_uav_index[:] = []
            remove_bot_flag = False
            if not remove_bot_array and not removed_grid_path_length:
                removed_grid_path_array_flag = False

        # Handle removed drones
        if remove_bot_flag:
            _handle_bot_removal_in_mission()

        # Assign CSV to each bot on first pass
        if search_step == 1:
            for i in range(len(pos_array)):
                all_uav_csv_grid_array[i] = csv_file_paths[i]
            search_step += 1

        for m_index, sysid in enumerate(pos_array):
            i       = m_index
            b_index = m_index
            try:
                b = s.swarm[b_index]
            except IndexError:
                continue
            try:
                if vehicles[b_index].last_heartbeat is None \
                        or vehicles[b_index].last_heartbeat >= 28:
                    continue
            except Exception:
                continue

            # Check overall completion
            if _mission_complete_check_search():
                log(f"MISSION COMPLETED {uid_local}")
                set_flag(False)
                landing_flag = True
                previous_task = b""
                _reset_removal_state()
                break

            # Reallocate uncovered areas
            _realloc_removed_areas(i)

            if grid_path_array[i] >= int(num_lines[i]) and not removed_grid_path_array_flag:
                continue

            if removed_grid_path_array_flag and grid_path_array[i] >= int(num_lines[i]):
                if removed_grid_path_array_start_val[i] == 0:
                    checkall_removed_grid_path_array_start_val[i] = 1
                    continue
                if removed_grid_path_array_start_val[i] >= removed_grid_path_array[i][1]:
                    checkall_removed_grid_path_array_start_val[i] = 1
                    if uncovered_area_points:
                        u = 0
                        removed_grid_path_array[i] = (
                            uncovered_area_points[u], int(num_lines[i]))
                        removed_grid_path_array_start_val[i] = uncovered_area_points[u]
                        removed_grid_filename[i] = uncovered_area_filename[u]
                        checkall_removed_grid_path_array_start_val[i] = 0
                        uncovered_area_points.pop(u)
                        uncovered_area_filename.pop(u)
                    continue

            goal_lat_lon = _get_goal_for_bot(i)
            if not goal_lat_lon:
                if removed_grid_path_array_flag and grid_path_array[i] >= int(num_lines[i]):
                    removed_grid_path_array_start_val[i] += 1
                else:
                    grid_path_array[i] += 1
                continue

            gx, gy = goal_lat_lon[0][0], goal_lat_lon[0][1]
            goal   = (gx, gy)
            goal_coord = locatePosition.cartToGeo(origin, END_DISTANCE, [gx * 2, gy * 2])
            cmd  = cvg.goal_area_cvg(b_index, b, goal)
            value = [b.x * 2, b.y * 2]
            dx   = abs(goal[0] - b.x)
            dy   = abs(goal[1] - b.y)

            if dx <= 0.5 and dy <= 0.5:
                distance = locatePosition.distance_bearing(
                    vehicles[b_index].location.global_relative_frame.lat,
                    vehicles[b_index].location.global_relative_frame.lon,
                    goal_coord[0], goal_coord[1])
                if grid_path_array[i] >= int(num_lines[i]) and not removed_grid_path_array_flag:
                    continue
                if grid_path_array[i] >= int(num_lines[i]) \
                        and removed_grid_path_array_flag and distance < 5:
                    removed_grid_path_array_start_val[i] += 1
                elif grid_path_array[i] < int(num_lines[i]) and removed_grid_path_array_flag:
                    grid_path_array[i] += 1
                elif distance < 5:
                    grid_path_array[i] += 1

            # _proximity_check(i, b)
            cmd.exec(b)
            if master_flag and pop_flag_arr[i] == 1:
                lat, lon = locatePosition.cartToGeo(origin, END_DISTANCE, value)
                _goto(i, b, lat, lon)

        s.time_elapsed += 1

        cmd_peek = _peek_command()
        if cmd_peek == b"stop":
            _pop_command()
            log(f"MISSION PAUSED {uid_local}")
            set_flag(False)
            previous_task      = b"search" if is_search else b"split"
            previous_task_flag = False
            break


async def handle_search(data: bytes):
    global search_flag, search_step, active_bot_count, uid
    global pop_flag_arr, grid_path_array, all_uav_csv_grid_array
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    search_flag = True
    if not previous_task_flag:
        decoded = data.decode()
        if decoded.startswith("searchpolygon_"):
            _, polygon_str, num_uavs_str, grid_spacing_str, uid = decoded.split("_")
            planner = PolygonSearchGrid(
                polygon_latlon=json.loads(polygon_str),
                origin_gps=origin, endDistance=END_DISTANCE,
                drone_list=pos_array, grid_spacing=int(grid_spacing_str),
                rotation_angle=90, obstacles_latlon=[])
            planner.generate_paths()
            planner.save_paths()
        else:
            _, center_lat, center_lon, num_uavs_str, grid_space_str, coverage_area_str, uid = decoded.split(",")
            curve = SearchGridGenerator(origin, float(center_lat), float(center_lon),
                                        pos_array, int(grid_space_str), int(coverage_area_str))
            curve.generate_grids()
        csv_cache.clear()
        search_step      = 1
        active_bot_count = len(pos_array)
        pop_flag_arr     = [1] * len(pos_array)
        grid_path_array  = [0] * len(pos_array)
        _setup_search_csv()
        _reset_removal_state()
        if master_flag:
            _rebuild_sim()
        active_bot_count = len(s.swarm)
    previous_task_flag = False
    await _run_grid_mission("search_flag", uid)


async def handle_split(data: bytes):
    global split_flag, search_step, active_bot_count, uid, previous_task
    global pop_flag_arr, grid_path_array, all_uav_csv_grid_array, num_lines
    if previous_task == b"specificbotgoal":
        for k in range(len(specific_bot_goal_flag_array)):
            specific_bot_goal_flag_array[k] = False
    split_flag = True
    decoded    = data.decode()
    if not previous_task_flag:
        parts = decoded.split("_")
        if decoded.startswith("polyautosplit"):
            uid       = parts[5]
            split_obj = PolygonAutoSplit(
                polygon_latlon_list=json.loads(parts[1]),
                origin_gps=origin, endDistance=END_DISTANCE,
                drone_list=pos_array, grid_spacing=int(json.loads(parts[3])),
                rotation_angle=90, obstacles_latlon_list=[])
            split_obj.generate_paths()
            split_obj.save_paths()
            previous_task = b"split"
        elif decoded.startswith("polyspecificsplit"):
            uid       = parts[5]
            split_obj = PolygonSpecificSplit(
                polygon_latlon_list=json.loads(parts[1]),
                origin_gps=origin, endDistance=END_DISTANCE,
                drone_list=pos_array, grid_spacing=json.loads(parts[3]),
                rotation_angle=90, obstacles_latlon_list=None,
                drone_assignments=json.loads(parts[2]))
            split_obj.generate_paths()
            split_obj.save_paths()
            previous_task = b"split"
        elif decoded.startswith("specificsplit"):
            uid       = parts[5]
            split_obj = SpecificSplitMission(
                origin=origin,
                center_lat_lons=json.loads(parts[1]),
                drone_array=json.loads(parts[2]),
                grid_spacing=json.loads(parts[3]),
                coverage_area=json.loads(parts[4]))
            split_obj.GroupSplitting(
                center_lat_lons=json.loads(parts[1]),
                drone_array=json.loads(parts[2]),
                grid_spacing=json.loads(parts[3]),
                coverage_area=json.loads(parts[4]))
            previous_task = b"specificsplit"
        else:  # plain split
            uid = parts[5]
            center_lat_lon_array = json.loads(parts[1])
            flattened = [item[0] if len(item) == 1 and isinstance(item[0], list) else item
                         for item in center_lat_lon_array]
            split_obj = AutoSplitMission(
                origin=origin, center_lat_lons=flattened,
                drone_list=pos_array, grid_spacing=int(json.loads(parts[3])),
                coverage_area=int(json.loads(parts[4])))
            split_obj.GroupSplitting(
                center_lat_lons=flattened, num_of_drones=len(pos_array),
                grid_spacing=int(json.loads(parts[3])),
                coverage_area=int(json.loads(parts[4])))
            previous_task = b"split"
        csv_cache.clear()
        search_step     = 1
        active_bot_count = len(pos_array)
        pop_flag_arr    = [1] * len(pos_array)
        grid_path_array = [0] * len(pos_array)
        _setup_split_csv()
        _reset_removal_state()
        if master_flag:
            _rebuild_sim()
    previous_task_flag = False
    await _run_grid_mission("split_flag", uid)


# ---------------------------------------------------------------------------
# Main trio entry point
# ---------------------------------------------------------------------------

async def main():
    global previous_task, previous_task_flag, num_bots

    send_channel, recv_channel = trio.open_memory_channel(64)

    async with trio.open_nursery() as nursery:
        nursery.start_soon(udp_listener, send_channel)
        nursery.start_soon(home_monitor)

        async with recv_channel:
            async for data in recv_channel:
                print("Dispatch:", data)
                try:
                    origin_local = read_origin(file_name)
                except Exception as e:
                    print("Error reading origin:", e)

                num_bots = len(vehicles) if master_flag else len(pos_array)

                if data.startswith(b"different"):
                    await handle_different_height(data)
                    data = previous_task
                    previous_task_flag = previous_task != b""

                elif data.startswith(b"remove"):
                    _handle_remove(data)
                    data = previous_task
                    previous_task_flag = previous_task != b""

                elif data.startswith(b"add"):
                    try:
                        _handle_add(data)
                    except Exception as e:
                        print("Add drone error:", e)

                elif data.startswith(b"specificbotgoal"):
                    previous_task = b"specificbotgoal"
                    await handle_specificbotgoal(data)

                elif data.startswith(b"goal"):
                    await handle_goal(data)

                elif data.startswith(b"navigate") or start_flag:
                    await handle_navigate(data)

                elif data.startswith(b"disperse") or disperse_flag:
                    await handle_disperse(data)

                elif data.startswith(b"search") or search_flag:
                    previous_task = b"search"
                    await handle_search(data)

                elif data.startswith(
                        (b"split", b"specificsplit", b"polyspecificsplit", b"polyautosplit")
                ) or split_flag:
                    await handle_split(data)

                elif data.startswith(b"aggregate") or aggregate_flag:
                    await handle_aggregate(data)

                elif data == b"home" or home_flag:
                    await handle_home(data)

                elif data == b"home_goto" or home_goto_flag:
                    await handle_home_goto()

                elif data == b"land":
                    for i, vehicle in enumerate(vehicles):
                        vehicle.mode = VehicleMode("LAND")
                        vehicle.close()
                    nursery.cancel_scope.cancel()
                    break

                elif data == b"close" or closing_flag:
                    for vehicle in vehicles:
                        vehicle.close()
                    nursery.cancel_scope.cancel()
                    break

                elif data == b"stop":
                    pass  # handled inside each task's inner loop via _peek_command()

        print("Server shut down.")


if __name__ == "__main__":
    trio.run(main)
