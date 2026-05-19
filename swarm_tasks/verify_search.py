import sys
from unittest.mock import MagicMock

# Mock problematic dependencies
mock_geopy = MagicMock()
sys.modules["geopy"] = mock_geopy
sys.modules["geopy.distance"] = mock_geopy
sys.modules["geopy.point"] = mock_geopy
sys.modules["simplekml"] = MagicMock()
sys.modules["shapely"] = MagicMock()
sys.modules["shapely.geometry"] = MagicMock()
sys.modules["shapely.errors"] = MagicMock()
sys.modules["scipy"] = MagicMock()
sys.modules["scipy.interpolate"] = MagicMock()
sys.modules["matplotlib"] = MagicMock()
sys.modules["matplotlib.pyplot"] = MagicMock()

# Mock internal dependencies that might fail
sys.modules["swarm_tasks.utils"] = MagicMock()
sys.modules["swarm_tasks.controllers"] = MagicMock()

import os

# Add the parent of 'swarm_tasks' to sys.path
parent_dir = os.path.dirname(r"d:\nithya\copter\swarm_tasks")
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

try:
    from swarm_tasks.modules.search import PolygonSearchGrid
    from swarm_tasks.modules.multipoly_grid import PolygonAutoSplit
    from swarm_tasks.modules.multipoly_specificgrid import PolygonSpecificSplit
    from swarm_tasks.modules.groupsplitauto import AutoSplitMission
    from swarm_tasks.modules.groupsplitspecific import SpecificSplitMission

    print("All modules imported (with mocks) successfully.")
except ImportError as e:
    print(f"Import Error even with mocks: {e}")
    sys.exit(1)

# Mock data
origin = (12.921654, 80.041917)
endDistance = 500000
pos_array = [1, 2, 3]
polygon_array = [
    [12.928780, 80.045609],
    [12.931230, 80.046191],
    [12.929966, 80.049102],
    [12.928780, 80.045609],
]
grid_spacing = 20
rotation_angle = 90
uav_array = [[1], [2], [3]]


def test_initialization(name, cls, **kwargs):
    print(f"Testing {name} initialization...")
    try:
        # We just want to see if it RAISES a TypeError due to argument mismatch
        # The actual __init__ might still fail due to mocks, but we look for TypeError
        cls(**kwargs)
        print(f"SUCCESS: {name} signature matched.")
    except TypeError as e:
        if (
            "unexpected keyword argument" in str(e)
            or "required positional argument" in str(e)
            or "multiple values for argument" in str(e)
        ):
            print(f"FAILURE: {name} signature mismatch: {e}")
        else:
            print(
                f"SUCCESS: {name} signature matched (raised non-signature TypeError: {e})"
            )
    except Exception as e:
        print(f"SUCCESS: {name} signature matched (raised exception: {e})")


# 1. PolygonSearchGrid
test_initialization(
    "PolygonSearchGrid",
    PolygonSearchGrid,
    polygon_latlon=polygon_array,
    origin_gps=origin,
    endDistance=endDistance,
    drone_list=pos_array,
    grid_spacing=grid_spacing,
    rotation_angle=rotation_angle,
    obstacles_latlon=[],
)

# 2. PolygonAutoSplit
test_initialization(
    "PolygonAutoSplit",
    PolygonAutoSplit,
    polygon_latlon_list=polygon_array,
    origin_gps=origin,
    endDistance=endDistance,
    drone_list=pos_array,
    grid_spacing=grid_spacing,
    rotation_angle=90,
    obstacles_latlon_list=[],
)

# 3. PolygonSpecificSplit
test_initialization(
    "PolygonSpecificSplit",
    PolygonSpecificSplit,
    polygon_latlon_list=[polygon_array],
    origin_gps=origin,
    endDistance=endDistance,
    drone_list=pos_array,
    grid_spacing=[grid_spacing],
    rotation_angle=90,
    obstacles_latlon_list=None,
    drone_assignments=uav_array,
)

# 4. AutoSplitMission
test_initialization(
    "AutoSplitMission",
    AutoSplitMission,
    origin=origin,
    center_lat_lons=[origin],
    drone_list=pos_array,
    grid_spacing=grid_spacing,
    coverage_area=100,
)

# 5. SpecificSplitMission
test_initialization(
    "SpecificSplitMission",
    SpecificSplitMission,
    origin=origin,
    center_lat_lons=[origin],
    drone_array=pos_array,
    grid_spacing=grid_spacing,
    coverage_area=100,
)
