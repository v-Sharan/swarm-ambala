import numpy as np
import simplekml
import csv
import os
import shutil
from shapely.geometry import (
    MultiPolygon,
    box,
    LineString,
    MultiLineString,
    GeometryCollection,
    Point,
)
from shapely.errors import TopologicalError
from functools import cmp_to_key
from math import atan2, degrees, radians, cos, sin
from .locatePosition import geoToCart, cartToGeo  # Your GPS <-> image conversion
import matplotlib.pyplot as plt


class PolygonSpecificSplit:
    class Rtf:
        def __init__(self, angle):
            self.angle = angle
            self.w = radians(90 - self.angle)
            self.irm = np.array(
                [
                    [cos(self.w), -sin(self.w), 0.0],
                    [sin(self.w), cos(self.w), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )

    class AreaPolygon:
        def __init__(self, coordinates, initial_pos, angle, interior=[], ft=5.0):
            self.P = Polygon(coordinates, interior)
            self.ft = ft
            if 0 <= angle <= 360:
                self.rtf = PolygonSpecificSplit.Rtf(angle)
            else:
                self.rtf = self.rtf_longest_edge()
            self.rP = self.rotated_polygon()
            self.origin = self.get_furthest_point(self.P.exterior.coords, initial_pos)[
                0
            ]

        def rtf_longest_edge(self):
            coords = list(self.P.exterior.coords)
            max_len = 0
            best_angle = 0
            for i in range(len(coords) - 1):
                dx = coords[i + 1][0] - coords[i][0]
                dy = coords[i + 1][1] - coords[i][1]
                length = (dx**2 + dy**2) ** 0.5
                if length > max_len:
                    max_len = length
                    best_angle = degrees(atan2(dy, dx))
            return PolygonSpecificSplit.Rtf(best_angle)

        def rotate_points(self, points):
            new_points = []
            for point in points:
                point_mat = np.array([[point[0]], [point[1]], [0]], dtype="float64")
                new_point = self.rtf.irm @ point_mat
                new_points.append(new_point[:-1].flatten())
            return np.array(new_points)

        def rotate_from(self, points):
            irm_inv = np.linalg.inv(self.rtf.irm)
            new_points = []
            for point in points:
                point_mat = np.array([[point[0]], [point[1]], [0]], dtype="float64")
                new_point = irm_inv @ point_mat
                new_points.append(new_point[:-1].flatten())
            return np.array(new_points)

        def rotated_polygon(self):
            tf_points = self.rotate_points(np.array(self.P.exterior.coords))
            tf_holes = [
                self.rotate_points(np.array(list(hole.coords)))
                for hole in self.P.interiors
            ]
            return Polygon(tf_points, tf_holes)

        def generate_path_lines(self):
            minx, miny, maxx, maxy = self.rP.bounds
            width = maxx - minx
            base_line = LineString([(minx, miny), (minx, maxy)])
            lines = []
            iterations = int(width / self.ft) + 2

            polygon = self.rP.buffer(0)

            for i in range(iterations):
                try:
                    offset_line = base_line.parallel_offset(i * self.ft, "right")
                    if offset_line.is_empty:
                        continue
                    intersection = polygon.intersection(offset_line)
                    if intersection.is_empty:
                        continue

                    if isinstance(intersection, (LineString, MultiLineString)):
                        for geom in getattr(intersection, "geoms", [intersection]):
                            if polygon.contains(geom) or polygon.touches(geom):
                                lines.append(geom)
                    elif isinstance(intersection, GeometryCollection):
                        for geom in intersection.geoms:
                            if isinstance(geom, (LineString, MultiLineString)):
                                if polygon.contains(geom) or polygon.touches(geom):
                                    lines.append(geom)

                except TopologicalError:
                    continue
            return lines

        def decompose_lines(self, lines):
            results = []
            reverse = False

            def leftmost_x(line):
                if isinstance(line, LineString):
                    return min(pt[0] for pt in line.coords)
                elif isinstance(line, MultiLineString):
                    return min(min(pt[0] for pt in l.coords) for l in line.geoms)
                return float("inf")

            lines.sort(key=leftmost_x)
            flat_lines = []
            for line in lines:
                if isinstance(line, LineString):
                    flat_lines.append(line)
                elif isinstance(line, MultiLineString):
                    flat_lines.extend(list(line.geoms))

            for line in flat_lines:
                coords = list(line.coords)
                if reverse:
                    coords.reverse()
                results.extend(coords)
                reverse = not reverse

            return results

        def get_furthest_point(self, points, origin):
            origin_pt = Point(*origin)

            def compare(x, y):
                dist_x = origin_pt.distance(Point(*x))
                dist_y = origin_pt.distance(Point(*y))
                return (dist_x > dist_y) - (dist_x < dist_y)

            return sorted(points, key=cmp_to_key(compare))

        def get_full_coverage_path(self):
            origin = self.rotate_points(np.array([self.origin]))[0].tolist()
            lines = self.generate_path_lines()
            ordered_points = self.decompose_lines(lines)
            tf_result = self.rotate_from(np.array(ordered_points))
            return tf_result

    def __init__(
        self,
        polygon_latlon_list,
        origin_gps,
        endDistance,
        drone_list=None,
        grid_spacing=5.0,
        rotation_angle=90,
        obstacles_latlon_list=None,
        drone_assignments=None,
    ):

        self.polygon_latlon_list = polygon_latlon_list
        self.origin_gps = origin_gps
        self.endDistance = endDistance
        if drone_list is None:
            drone_list = [1]
        self.drone_list = drone_list
        self.num_drones = len(self.drone_list)
        # self.grid_spacing = grid_spacing
        self.rotation_angle = rotation_angle
        self.obstacles_latlon_list = (
            obstacles_latlon_list
            if obstacles_latlon_list
            else [[] for _ in polygon_latlon_list]
        )

        self.output_dir = os.path.join(os.getcwd(), "group_split")
        os.makedirs(self.output_dir, exist_ok=True)
        if os.path.exists(self.output_dir):
            for filename in os.listdir(self.output_dir):
                file_path = os.path.join(self.output_dir, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)
                except Exception as e:
                    print(f"⚠️ Failed to delete {file_path}. Reason: {e}")
        if isinstance(grid_spacing, (list, tuple)):
            if len(grid_spacing) != len(polygon_latlon_list):
                raise ValueError(
                    "grid_spacing list length must match number of polygons"
                )
            self.grid_spacing = grid_spacing
        else:
            self.grid_spacing = [grid_spacing] * len(polygon_latlon_list)

        if drone_assignments:
            # Collect all drone IDs assigned
            all_assigned_ids = []
            for group in drone_assignments:
                if isinstance(group, list):
                    all_assigned_ids.extend(group)
                else:
                    all_assigned_ids.append(group)

            # Check if the assigned drone IDs exactly match the expected set of drones 1..num_drones
            expected_ids = set(range(1, self.num_drones + 1))
            assigned_ids = set(all_assigned_ids)

            if assigned_ids != expected_ids:
                raise ValueError(
                    f"Drone IDs assigned ({all_assigned_ids}) do not match expected drone IDs {list(expected_ids)}"
                )

            self.drone_assignments = drone_assignments
        else:
            raise ValueError("drone_assignments must be provided explicitly now.")

        self.planners = []  # one planner per polygon
        self.drone_paths = []  # final paths per drone

    def split_polygon_equally(self, polygon, num_splits):
        if num_splits <= 1:
            return [polygon]
            
        minx, miny, maxx, maxy = polygon.bounds
        total_area = polygon.area
        target_area = total_area / num_splits
        slices = []
        
        current_minx = minx
        
        for i in range(1, num_splits):
            low = current_minx
            high = maxx
            best_maxx = current_minx
            
            # Binary search for the right edge of this slice to get target_area
            tolerance = target_area * 0.005 # 0.5% tolerance
            
            for _ in range(50):
                mid = (low + high) / 2
                slicing_rect = box(current_minx, miny, mid, maxy)
                sliced_part = polygon.intersection(slicing_rect)
                area = sliced_part.area
                
                if abs(area - target_area) < tolerance:
                    best_maxx = mid
                    break
                elif area < target_area:
                    low = mid
                    best_maxx = mid
                else:
                    high = mid
                    
            slice_box = box(current_minx, miny, best_maxx + 0.0001, maxy)
            poly_slice = polygon.intersection(slice_box)
            if poly_slice.is_empty:
                slices.append(Polygon())
            else:
                slices.append(poly_slice)
            current_minx = best_maxx
            
        # The last slice takes the remainder
        slice_box = box(current_minx, miny, maxx + 0.0001, maxy)
        poly_slice = polygon.intersection(slice_box)
        if poly_slice.is_empty:
            slices.append(Polygon())
        else:
            slices.append(poly_slice)
            
        return slices

    def gps_to_image_coords(self, gps_list):
        if len(gps_list) == 0:
            return []
        if isinstance(gps_list[0], float) or isinstance(gps_list[0], int):
            raise ValueError(
                "Expected a list of (lat, lon) pairs, got a flat list of floats instead."
            )
        return [
            geoToCart(self.origin_gps, self.endDistance, (lat, lon))
            for lat, lon in gps_list
        ]

    def create_planner(self, polygon_latlon, obstacles_latlon, num_drones):
        outer_poly_img = self.gps_to_image_coords(polygon_latlon)
        obstacles_img = (
            [self.gps_to_image_coords(obst) for obst in obstacles_latlon]
            if obstacles_latlon
            else []
        )

        full_polygon = Polygon(outer_poly_img, holes=obstacles_img)

        buffer_dist = 0.5
        buffered_polygon = full_polygon.buffer(buffer_dist).buffer(-buffer_dist)

        # Determine rotation angle or use given one
        rotation = self.rotation_angle
        if not (0 <= rotation <= 360):
            coords = list(buffered_polygon.exterior.coords)
            max_len = 0
            best_angle = 0
            for i in range(len(coords) - 1):
                dx = coords[i + 1][0] - coords[i][0]
                dy = coords[i + 1][1] - coords[i][1]
                length = (dx**2 + dy**2) ** 0.5
                if length > max_len:
                    max_len = length
                    best_angle = degrees(atan2(dy, dx))
            rotation = best_angle

        rtf = self.Rtf(rotation)
        rotated_polygon = self.rotate_polygon(buffered_polygon, rtf)

        return outer_poly_img, obstacles_img, buffered_polygon, rotated_polygon, rtf

    def rotate_points(self, points, rtf):
        new_points = []
        for point in points:
            point_mat = np.array([[point[0]], [point[1]], [0]], dtype="float64")
            new_point = rtf.irm @ point_mat
            new_points.append(new_point[:-1].flatten())
        return np.array(new_points)

    def rotate_from(self, points, rtf):
        irm_inv = np.linalg.inv(rtf.irm)
        new_points = []
        for point in points:
            point_mat = np.array([[point[0]], [point[1]], [0]], dtype="float64")
            new_point = irm_inv @ point_mat
            new_points.append(new_point[:-1].flatten())
        return np.array(new_points)

    def rotate_polygon(self, polygon, rtf):
        rotated_exterior = self.rotate_points(np.array(polygon.exterior.coords), rtf)
        rotated_holes = [
            self.rotate_points(np.array(hole.coords), rtf) for hole in polygon.interiors
        ]
        return Polygon(rotated_exterior, rotated_holes)

    def generate_paths(self):
        self.drone_paths = []
        drone_id_to_path_index = {}

        for polygon_idx, (poly_coords, obstacles, drone_ids) in enumerate(
            zip(
                self.polygon_latlon_list,
                self.obstacles_latlon_list,
                self.drone_assignments,
            )
        ):

            # Prepare planner (replace with actual call)
            outer_poly_img, obstacles_img, buffered_polygon, rotated_polygon, rtf = (
                self.create_planner(poly_coords, obstacles, len(drone_ids))
            )

            # Split polygon into pieces equal to the number of drones assigned
            split_polygons = self.split_polygon_equally(rotated_polygon, len(drone_ids))

            for split_idx, poly in enumerate(split_polygons):
                if poly.is_empty:
                    self.drone_paths.append([])
                    continue

                polys_to_process = [poly]
                if isinstance(poly, MultiPolygon):
                    polys_to_process = poly.geoms

                combined_path = []
                for sub_poly in polys_to_process:
                    holes = [list(hole.coords) for hole in sub_poly.interiors]
                    initial_pos = (sub_poly.centroid.x, sub_poly.centroid.y)
                    area_poly = self.AreaPolygon(
                        list(sub_poly.exterior.coords),
                        initial_pos=initial_pos,
                        angle=self.rotation_angle,
                        interior=holes,
                        ft=self.grid_spacing[polygon_idx],
                    )

                    path = area_poly.get_full_coverage_path()
                    if path is not None:
                        combined_path.extend(path)

                self.drone_paths.append(combined_path)

                # Assign path index to drone ID
                drone_id = drone_ids[split_idx]
                drone_id_to_path_index[drone_id] = len(self.drone_paths) - 1

        # Reorder paths by given drone_list directly
        ordered_paths = [[] for _ in range(self.num_drones)]
        for i, assigned_drone_id in enumerate(self.drone_list):
            if assigned_drone_id in drone_id_to_path_index:
                path_idx = drone_id_to_path_index[assigned_drone_id]
                ordered_paths[i] = self.drone_paths[path_idx]

        self.drone_paths = ordered_paths
        return self.drone_paths

    def save_paths(self):
        if not self.drone_paths:
            raise RuntimeError(
                "No drone paths generated yet. Call generate_paths() first."
            )

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        for i, path in enumerate(self.drone_paths):
            drone_id = self.drone_list[i]
            if not path:
                print(f"⚠️ No path for drone {drone_id}, skipping save.")
                continue

            gps_coords = [cartToGeo(self.origin_gps, self.endDistance, p) for p in path]
            kml = simplekml.Kml()

            coords = [(lon, lat) for lat, lon in gps_coords]

            linestring = kml.newlinestring(name=f"Drone path {drone_id}")
            linestring.coords = coords
            linestring.altitudemode = simplekml.AltitudeMode.clamptoground
            linestring.style.linestyle.color = simplekml.Color.red
            linestring.style.linestyle.width = 3

            for idx, (lon, lat) in enumerate(coords, start=1):
                pnt = kml.newpoint(name=str(idx), coords=[(lon, lat)])
                pnt.style.labelstyle.color = simplekml.Color.blue
                pnt.style.iconstyle.color = simplekml.Color.yellow
                pnt.style.iconstyle.scale = 1.0

            kml_path = os.path.join(self.output_dir, f"drone{drone_id}.kml")
            kml.save(kml_path)
            print(f"✅ Saved KML for drone {drone_id} to {kml_path}")

            csv_path = os.path.join(self.output_dir, f"grid_{drone_id}.csv")
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                # writer.writerow(['lat', 'lon'])
                for lat, lon in gps_coords:
                    x, y = geoToCart(self.origin_gps, 500000, [lat, lon])
                    writer.writerow([x / 2, y / 2])
            print(f"✅ Saved CSV for drone {drone_id} to {csv_path}")


# polygon_list = [
#     [
#         (12.932247, 80.046899), (12.930835, 80.048966), (12.931192, 80.053090),
#         (12.929339, 80.054377), (12.928348, 80.053753), (12.930196, 80.052207),
#         (12.929624, 80.050959), (12.929785, 80.048309), (12.931825, 80.046061)
#     ],
#     [
#         (12.909752, 80.034471), (12.902516, 80.028170), (12.899896, 80.032757),
#         (12.893116, 80.033657), (12.904027, 80.042812), (12.909904, 80.041887),
#         (12.909898, 80.034431)
#     ]
# ]

# origin_gps= (12.921654, 80.041917)
# endDistance=500000
# num_drones = 3
# assignments = [[3], [1,2]]  # drone IDs assigned per polygon
# grid_spacing = [50,50]
# planner = PolygonSpecificSplit(
#     polygon_latlon_list=polygon_list,
#     origin_gps=origin_gps,
#     endDistance=endDistance,
#     num_drones=num_drones,
#     grid_spacing=grid_spacing,
#     rotation_angle=90,
#     obstacles_latlon_list=None,
#     drone_assignments=assignments
# )

# paths = planner.generate_paths()
# planner.save_paths()
