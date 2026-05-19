import numpy as np
import simplekml
import csv
import os
from shapely.geometry import (
    Polygon,
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


class PolygonAutoSplit:
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
                self.rtf = PolygonAutoSplit.Rtf(angle)
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
            return PolygonAutoSplit.Rtf(best_angle)

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
    ):

        self.polygon_latlon_list = polygon_latlon_list
        print("polygon_latlon_list", polygon_latlon_list)
        self.origin_gps = origin_gps
        self.endDistance = endDistance
        if drone_list is None:
            drone_list = [1]
        self.drone_list = drone_list
        self.num_drones = len(self.drone_list)
        self.grid_spacing = grid_spacing
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
        # Assign drones to polygons fairly
        self.drone_assignments = self.split_drones_among_polygons(
            self.num_drones, len(self.polygon_latlon_list)
        )

        self.planners = []  # one planner per polygon

        self.drone_paths = []  # final paths per drone

    @staticmethod
    def split_drones_among_polygons(total_num_drones, num_polygons):
        base = total_num_drones // num_polygons
        remainder = total_num_drones % num_polygons
        assignments = [base] * num_polygons
        for i in range(remainder):
            assignments[i] += 1
        return assignments

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

    def split_polygon_equally(self, polygon, num_parts):
        minx, miny, maxx, maxy = polygon.bounds
        width = maxx - minx
        slice_width = width / num_parts
        slices = []
        for i in range(num_parts):
            slice_minx = minx + i * slice_width
            slice_maxx = slice_minx + slice_width
            slicing_rect = box(slice_minx, miny, slice_maxx, maxy)
            sliced_part = polygon.intersection(slicing_rect)
            slices.append(sliced_part)
        return slices

    def generate_paths(self):
        self.drone_paths = []
        drone_id = 1
        for i, (poly_coords, obstacles, num_drones) in enumerate(
            zip(
                self.polygon_latlon_list,
                self.obstacles_latlon_list,
                self.drone_assignments,
            )
        ):
            outer_poly_img, obstacles_img, buffered_polygon, rotated_polygon, rtf = (
                self.create_planner(poly_coords, obstacles, num_drones)
            )

            split_polygons = self.split_polygon_equally(rotated_polygon, num_drones)
            for part_idx, poly in enumerate(split_polygons):
                if poly.is_empty:
                    self.drone_paths.append([])
                    drone_id += 1
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
                        ft=self.grid_spacing,
                    )
                    path = area_poly.get_full_coverage_path()
                    if path is not None:
                        combined_path.extend(path)

                self.drone_paths.append(combined_path)
                drone_id += 1
        return self.drone_paths

    def save_paths(self):
        if not self.drone_paths:
            raise RuntimeError(
                "No drone paths generated yet. Call generate_paths() first."
            )

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        polygon_idx = 0
        drone_count_in_poly = 0
        drones_in_poly = self.drone_assignments[polygon_idx]

        for i, path in enumerate(self.drone_paths):
            drone_id = self.drone_list[i]
            drone_count_in_poly += 1
            if drone_count_in_poly > drones_in_poly:
                polygon_idx += 1
                drones_in_poly = self.drone_assignments[polygon_idx]
                drone_count_in_poly = 1

            if not path:
                print(f"⚠️ No path for drone {drone_id}, skipping file save.")
                continue

            # Convert Cartesian points to GPS (lat, lon)
            gps_coords = [cartToGeo(self.origin_gps, self.endDistance, p) for p in path]

            kml = simplekml.Kml()

            # Prepare coords for KML in (lon, lat) order
            coords = [(lon, lat) for lat, lon in gps_coords]

            linestring = kml.newlinestring(name=f"Drone path {drone_id}")
            linestring.coords = coords
            linestring.altitudemode = simplekml.AltitudeMode.clamptoground
            linestring.style.linestyle.color = simplekml.Color.red
            linestring.style.linestyle.width = 3

            # Add numbered points (waypoints)
            for idx, (lon, lat) in enumerate(coords, start=1):
                pnt = kml.newpoint(name=str(idx), coords=[(lon, lat)])
                pnt.style.labelstyle.color = simplekml.Color.blue
                pnt.style.iconstyle.color = simplekml.Color.yellow
                pnt.style.iconstyle.scale = 1.0

            kml_path = os.path.join(
                self.output_dir, f"poly{polygon_idx}_drone{drone_id}.kml"
            )
            kml.save(kml_path)

            # Save CSV with (lat, lon)
            csv_path = os.path.join(self.output_dir, f"grid_{drone_id}.csv")
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                # writer.writerow(['lat', 'lon'])
                for lat, lon in gps_coords:
                    x, y = geoToCart(self.origin_gps, 500000, [lat, lon])
                    writer.writerow([x / 2, y / 2])

    def visualize(self):
        fig, ax = plt.subplots(figsize=(10, 10))

        # Plot polygons and obstacles
        for poly_coords, obstacles_latlon in zip(
            self.polygon_latlon_list, self.obstacles_latlon_list
        ):
            poly_img = self.gps_to_image_coords(poly_coords)
            x, y = zip(*poly_img)
            ax.plot(x, y, "k-", label="Polygon")

            for obst in obstacles_latlon:
                obst_img = self.gps_to_image_coords(obst)
                ox, oy = zip(*obst_img)
                ax.fill(ox, oy, "r", alpha=0.5, label="Obstacle")

        # Plot coverage paths
        for path in self.drone_paths:
            if path:
                px, py = zip(*path)
                ax.plot(px, py, "b-", label="Coverage Path")

        ax.set_aspect("equal")
        plt.legend()
        plt.title("Drone Coverage Planning")
        plt.show()


# planner = PolygonAutoSplit(
#     polygon_latlon_list = [
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
#     ],

#     origin_gps= (12.921654, 80.041917),
#     endDistance=500000,       # as in your original code
#     num_drones=3,
#     grid_spacing=20,
#     rotation_angle=90,               # or -1 to auto-detect angle
#     obstacles_latlon_list=[],
# )

# planner.generate_paths()
# planner.save_paths()
# planner.visualize()
