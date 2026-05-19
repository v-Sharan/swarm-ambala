import csv
import os
import simplekml
from geopy.distance import distance
from geopy import Point as GeoPoint
from .locatePosition import geoToCart, cartToGeo
import numpy as np
from shapely.geometry import (
    Point,
    Polygon,
    LineString,
    MultiLineString,
    GeometryCollection,
    box,
    MultiPolygon,
)
from shapely.errors import TopologicalError
from functools import cmp_to_key
from math import atan2, degrees, radians, cos, sin


class SearchGridGenerator:
    def __init__(
        self,
        origin,
        center_latitude,
        center_longitude,
        drone_list,
        grid_spacing,
        coverage_area,
    ):
        self.center_lat = center_latitude
        self.center_lon = center_longitude
        self.drone_list = drone_list
        self.num_of_drones = len(self.drone_list)
        self.grid_spacing = grid_spacing  # in meters
        self.coverage_area = coverage_area  # in meters (width and height)
        self.output_dir = self._create_output_directory()
        self.origin = origin
        self.search_csv_file = os.path.join(self.output_dir, "d{}.csv")

    def _create_output_directory(self):
        base_dir = os.path.join(os.getcwd(), "searchgrid")
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)

        for filename in os.listdir(base_dir):
            file_path = os.path.join(base_dir, filename)
            try:
                if os.path.isfile(file_path):
                    try:
                        os.remove(file_path)
                    except PermissionError:
                        pass
            except Exception as e:
                print(f"Error flushing file {file_path}: {e}")

        return base_dir

    def generate_grids(self):
        center_point = GeoPoint(self.center_lat, self.center_lon)
        full_width = full_height = self.coverage_area
        rectangle_height = full_height / self.num_of_drones

        west_edge = distance(meters=full_width / 2).destination(center_point, 270)
        east_edge = distance(meters=full_width / 2).destination(center_point, 90)
        csv_datas = []
        for i in range(self.num_of_drones):
            top_offset = (
                (i * rectangle_height) - (full_height / 2) + (rectangle_height / 2)
            )
            top_center = distance(meters=top_offset).destination(center_point, 0)
            top = distance(meters=rectangle_height / 2).destination(top_center, 0)
            bottom = distance(meters=rectangle_height / 2).destination(top_center, 180)

            kml = simplekml.Kml()
            csv_data = []

            current_lat = bottom.latitude
            line_number = 0
            line = kml.newlinestring()
            line.altitudemode = simplekml.AltitudeMode.clamptoground
            line.style.linestyle.color = simplekml.Color.red
            line.style.linestyle.width = 2
            waypoint_number = 1

            while current_lat <= top.latitude:
                line_number += 1
                current_point = GeoPoint(current_lat, west_edge.longitude)
                east_point = distance(meters=full_width).destination(current_point, 90)

                if line_number % 2 == 1:
                    csv_data.extend(
                        [
                            (current_point.latitude, current_point.longitude),
                            (east_point.latitude, east_point.longitude),
                        ]
                    )
                    line.coords.addcoordinates(
                        [
                            (current_point.longitude, current_point.latitude),
                            (east_point.longitude, east_point.latitude),
                        ]
                    )
                    kml.newpoint(
                        name=str(waypoint_number),
                        coords=[(current_point.longitude, current_point.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=str(waypoint_number),
                        coords=[(east_point.longitude, east_point.latitude)],
                    )
                    waypoint_number += 1
                else:
                    csv_data.extend(
                        [
                            (east_point.latitude, east_point.longitude),
                            (current_point.latitude, current_point.longitude),
                        ]
                    )
                    line.coords.addcoordinates(
                        [
                            (east_point.longitude, east_point.latitude),
                            (current_point.longitude, current_point.latitude),
                        ]
                    )
                    kml.newpoint(
                        name=str(waypoint_number),
                        coords=[(east_point.longitude, east_point.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=str(waypoint_number),
                        coords=[(current_point.longitude, current_point.latitude)],
                    )
                    waypoint_number += 1

                current_lat = (
                    distance(meters=self.grid_spacing)
                    .destination(current_point, 0)
                    .latitude
                )

            drone_id = self.drone_list[i]
            kml_filename = os.path.join(self.output_dir, f"search-drone-{drone_id}.kml")
            # csv_filename = os.path.join(self.output_dir, f"d{i+1}.csv")

            kml.save(kml_filename)
            csv_datas.append(csv_data)
        minimum_waypoints = len(min(csv_datas, key=len))
        for i in range(len(csv_datas)):
            drone_id = self.drone_list[i]
            number_of_waypoints = 0
            with open(
                self.search_csv_file.format(drone_id),
                mode="w",
                newline="",
            ) as file:
                for j in range(len(csv_datas[i])):
                    if number_of_waypoints < minimum_waypoints:
                        writer = csv.writer(file)
                        x, y = geoToCart(self.origin, 500000, csv_datas[i][j])
                        writer.writerow((x / 2, y / 2))
                    number_of_waypoints += 1
        # print('csv_data',csv_datas)
        # with open(csv_filename, mode='w', newline='') as file:
        #         writer = csv.writer(file)
        #         x,y = geoToCart(self.origin,500000,csv_data[i])
        #         writer.writerow((x/2,y/2))

        # print(f"Grid generation complete. Files saved in: {self.output_dir}")
        return 1


class PolygonSearchGrid:
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
                self.rtf = PolygonSearchGrid.Rtf(angle)
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
            return PolygonSearchGrid.Rtf(best_angle)

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

            # Ensure polygon validity
            polygon = self.rP.buffer(0)

            for i in range(iterations):
                try:
                    offset_line = base_line.parallel_offset(i * self.ft, "right")
                    if offset_line.is_empty:
                        continue
                    # Intersect with polygon (with holes)
                    intersection = polygon.intersection(offset_line)
                    if intersection.is_empty:
                        continue

                    # Flatten intersection geometries and filter LineStrings
                    if isinstance(intersection, (LineString, MultiLineString)):
                        for geom in getattr(intersection, "geoms", [intersection]):
                            # Check if line is inside polygon but outside holes
                            # Holes are part of polygon interiors, so intersection handles this
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
        polygon_latlon,
        origin_gps,
        endDistance,
        drone_list=None,
        grid_spacing=5.0,
        rotation_angle=90,
        obstacles_latlon=None,
    ):
        if drone_list is None:
            drone_list = [1]
        self.drone_list = drone_list
        self.num_drones = len(self.drone_list)
        self.grid_spacing = grid_spacing
        self.rotation_angle = rotation_angle
        self.origin_gps = origin_gps
        self.endDistance = endDistance
        self.output_dir = self._create_output_directory()
        # os.makedirs(self.output_dir, exist_ok=True)

        if obstacles_latlon is None:
            obstacles_latlon = []

        self.polygon_latlon = polygon_latlon
        self.obstacles_latlon = obstacles_latlon

        # Convert polygon and obstacles from GPS to image coords
        self.outer_poly_img = self.gps_to_image_coords(polygon_latlon)
        self.obstacles_img = [
            self.gps_to_image_coords(obst) for obst in obstacles_latlon
        ]

        # Create polygon with holes
        self.full_polygon = Polygon(self.outer_poly_img, holes=self.obstacles_img)

        # Buffer polygon to fix minor gaps
        self.buffer_dist = 0.5
        self.buffered_polygon = self.full_polygon.buffer(self.buffer_dist)
        self.buffered_polygon = self.buffered_polygon.buffer(-self.buffer_dist)

        # Rotation object
        if 0 <= rotation_angle <= 360:
            self.rtf = self.Rtf(rotation_angle)
        else:
            coords = list(self.buffered_polygon.exterior.coords)
            max_len = 0
            best_angle = 0
            for i in range(len(coords) - 1):
                dx = coords[i + 1][0] - coords[i][0]
                dy = coords[i + 1][1] - coords[i][1]
                length = (dx**2 + dy**2) ** 0.5
                if length > max_len:
                    max_len = length
                    best_angle = degrees(atan2(dy, dx))
            self.rtf = self.Rtf(best_angle)

        self.rotated_polygon = self.rotate_polygon(self.buffered_polygon, self.rtf)

    def _create_output_directory(self):
        base_dir = os.path.join(os.getcwd(), "searchgrid")
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)

        for filename in os.listdir(base_dir):
            file_path = os.path.join(base_dir, filename)
            try:
                if os.path.isfile(file_path):
                    try:
                        os.remove(file_path)
                    except PermissionError:
                        pass
            except Exception as e:
                print(f"Error flushing file {file_path}: {e}")

        return base_dir

    def gps_to_image_coords(self, gps_list):
        print("gps_list", gps_list, gps_list[0])
        # If gps_list contains floats instead of pairs, raise a clear error
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
        if num_parts <= 1:
            return [polygon]
            
        minx, miny, maxx, maxy = polygon.bounds
        total_area = polygon.area
        target_area = total_area / num_parts
        slices = []
        
        current_minx = minx
        
        for i in range(1, num_parts):
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
                    
            slicing_rect = box(current_minx, miny, best_maxx, maxy)
            slices.append(polygon.intersection(slicing_rect))
            current_minx = best_maxx
            
        # The last slice takes the remainder
        slicing_rect = box(current_minx, miny, maxx, maxy)
        slices.append(polygon.intersection(slicing_rect))
        
        return slices

    def generate_paths(self):
        print("enter generate_paths")
        split_polygons = self.split_polygon_equally(
            self.rotated_polygon, self.num_drones
        )
        drone_paths = []

        for i, poly in enumerate(split_polygons):
            if poly.is_empty:
                drone_paths.append([])
                continue
            polys_to_process = [poly]
            if isinstance(poly, MultiPolygon):
                polys_to_process = poly.geoms
            combined_path = []
            for sub_poly in polys_to_process:
                holes = [list(hole.coords) for hole in sub_poly.interiors]
                initial_pos = (sub_poly.centroid.x, sub_poly.centroid.y)

                sub_area = self.AreaPolygon(
                    list(sub_poly.exterior.coords),
                    initial_pos=initial_pos,
                    angle=self.rotation_angle,
                    interior=holes,
                    ft=self.grid_spacing,
                )
                path = sub_area.get_full_coverage_path()
                if path is not None:
                    combined_path.extend(path)
            drone_paths.append(combined_path)
        self.drone_paths = drone_paths
        return drone_paths

    def save_paths(self):
        if not hasattr(self, "drone_paths"):
            raise RuntimeError("Paths not generated yet. Call generate_paths() first.")

        for i, path in enumerate(self.drone_paths):
            drone_id = self.drone_list[i]
            if path is None or len(path) == 0:
                print(f"Drone {drone_id} path is empty. Skipping.")
                continue

            kml = simplekml.Kml()
            ls = kml.newlinestring(name=f"Drone {drone_id} Path")
            ls.altitudemode = simplekml.AltitudeMode.clamptoground
            ls.style.linestyle.color = simplekml.Color.red
            ls.style.linestyle.width = 2

            csv_data = []
            waypoint_number = 1
            for pt in path:
                lonlat = cartToGeo(self.origin_gps, self.endDistance, [pt[0], pt[1]])
                ls.coords.addcoordinates([(lonlat[1], lonlat[0])])
                x, y = geoToCart(self.origin_gps, 500000, [lonlat[0], lonlat[1]])
                csv_data.append([x / 2, y / 2])
                kml.newpoint(name=str(waypoint_number), coords=[(lonlat[1], lonlat[0])])
                waypoint_number += 1

            kml_path = os.path.join(self.output_dir, f"drone{drone_id}.kml")
            csv_path = os.path.join(self.output_dir, f"d{drone_id}.csv")

            kml.save(kml_path)

            with open(csv_path, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                # writer.writerow(['latitude', 'longitude'])
                writer.writerows(csv_data)


# Example usage:
# if __name__ == "__main__":
#     center_lat = 13.389460
#     center_lon = 80.233607
#     num_of_drones = 3
#     grid_spacing = 8  # in meters
#     coverage_area = 200  # in meters (width and height)
#     origin = [ 13.308039,  80.146629]

#     generator = SearchGridGenerator(origin,center_lat, center_lon, num_of_drones, grid_spacing, coverage_area)
#     generator.generate_grids()


# # user params
# num_of_drones = 3
# grid_spacing = 20
# rotation_angle = 90
# origin = (12.921654, 80.041917)
# endDistance = 500000

# polygon_latlon = [
#     (12.928780, 80.045609), (12.931230, 80.046191), (12.929966, 80.049102),
#     (12.929860, 80.051336), (12.930454, 80.052037), (12.928564, 80.054035),
#     (12.928856, 80.055802), (12.928656, 80.056840), (12.926818, 80.056999),
#     (12.925478, 80.056915), (12.927396, 80.049188), (12.927710, 80.049136),
#     (12.928780, 80.045609)
# ]
# polygon_latlon = [
#     [12.73925362418133, 80.11515441301884], [12.689066823197209, 80.09303907195704], [12.677179018464003, 80.15442043324109], [12.737539859706118, 80.18854396070141]
# ]
# #obstacles_latlon= [ [(12.929028, 80.046939),( 12.929715, 80.046791),( 12.929245, 80.049105),( 12.928527, 80.049221),(12.929028, 80.046939)]]
# obstacles_latlon=[]
# planner = PolygonSearchGrid(
#     polygon_latlon=polygon_latlon,
#     origin_gps=origin,
#     endDistance=endDistance,
#     num_drones=num_of_drones,
#     grid_spacing=grid_spacing,
#     rotation_angle=rotation_angle,
#     obstacles_latlon=obstacles_latlon
# )

# planner.generate_paths()
# planner.save_paths()
