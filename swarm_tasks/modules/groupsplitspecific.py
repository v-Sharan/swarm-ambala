import csv, os, sys
import simplekml
from geopy.distance import distance
from geopy.point import Point
from .locatePosition import geoToCart, cartToGeo
import numpy as np
import matplotlib.pyplot as plt


class SpecificSplitMission:
    def __init__(
        self, origin, center_lat_lons, drone_array, grid_spacing, coverage_area
    ):
        self.base_dir = os.getcwd()
        self.origin = origin
        self.center_lat_lons = center_lat_lons
        self.drone_list = drone_array
        self.grid_spacing = grid_spacing
        self.coverage_area = coverage_area
        self.path_kml = os.path.join(self.base_dir, "group_split")
        self.path_csv = os.path.join(self.base_dir, "group_split")
        # self.search_curve = os.path.join(self.base_dir, "group_split", "bezier", "search_{}.kml")
        # self.curve_csv_file = os.path.join(self.base_dir, "group_split", "bezier", "d{}.csv")
        # self.initial_heading = np.radians(0)  # Initial heading angle in radians
        # self.G = 9.81  # Gravity (m/s²)
        # self.MAX_BANK_ANGLE = np.radians(40)  # 40 degrees in radians
        # self.SPEED = 18  # Aircraft speed in m/s
        # self.TURN_RATE = (self.G * np.tan(self.MAX_BANK_ANGLE)) / self.SPEED  # rad/s
        self.sample_points = []
        self.path = []
        self.waypoints = []

        os.makedirs(self.path_kml, exist_ok=True)
        os.makedirs(self.path_csv, exist_ok=True)
        # os.makedirs(os.path.dirname(self.search_curve), exist_ok=True)
        # os.makedirs(os.path.dirname(self.curve_csv_file), exist_ok=True)
        for folder in [self.path_kml, self.path_csv]:
            for filename in os.listdir(folder):
                file_path = os.path.join(folder, filename)
                try:
                    if os.path.isfile(file_path):
                        os.remove(file_path)
                except Exception as e:
                    print(f"Error deleting file {file_path}: {e}")

    def CreateGridsForSpecifiedAreaAndSpecifiedDrones(
        self,
        center_latitude: float,
        center_longitude: float,
        drone_array: int,
        grid_space: int,
        coverage_area: int,
    ) -> None:

        center_lat = center_latitude
        center_lon = center_longitude

        num_rectangles = drone_array
        grid_spacing = grid_space
        meters_for_extended_lines = 250
        full_width, full_height = coverage_area, coverage_area

        rectangle_height = full_height / len(num_rectangles)

        center_point = Point(center_lat, center_lon)

        west_edge = distance(meters=full_width / 2).destination(center_point, 270)

        for i in range(len(drone_array)):
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
            line.style.linestyle.color = simplekml.Color.black
            line.style.linestyle.width = 2
            waypoint_number = 1

            while current_lat <= top.latitude:
                # line_number += 1
                # current_point = Point(current_lat, west_edge.longitude)
                # east_point = distance(meters=full_width).destination(current_point, 90)
                # if line_number % 2 == 1:
                #     csv_data.append((current_point.latitude, current_point.longitude))
                #     csv_data.append((east_point.latitude, east_point.longitude))

                #     line.coords.addcoordinates(
                #         [
                #             (current_point.longitude, current_point.latitude),
                #             (east_point.longitude, east_point.latitude),
                #         ]
                #     )
                #     kml.newpoint(
                #         name=f"{waypoint_number}",
                #         coords=[(current_point.longitude, current_point.latitude)],
                #     )
                #     waypoint_number += 1
                #     kml.newpoint(
                #         name=f"{waypoint_number}",
                #         coords=[(east_point.longitude, east_point.latitude)],
                #     )
                #     waypoint_number += 1
                # else:
                #     csv_data.append((east_point.latitude, east_point.longitude))
                #     csv_data.append((current_point.latitude, current_point.longitude))

                #     line.coords.addcoordinates(
                #         [
                #             (east_point.longitude, east_point.latitude),
                #             (current_point.longitude, current_point.latitude),
                #         ]
                #     )
                #     kml.newpoint(
                #         name=f"{waypoint_number}",
                #         coords=[(east_point.longitude, east_point.latitude)],
                #     )
                #     waypoint_number += 1
                #     kml.newpoint(
                #         name=f"{waypoint_number}",
                #         coords=[(current_point.longitude, current_point.latitude)],
                #     )
                #     waypoint_number += 1
                line_number += 1
                current_point = Point(current_lat, west_edge.longitude)
                mid_point1 = distance(meters=full_width * 1 / 3).destination(
                    current_point, 90
                )
                mid_point2 = distance(
                    meters=full_width - (full_width * 1 / 3)
                ).destination(current_point, 90)
                east_point = distance(meters=full_width).destination(current_point, 90)
                if line_number % 2 == 1:
                    csv_data.append((current_point.latitude, current_point.longitude))
                    csv_data.append((mid_point1.latitude, mid_point1.longitude))
                    csv_data.append((mid_point2.latitude, mid_point2.longitude))
                    csv_data.append((east_point.latitude, east_point.longitude))

                    line.coords.addcoordinates(
                        [
                            (current_point.longitude, current_point.latitude),
                            (east_point.longitude, east_point.latitude),
                        ]
                    )
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(current_point.longitude, current_point.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(mid_point1.longitude, mid_point1.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(mid_point2.longitude, mid_point2.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(east_point.longitude, east_point.latitude)],
                    )
                    waypoint_number += 1
                else:
                    csv_data.append((east_point.latitude, east_point.longitude))
                    csv_data.append((mid_point2.latitude, mid_point2.longitude))
                    csv_data.append((mid_point1.latitude, mid_point1.longitude))
                    csv_data.append((current_point.latitude, current_point.longitude))

                    line.coords.addcoordinates(
                        [
                            (east_point.longitude, east_point.latitude),
                            (current_point.longitude, current_point.latitude),
                        ]
                    )
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(east_point.longitude, east_point.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(mid_point2.longitude, mid_point2.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(mid_point1.longitude, mid_point1.latitude)],
                    )
                    waypoint_number += 1
                    kml.newpoint(
                        name=f"{waypoint_number}",
                        coords=[(current_point.longitude, current_point.latitude)],
                    )
                    waypoint_number += 1

                current_lat = (
                    distance(meters=grid_spacing).destination(current_point, 0).latitude
                )

            drone_id = drone_array[i]
            kml_filename = f"search-drone-{drone_id}.kml"
            kml.save(os.path.join(self.path_kml, kml_filename))
            csv_filename = f"grid_{drone_id}.csv"
            xy = []
            for data in csv_data:
                x, y = geoToCart(self.origin, 500000, data)
                xy.append((x / 2.0, y / 2.0))
            with open(
                os.path.join(self.path_csv, csv_filename),
                mode="w",
                newline="",
            ) as file:
                writer = csv.writer(file)
                writer.writerows(xy)
                # self.generate_bezier_curve(xy,num_rectangles[i])

    def GroupSplitting(
        self, center_lat_lons, drone_array, grid_spacing, coverage_area
    ) -> bool:
        for i in range(len(center_lat_lons)):
            if len(drone_array[i]) == 0:
                continue
            self.CreateGridsForSpecifiedAreaAndSpecifiedDrones(
                center_lat_lons[i][0],
                center_lat_lons[i][1],
                drone_array[i],
                grid_spacing[i],
                coverage_area[i],
            )

        return True


# center_latlon = [
#     [ 12.929813,  80.046912],
#     [ 12.929031 ,  80.050313],
# ]
# origin =(12.921654, 80.041917)
# drone_array = [[3],[1]]
# grid_spacing = [5,5]
# coverage_area = [100,100]
# split = SpecificSplitMission(origin=origin,center_lat_lons=center_latlon, drone_array=drone_array, grid_spacing=grid_spacing,
#                          coverage_area=coverage_area)
# isDone = split.GroupSplitting(
#     center_lat_lons=center_latlon,
#     drone_array=drone_array,
#     grid_spacing=grid_spacing,
#     coverage_area=coverage_area,
# )
