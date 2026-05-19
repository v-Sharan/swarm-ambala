import csv
import os
import simplekml
from geopy.distance import distance
from geopy.point import Point
from .locatePosition import geoToCart, cartToGeo


class SearchGridGenerator:
    def __init__(
        self,
        origin,
        center_latitude,
        center_longitude,
        drone_list=None,
        grid_spacing=15,
        coverage_area=1000,
    ):
        self.center_lat = center_latitude
        self.center_lon = center_longitude

        if drone_list is None:
            drone_list = [1]
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
                    os.remove(file_path)
            except Exception as e:
                print(f"Error deleting file {file_path}: {e}")

        return base_dir

    def generate_grids(self):
        center_point = Point(self.center_lat, self.center_lon)
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
                # line_number += 1
                # current_point = Point(current_lat, west_edge.longitude)
                # east_point = distance(meters=full_width).destination(current_point, 90)

                # if line_number % 2 == 1:
                #     csv_data.extend([(current_point.latitude, current_point.longitude),
                #                      (east_point.latitude, east_point.longitude)])
                #     line.coords.addcoordinates([(current_point.longitude, current_point.latitude),
                #                                 (east_point.longitude, east_point.latitude)])
                #     kml.newpoint(name=str(waypoint_number), coords=[(current_point.longitude, current_point.latitude)])
                #     waypoint_number += 1
                #     kml.newpoint(name=str(waypoint_number), coords=[(east_point.longitude, east_point.latitude)])
                #     waypoint_number += 1
                # else:
                #     csv_data.extend([(east_point.latitude, east_point.longitude),
                #                      (current_point.latitude, current_point.longitude)])
                #     line.coords.addcoordinates([(east_point.longitude, east_point.latitude),
                #                                 (current_point.longitude, current_point.latitude)])
                #     kml.newpoint(name=str(waypoint_number), coords=[(east_point.longitude, east_point.latitude)])
                #     waypoint_number += 1
                #     kml.newpoint(name=str(waypoint_number), coords=[(current_point.longitude, current_point.latitude)])
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
                    distance(meters=self.grid_spacing)
                    .destination(current_point, 0)
                    .latitude
                )

            drone_id = self.drone_list[i]
            kml_filename = os.path.join(self.output_dir, f"search-drone-{drone_id}.kml")
            # csv_filename = os.path.join(self.output_dir, f"d{drone_id}.csv")

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


# # Example usage:
# if __name__ == "__main__":
#     center_lat = 13.389460
#     center_lon = 80.233607
#     num_of_drones = 3
#     grid_spacing = 8  # in meters
#     coverage_area = 200  # in meters (width and height)
#     origin = [ 13.308039,  80.146629]

#     generator = SearchGridGenerator(origin,center_lat, center_lon, num_of_drones, grid_spacing, coverage_area)
#     generator.generate_grids()
