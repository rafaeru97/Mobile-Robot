import numpy as np
import matplotlib.pyplot as plt
import math
import json

from scipy.ndimage import binary_dilation, binary_erosion
from scipy.spatial import distance_matrix, ConvexHull
import alphashape
from scipy.stats import zscore


class EKF_SLAM:
    def __init__(self):
        # Stan robota i mapy (początkowo tylko robot)
        self.state = np.zeros(3)  # [x, y, theta]
        self.covariance = np.eye(3) * 0.1  # Początkowa niepewność

        # Przechowuj mapę: lista punktów mapy i ich niepewności
        self.map_points = []  # Lista punktów mapy

        # Ustawienia
        self.process_noise = np.eye(3) * 0.01  # Szum procesu
        self.measurement_noise = np.eye(2) * 0.1  # Szum pomiaru

    def update(self, control_input, measurements):
        # Ekstrahuj dane wejściowe kontroli
        delta_x, delta_y, delta_theta = control_input

        # Aktualizacja lokalizacji robota
        self._predict(delta_x, delta_y, delta_theta)
        self._update(measurements)

    def _predict(self, delta_x, delta_y, delta_theta):
        # Model ruchu robota
        self.state[0] += delta_x * np.cos(self.state[2])
        self.state[1] += delta_y * np.sin(self.state[2])
        self.state[2] += delta_theta

        # Aktualizacja niepewności
        self.covariance += self.process_noise

    def _update(self, measurements):
        # Aktualizacja mapy na podstawie pomiarów
        for measurement in measurements:
            # Zakładamy, że measurements to [(x, y, distance), ...]
            x_map, y_map, dist = measurement
            # Dodaj nowy punkt do mapy i zaktualizuj niepewność
            self.map_points.append((x_map, y_map, dist))
            # Tutaj powinna być dodatkowa aktualizacja macierzy kowariancji

    def get_state(self):
        return self.state

    def get_map(self):
        return self.map_points


class Mapper:
    def __init__(self, motor_controller, gyro, distance_sensor):
        self.motor_controller = motor_controller
        self.gyro = gyro
        self.distance_sensor = distance_sensor
        self.slam = EKF_SLAM()
        self.positions = [(0, 0)]
        self.current_angle = 0
        self.x = 0
        self.y = 0
        self.last_encoder_distance = 0
        self.detected_points = []  # Inicjalizacja atrybutu

    def get_pos(self):
        return self.x, self.y

    def save_detected_points(self, filename="mapa.json", format="json"):
        """
        Save detected points to a file in a specified format (e.g., JSON).
        """
        if format == "json":
            # Konwersja numpy array do listy
            detected_points_list = [list(point) for point in self.detected_points]

            with open(filename, "w") as f:
                json.dump(detected_points_list, f)

    def process_saved_points(self, filename, format="json", output_filename="output_map.png", zoom_level=100):
        """
        Load points from a file, process them and generate a map.
        :param filename: The name of the file where the points are stored.
        :param format: Format of the file (default is JSON).
        :param output_filename: The name of the output image file.
        :param zoom_level: Zoom level for the map visualization.
        """
        if format == "json":
            with open(filename, "r") as f:
                loaded_points = json.load(f)

        # Convert loaded points to a numpy array
        loaded_points = np.array(loaded_points)

        # Check if loaded points are valid
        if len(loaded_points) == 0:
            print("Brak wczytanych punktów do przetworzenia.")
            return

        # Save loaded points to detected_points for processing
        self.detected_points = list(loaded_points)  # Convert array back to list if necessary

        # Process the points and generate the map
        self.process_detected_points(zoom_level=zoom_level, filename=output_filename)

    def create_map(self, filename="robot_map.png", zoom_level=100):
        positions = np.array(self.positions)
        x_positions = positions[:, 0]
        y_positions = positions[:, 1]

        detected_points = np.array(self.slam.get_map())
        if len(detected_points) > 0:
            x_detected = detected_points[:, 0]
            y_detected = detected_points[:, 1]

        orientation_length = 10
        angle_rad = math.radians(self.current_angle)
        dx_arrow = orientation_length * math.cos(angle_rad)
        dy_arrow = orientation_length * math.sin(angle_rad)

        plt.figure(figsize=(8, 8))
        plt.plot(x_positions, y_positions, marker="o", color="b", markersize=3)
        plt.plot(x_positions[-1], y_positions[-1], marker="o", color="r", markersize=10, label="Current Position")
        plt.arrow(x_positions[-1], y_positions[-1], dx_arrow, dy_arrow, head_width=2, head_length=2, fc='k', ec='k',
                  label="Orientation")

        if len(detected_points) > 0:
            plt.scatter(x_detected, y_detected, color='g', label="Detected Points", s=30)

        center_x, center_y = x_positions[-1], y_positions[-1]
        plt.xlim(center_x - zoom_level, center_x + zoom_level)
        plt.ylim(center_y - zoom_level, center_y + zoom_level)

        plt.title("Robot Movement Path with Detected Points")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True)
        plt.legend()
        plt.savefig(filename)
        plt.close()

    def update_position(self):
        # Aktualizacja pozycji robota
        self.current_angle = self.gyro.get_angle_z()
        current_distance = self.motor_controller.getEncoderDistance()
        distance = current_distance - self.last_encoder_distance
        self.last_encoder_distance = current_distance
        distance_cm = distance * 100

        angle_rad = math.radians(self.current_angle)
        dx = distance_cm * math.cos(angle_rad)
        dy = distance_cm * math.sin(angle_rad)

        self.x += dx
        self.y += dy
        self.positions.append((round(self.x, 2), round(self.y, 2)))

        distance_from_sensor = self.distance_sensor.get_distance()
        if self.distance_sensor.get_status():
            if distance_from_sensor is not None and 0 < distance_from_sensor < 100:
                distance_from_sensor_cm = distance_from_sensor
                detected_x = self.x + distance_from_sensor_cm * math.cos(angle_rad)
                detected_y = self.y + distance_from_sensor_cm * math.sin(angle_rad)
                self.detected_points.append((detected_x, detected_y, distance_from_sensor_cm))
                self.slam.update((dx, dy, angle_rad), [(detected_x, detected_y, distance_from_sensor_cm)])

    def process_detected_points(self, zoom_level=100, filename="output_map.png"):
        """
        Process the detected points by filtering noise, estimating boundaries, and detecting objects.
        :param zoom_level: Zoom level for the map visualization.
        :param filename: The name of the output image file.
        :return: None (saves a visual map with boundaries and detected objects).
        """
        if len(self.detected_points) == 0:
            return

        # Convert detected points to a numpy array
        detected_array = np.array(self.detected_points)
        points = detected_array[:, :2]

        # Compute distance matrix
        dist_matrix = distance_matrix(points, points)

        # Threshold to consider a point as a neighbor
        threshold = 3  # Adjust this value based on your data

        # Filter points that are isolated
        filtered_points = []
        for i, point in enumerate(points):
            neighbors = np.sum(dist_matrix[i] < threshold)
            if neighbors > 1:  # If more than one neighbor, keep the point
                filtered_points.append(point)

        filtered_points = np.array(filtered_points)

        if len(filtered_points) < 3:
            return

        # Apply Z-score for statistical filtering
        z_scores = np.abs(zscore(filtered_points, axis=0))
        filtered_points = filtered_points[(z_scores < 3).all(axis=1)]

        if len(filtered_points) < 3:
            return

        # Generate Alpha Shape
        alpha = 0.05  # Adjust this value to control the level of detail
        alpha_shape = alphashape.alphashape(filtered_points, alpha)

        # Convert Alpha Shape to coordinates for plotting
        if alpha_shape.geom_type == 'Polygon':
            boundary = np.array(alpha_shape.exterior.coords)
        elif alpha_shape.geom_type == 'MultiPolygon':
            boundary = np.concatenate([np.array(p.exterior.coords) for p in alpha_shape.geoms], axis=0)
        else:
            boundary = np.array([])

        # Plotting the results
        plt.figure(figsize=(8, 8))
        plt.plot(filtered_points[:, 0], filtered_points[:, 1], 'o', label='Filtered Points')
        if boundary.size > 0:
            plt.plot(boundary[:, 0], boundary[:, 1], 'r--', lw=2, label='Alpha Shape')

        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Map with Alpha Shape and Interpolated Points')
        plt.legend()
        plt.grid()
        plt.savefig(filename)
        plt.show()

    def generate_map_grid(self, resolution=1.0, dilation_radius=2, erosion_radius=2, min_width=200, min_height=200):
        """
        Generate a grid map from detected points with enhanced morphological operations to fill gaps.
        :param resolution: The resolution of the grid in the same units as the detected points.
        :param dilation_radius: The radius of the dilation operation.
        :param erosion_radius: The radius of the erosion operation.
        :param min_width: Minimum width of the grid (in cells).
        :param min_height: Minimum height of the grid (in cells).
        :return: A numpy array representing the grid map.
        """
        if len(self.detected_points) == 0:
            # Return a grid of minimum size if no points are detected
            return np.zeros((min_height, min_width), dtype=bool)

        # Convert detected points to a numpy array
        detected_array = np.array(self.detected_points)
        points = detected_array[:, :2]

        # Determine the bounds of the grid
        min_x, max_x = points[:, 0].min(), points[:, 0].max()
        min_y, max_y = points[:, 1].min(), points[:, 1].max()

        # Determine grid dimensions
        width = max(min_width, int(np.ceil((max_x - min_x) / resolution)))
        height = max(min_height, int(np.ceil((max_y - min_y) / resolution)))

        # Create an empty grid
        map_grid = np.zeros((height, width), dtype=bool)

        # Fill the grid with obstacles
        for point in points:
            grid_x = int((point[0] - min_x) / resolution)
            grid_y = int((point[1] - min_y) / resolution)
            if 0 <= grid_x < width and 0 <= grid_y < height:
                map_grid[grid_y, grid_x] = True

        # Create structuring elements for dilation and erosion
        dilate_elem = np.ones((2 * dilation_radius + 1, 2 * dilation_radius + 1), dtype=bool)
        erode_elem = np.ones((2 * erosion_radius + 1, 2 * erosion_radius + 1), dtype=bool)

        # Apply multiple dilation to fill larger gaps
        for _ in range(3):  # Apply dilation 3 times
            map_grid = binary_dilation(map_grid, structure=dilate_elem).astype(map_grid.dtype)

        # Apply erosion to remove small noise
        map_grid = binary_erosion(map_grid, structure=erode_elem).astype(map_grid.dtype)

        return map_grid

    def save_map_grid_to_file(self, map_grid, filename="map_grid.txt"):
        """
        Save the map grid to a text file.
        :param map_grid: The grid map array to save.
        :param filename: The name of the output text file.
        :return: None
        """
        np.savetxt(filename, map_grid, fmt='%d', delimiter=' ')

    def get_robot_grid_position(self, map_grid, resolution=1.0):
        """
        Calculate the robot's position on the grid map based on its current position and grid dimensions.
        :param map_grid: The grid map array.
        :param robot_pos: A tuple (x, y) representing the robot's position in map units.
        :param resolution: The resolution of the grid in the same units as the robot position.
        :return: A tuple (grid_x, grid_y) representing the robot's position in grid coordinates.
        """
        # Sprawdzenie wymiarów mapy siatki
        height, width = map_grid.shape

        # Zakładamy, że środek mapy siatki to (width/2, height/2)
        center_x = width / 2
        center_y = height / 2

        # Uzyskanie pozycji robota
        robot_x, robot_y = self.get_pos()

        # Obliczenie przesunięcia robota w kontekście siatki
        grid_x = int(center_x + (robot_x / resolution))
        grid_y = int(center_y + (robot_y / resolution))

        # Upewnij się, że współrzędne są w granicach mapy
        grid_x = max(0, min(grid_x, width - 1))
        grid_y = max(0, min(grid_y, height - 1))

        return grid_x, grid_y
