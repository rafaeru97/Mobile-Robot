import numpy as np
import matplotlib.pyplot as plt
import math
import json

from scipy.ndimage import binary_dilation, generic_filter
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
    def __init__(self, motor_controller, gyro, distance_sensor, grid_size=(200, 200)):
        self.motor_controller = motor_controller
        self.gyro = gyro
        self.distance_sensor = distance_sensor
        self.slam = EKF_SLAM()
        self.positions = [(100, 100)]
        self.current_angle = 0
        self.x = 100
        self.y = 100
        self.last_encoder_distance = 0
        self.detected_points = []  # Inicjalizacja atrybutu

        self.map_grid = None
        self.grid_size = grid_size

    def clear_robot_path(self):
        self.positions = [(self.x, self.y)]

    def get_pos(self):
        return self.x, self.y

    def get_map_grid(self):
        return self.map_grid

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
        self.create_map_grid()
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
        plt.plot(x_positions[-1], y_positions[-1], marker="s", color="r", markersize=25, label="Current Position")
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

    def create_map_grid(self):
        # Inicjalizuj mapę gridową jako wolną
        self.map_grid = np.zeros(self.grid_size)

        # Zbierz dane z wykrytych punktów
        detected_points = np.array(self.slam.get_map())
        if len(detected_points) > 0:
            x_detected = detected_points[:, 0]
            y_detected = detected_points[:, 1]
        else:
            x_detected = np.array([])
            y_detected = np.array([])

        # Zaktualizuj grid na podstawie wykrytych punktów
        for x, y in zip(x_detected, y_detected):
            x = int(round(x // 1))
            y = int(round(y // 1))

            if 0 <= x < self.grid_size[1] and 0 <= y < self.grid_size[0]:
                self.map_grid[y, x] = 1

        # Dylatacja (powiększenie obszarów przeszkód)
        self.map_grid = binary_dilation(self.map_grid, iterations=1)

        distance_threshold = 8
        def count_neighbors(values):
            center = values[len(values) // 2]  # wartość centralna
            neighbors = np.sum(values) - center  # suma sąsiadów (bez środka)
            if center == 1 and neighbors < distance_threshold:
                return 0  # Zamień "1" na "0", jeśli ma mniej niż distance_threshold sąsiadów
            return center

        # Używamy filtra do przetwarzania mapy
        self.map_grid = generic_filter(self.map_grid, count_neighbors, size=10)

        self.map_grid = binary_dilation(self.map_grid, iterations=1)

        return self.map_grid

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

    def process_detected_points(self, filename="output_map.png"):
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

        plt.xlabel('X position (cm)')
        plt.ylabel('Y position (cm)')
        plt.title('Map with Alpha Shape')
        plt.legend()
        plt.grid()
        plt.savefig(filename)
        plt.show()

    def save_map_grid_to_file(self, filename="map_grid.txt"):
        np.savetxt(filename, self.map_grid, fmt='%d', delimiter=' ')

    def get_grid_pos(self, pos):
        pos_x, pos_y = pos

        grid_y = self.grid_size[1]
        pos_y = grid_y - pos_y

        return int(pos_x), int(pos_y)

