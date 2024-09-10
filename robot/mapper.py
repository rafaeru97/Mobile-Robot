import numpy as np
import matplotlib.pyplot as plt
import math
import json

import logging

# Konfiguracja logowania do pliku
logging.basicConfig(filename='mapper_log.txt', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')

from scipy.spatial import distance_matrix, ConvexHull
from shapely.geometry import MultiPoint
import alphashape

from shapely.geometry import LineString
from scipy.interpolate import griddata

def interpolate_points(points, resolution=100):
    """
    Interpolates points to create a smooth grid.
    :param points: Array of detected points.
    :param resolution: The resolution of the interpolated grid.
    :return: Interpolated grid points.
    """
    x = points[:, 0]
    y = points[:, 1]
    grid_x, grid_y = np.mgrid[x.min():x.max():resolution*1j, y.min():y.max():resolution*1j]
    grid_z = griddata((x, y), np.ones(len(x)), (grid_x, grid_y), method='cubic')
    return grid_x, grid_y, grid_z

def smooth_boundary(boundary, smoothing_factor=0.01):
    """
    Smooth the boundary using a smoothing factor.
    :param boundary: Array of boundary coordinates.
    :param smoothing_factor: Smoothing factor for the boundary.
    :return: Smoothed boundary.
    """
    line = LineString(boundary)
    smoothed = line.simplify(smoothing_factor)
    return np.array(smoothed.xy).T

def filter_points(points, min_distance=1.0):
    """
    Filter points based on minimum distance between them.
    :param points: Array of detected points.
    :param min_distance: Minimum distance to keep a point.
    :return: Filtered points.
    """
    filtered = []
    for point in points:
        distances = np.linalg.norm(points - point, axis=1)
        if np.all(distances >= min_distance):
            filtered.append(point)
    return np.array(filtered)

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
            if distance_from_sensor is not None and distance_from_sensor > 0:
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
        logging.info("Start processing detected points")

        if len(self.detected_points) == 0:
            logging.warning("No detected points")
            return

        # Convert detected points to a numpy array
        detected_array = np.array(self.detected_points)
        points = detected_array[:, :2]
        logging.debug(f"Detected points: {points}")

        # Compute distance matrix
        dist_matrix = distance_matrix(points, points)

        # Threshold to consider a point as a neighbor
        threshold = 4  # Adjust this value based on your data

        # Filter points that are isolated
        filtered_points = []
        for i, point in enumerate(points):
            neighbors = np.sum(dist_matrix[i] < threshold)
            if neighbors > 1:  # If more than one neighbor, keep the point
                filtered_points.append(point)

        filtered_points = np.array(filtered_points)
        logging.debug(f"Filtered Points: {filtered_points}")

        if len(filtered_points) < 3:
            logging.warning("Not enough points after filtering to compute Alpha Shape.")
            return

        # Additional filtering to remove outliers
        filtered_points = filter_points(filtered_points, min_distance=0.1)
        logging.debug(f"Filtered Points after Outlier Removal: {filtered_points}")

        if len(filtered_points) < 3:
            logging.warning("Not enough points after outlier removal to compute Alpha Shape.")
            return

        # Generate Alpha Shape
        alpha = 0.1  # Adjust this value to control the level of detail
        alpha_shape = alphashape.alphashape(filtered_points, alpha)
        logging.debug(f"Alpha Shape generated: {alpha_shape}")

        # Convert Alpha Shape to coordinates for plotting
        if alpha_shape.geom_type == 'Polygon':
            boundary = np.array(alpha_shape.exterior.coords)
            logging.debug(f"Alpha Shape boundary (Polygon): {boundary}")
        elif alpha_shape.geom_type == 'MultiPolygon':
            boundary = np.concatenate([np.array(p.exterior.coords) for p in alpha_shape], axis=0)
            logging.debug(f"Alpha Shape boundary (MultiPolygon): {boundary}")
        else:
            boundary = np.array([])
            logging.warning("Alpha shape is not a polygon or multipolygon.")

        # Smooth the boundary if needed
        if boundary.size > 0:
            boundary = smooth_boundary(boundary)
            logging.debug(f"Smoothed boundary: {boundary}")

        # Plotting the results
        try:
            plt.figure(figsize=(8, 8))
            plt.plot(filtered_points[:, 0], filtered_points[:, 1], 'o', label='Filtered Points')
            if boundary.size > 0:
                plt.plot(boundary[:, 0], boundary[:, 1], 'r--', lw=2, label='Alpha Shape')

            plt.xlabel('X Coordinate')
            plt.ylabel('Y Coordinate')
            plt.title('Map with Alpha Shape')
            plt.legend()
            plt.grid()
            plt.savefig(filename)
            plt.show()
            logging.info(f"Map saved as {filename}")
        except Exception as e:
            logging.error(f"Error during plotting or saving: {e}")

        logging.info("Finished processing detected points")

    def get_bounding_boxes(self, points, labels):
        """
        Generate bounding boxes for clusters in the detected points.
        :param points: List of (x, y) points.
        :param labels: Array of cluster labels.
        :return: List of bounding boxes.
        """
        unique_labels = set(labels)
        boxes = []

        for label in unique_labels:
            if label == -1:
                continue  # Skip noise

            # Ensure labels and points have matching lengths
            if len(points) != len(labels):
                continue

            # Create boolean index based on labels
            boolean_index = np.array(labels) == label
            if boolean_index.sum() == 0:
                continue

            # Filter points based on the boolean index
            cluster_points = points[boolean_index]
            min_x, min_y = np.min(cluster_points, axis=0)
            max_x, max_y = np.max(cluster_points, axis=0)

            boxes.append((min_x, min_y, max_x, max_y))

        return boxes

    def plot_map_with_boundaries_and_boxes(self, filtered_points, hull, boxes, zoom_level, filename):
        """
        Plot the map with boundaries and bounding boxes.
        :param filtered_points: Points after noise filtering.
        :param hull: Convex Hull object.
        :param boxes: List of bounding boxes.
        :param zoom_level: Zoom level for the map visualization.
        :param filename: The name of the output image file.
        """
        plt.figure(figsize=(8, 8))
        # plt.plot(filtered_points[:, 0], filtered_points[:, 1], marker="o", color="b", markersize=3, label="Filtered Points")

        # Plot Convex Hull
        for simplex in hull.simplices:
            plt.plot(filtered_points[simplex, 0], filtered_points[simplex, 1], 'k--', label="Boundary")

        # Plot bounding boxes
        for (min_x, min_y, max_x, max_y) in boxes:
            plt.plot([min_x, max_x], [min_y, min_y], 'r-', lw=2)
            plt.plot([max_x, max_x], [min_y, max_y], 'r-', lw=2)
            plt.plot([max_x, min_x], [max_y, max_y], 'r-', lw=2)
            plt.plot([min_x, min_x], [max_y, min_y], 'r-', lw=2)

        center_x, center_y = self.positions[-1]
        plt.xlim(center_x - zoom_level, center_x + zoom_level)
        plt.ylim(center_y - zoom_level, center_y + zoom_level)

        plt.title("Processed Map")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True)
        plt.savefig(filename)
        plt.close()