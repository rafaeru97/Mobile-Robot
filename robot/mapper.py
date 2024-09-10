from scipy.spatial import ConvexHull
import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import json

from scipy.spatial import Delaunay
from scipy.spatial.distance import euclidean
from collections import deque



def alpha_shape(points, alpha):
    """
    Calculate the alpha shape (concave hull) of a set of points.
    :param points: np.array of shape (n, 2) points.
    :param alpha: alpha value to control the shape of the hull.
    :return: Set of edges representing the alpha shape.
    """
    if len(points) < 4:
        # No alpha shape possible, return a convex hull.
        return Delaunay(points).convex_hull

    tri = Delaunay(points)
    edges = set()
    for ia, ib, ic in tri.simplices:
        a = points[ia]
        b = points[ib]
        c = points[ic]
        # Length of triangle edges
        ab = np.linalg.norm(a - b)
        bc = np.linalg.norm(b - c)
        ca = np.linalg.norm(c - a)
        # Circumradius of the triangle
        s = (ab + bc + ca) / 2.0
        area = max(s * (s - ab) * (s - bc) * (s - ca), 0.00001)
        circum_r = ab * bc * ca / (4.0 * np.sqrt(area))
        if circum_r < 1.0 / alpha:
            edges.add((ia, ib))
            edges.add((ib, ic))
            edges.add((ic, ia))
    return edges

def plot_alpha_shape(points, alpha, filename="alpha_shape_map.png"):
    """
    Plot the alpha shape of the given points.
    :param points: Array of detected points.
    :param alpha: Alpha value for the shape.
    :param filename: Output image file.
    """
    plt.figure(figsize=(8, 8))
    # Plot points
    plt.scatter(points[:, 0], points[:, 1], color='g', s=30, label="Detected Points")

    # Compute and plot alpha shape
    edges = alpha_shape(points, alpha)
    for i, j in edges:
        plt.plot([points[i, 0], points[j, 0]], [points[i, 1], points[j, 1]], 'k-')

    plt.title("Alpha Shape Map")
    plt.xlabel("X position (cm)")
    plt.ylabel("Y position (cm)")
    plt.grid(True)
    plt.savefig(filename)
    plt.close()


class SimpleDBSCAN:
    def __init__(self, eps=5, min_samples=3):
        self.eps = eps
        self.min_samples = min_samples

    def fit(self, points):
        labels = np.full(len(points), -1)  # -1 indicates noise
        cluster_id = 0

        for i in range(len(points)):
            if labels[i] != -1:
                continue

            neighbors = self._region_query(points, i)
            if len(neighbors) < self.min_samples:
                labels[i] = -1
            else:
                self._expand_cluster(points, labels, i, neighbors, cluster_id)
                cluster_id += 1

        return labels

    def _region_query(self, points, index):
        neighbors = []
        for i, point in enumerate(points):
            if euclidean(points[index], point) < self.eps:
                neighbors.append(i)
        return neighbors

    def _expand_cluster(self, points, labels, index, neighbors, cluster_id):
        labels[index] = cluster_id
        queue = deque(neighbors)

        while queue:
            point_index = queue.popleft()
            if labels[point_index] == -1:
                labels[point_index] = cluster_id
            elif labels[point_index] != -1:
                continue

            point_neighbors = self._region_query(points, point_index)
            if len(point_neighbors) >= self.min_samples:
                queue.extend(point_neighbors)

        return labels


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
            with open(filename, "w") as f:
                json.dump(self.detected_points, f)

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

        # Przetwarzanie wczytanych punktów
        if len(loaded_points) == 0:
            print("Brak wczytanych punktów do przetworzenia.")
            return

        # Zamiana listy na numpy array
        detected_array = np.array(loaded_points)

        # Wykorzystanie tej samej logiki co w `process_detected_points`
        self.detected_points = detected_array  # Zapisz wczytane punkty do atrybutu detected_points
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
        if distance_from_sensor is not None and distance_from_sensor > 0:
            distance_from_sensor_cm = distance_from_sensor
            detected_x = self.x + distance_from_sensor_cm * math.cos(angle_rad)
            detected_y = self.y + distance_from_sensor_cm * math.sin(angle_rad)
            self.detected_points.append((detected_x, detected_y, distance_from_sensor_cm))
            self.slam.update((dx, dy, angle_rad), [(detected_x, detected_y, distance_from_sensor_cm)])

    def process_detected_points(self, eps=5, min_samples=3, alpha=1.0, zoom_level=100, filename="output_map.png"):
        """
        Process the detected points by filtering noise, estimating boundaries, and detecting objects.
        :param eps: Maximum distance between two samples for DBSCAN to cluster them.
        :param min_samples: Minimum number of samples in a cluster for it not to be considered noise.
        :param alpha: Alpha value for the alpha shape algorithm (for concave boundary estimation).
        :param zoom_level: Zoom level for the map visualization.
        :param filename: The name of the output image file.
        :return: None (saves a visual map with boundaries and detected objects).
        """
        if len(self.detected_points) == 0:
            return

        # Convert detected points to a numpy array and apply SimpleDBSCAN for filtering
        detected_array = np.array(self.detected_points)
        clustering = SimpleDBSCAN(eps=eps, min_samples=min_samples)
        labels = clustering.fit(detected_array[:, :2])

        if len(labels) != len(detected_array):
            return

        # Filter out noise points (label = -1)
        filtered_points = detected_array[np.array(labels) != -1]

        if len(filtered_points) == 0:
            return

        # Estimate boundaries using Alpha Shape (instead of Convex Hull)
        plot_alpha_shape(filtered_points[:, :2], alpha, filename=filename)

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