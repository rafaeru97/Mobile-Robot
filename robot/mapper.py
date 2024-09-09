import matplotlib.pyplot as plt
import numpy as np
import math

import numpy as np
import math


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
        self.positions = [(0, 0)]  # Świetnie, ale warto dodać np. (0,0) jako początkową pozycję
        self.current_angle = 0
        self.x = 0
        self.y = 0
        self.last_encoder_distance = 0

    def update_position(self):
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
        if distance_from_sensor is not None and distance_from_sensor > 0 and distance_from_sensor < 100:
            distance_from_sensor_cm = distance_from_sensor
            detected_x = self.x + distance_from_sensor_cm * math.cos(angle_rad)
            detected_y = self.y + distance_from_sensor_cm * math.sin(angle_rad)

            # Debugowanie
            print(f"Detected point: ({detected_x}, {detected_y})")

            self.slam.update((dx, dy, angle_rad), [(detected_x, detected_y, distance_from_sensor_cm)])

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

        plt.title("Robot Movement Path with Detected Points (in cm)")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True)
        plt.legend()
        plt.savefig(filename)
        plt.close()

    def process_detected_points(self, detected_points, eps=5, min_samples=3, zoom_level=100, filename="output_map.png"):
        """
        Process the detected points by filtering noise, estimating boundaries, and detecting objects.
        :param detected_points: List of (x, y, distance) points.
        :param eps: Maximum distance between two samples for DBSCAN to cluster them.
        :param min_samples: Minimum number of samples in a cluster for it not to be considered noise.
        :param zoom_level: Zoom level for the map visualization.
        :param filename: The name of the output image file.
        :return: None (saves a visual map with boundaries and detected objects).
        """
        if len(detected_points) == 0:
            print("No detected points to process.")
            return

        # Step 1: Convert detected points to a numpy array and apply DBSCAN for filtering
        detected_array = np.array(detected_points)
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(detected_array[:, :2])

        # Filter out noise points (label = -1)
        filtered_points = detected_array[clustering.labels_ != -1]

        if len(filtered_points) == 0:
            print("No valid points after filtering.")
            return

        # Step 2: Estimate boundaries using Convex Hull
        hull = ConvexHull(filtered_points[:, :2])

        # Step 3: Detect objects by clustering points and generating bounding boxes
        boxes = self.get_bounding_boxes(filtered_points, clustering)

        # Step 4: Plot the map with boundaries and bounding boxes
        self.plot_map_with_boundaries_and_boxes(filtered_points, hull, boxes, zoom_level, filename)

    def get_bounding_boxes(self, filtered_points, clustering):
        """
        Generate bounding boxes for filtered clusters of points.
        :param filtered_points: List of (x, y, distance) points.
        :param clustering: DBSCAN clustering result.
        :return: List of bounding boxes [(x_min, y_min, x_max, y_max)].
        """
        clusters = np.array(filtered_points)
        boxes = []

        # Get unique clusters from DBSCAN
        for cluster in np.unique(clustering.labels_):
            if cluster == -1:
                continue  # Skip noise points

            cluster_points = clusters[clustering.labels_ == cluster][:, :2]
            x_min, y_min = cluster_points.min(axis=0)
            x_max, y_max = cluster_points.max(axis=0)
            boxes.append((x_min, y_min, x_max, y_max))

        return boxes

    def plot_map_with_boundaries_and_boxes(self, filtered_points, hull, boxes, zoom_level, filename):
        """
        Plot the filtered points, convex hull boundaries, and bounding boxes.
        :param filtered_points: Filtered points after noise reduction.
        :param hull: Convex Hull object for boundary estimation.
        :param boxes: Bounding boxes for detected objects.
        :param zoom_level: Zoom level for map visualization.
        :param filename: Name of the output file.
        """
        plt.figure(figsize=(8, 8))

        # Plot the filtered points
        plt.scatter(filtered_points[:, 0], filtered_points[:, 1], color='g', label="Filtered Points", s=30)

        # Plot the convex hull boundary
        for simplex in hull.simplices:
            plt.plot(filtered_points[simplex, 0], filtered_points[simplex, 1], 'k-', label="Boundary")

        # Plot the bounding boxes
        for box in boxes:
            x_min, y_min, x_max, y_max = box
            plt.plot([x_min, x_max, x_max, x_min, x_min],
                     [y_min, y_min, y_max, y_max, y_min], 'r-', label="Bounding Box")

        # Adjust zoom and labels
        center_x, center_y = filtered_points[-1, 0], filtered_points[-1, 1]
        plt.xlim(center_x - zoom_level, center_x + zoom_level)
        plt.ylim(center_y - zoom_level, center_y + zoom_level)

        plt.title("Processed Map with Boundaries and Objects")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True)
        plt.legend()
        plt.savefig(filename)
        plt.close()

# Example of using plot_map_with_boundaries_and_boxes manually
def generate_and_plot_map(self):
    # Get the detected points from SLAM
    detected_points = np.array(self.slam.get_map())

    if len(detected_points) == 0:
        print("No detected points available.")
        return

    # Step 1: Perform DBSCAN clustering (for noise reduction)
    eps = 5  # You can adjust this value based on your data
    min_samples = 3  # Minimum samples for a valid cluster
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(detected_points[:, :2])

    # Step 2: Filter points (removing noise)
    filtered_points = detected_points[clustering.labels_ != -1]

    if len(filtered_points) == 0:
        print("No valid points after filtering.")
        return

    # Step 3: Compute convex hull (for boundary estimation)
    hull = ConvexHull(filtered_points[:, :2])

    # Step 4: Generate bounding boxes for object detection
    boxes = self.get_bounding_boxes(filtered_points, clustering)

    # Step 5: Plot the results
    zoom_level = 100  # You can adjust the zoom level as needed
    filename = "output_map.png"  # You can specify the file name here
    self.plot_map_with_boundaries_and_boxes(filtered_points, hull, boxes, zoom_level, filename)

