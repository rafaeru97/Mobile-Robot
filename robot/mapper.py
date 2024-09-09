import matplotlib.pyplot as plt
import numpy as np
import math


class Mapper:
    def __init__(self, motor_controller, gyro, distance_sensor):
        self.motor_controller = motor_controller
        self.gyro = gyro
        self.distance_sensor = distance_sensor  # Sensor odległości
        self.positions = [(0, 0)]  # Starting at the origin (0,0)
        self.detected_points = []  # Lista na punkty wykryte przez sensor odległości
        self.current_angle = 0  # Initial angle is 0 degrees
        self.x = 0
        self.y = 0
        self.angle_history = []
        self.last_encoder_distance = 0  # Track the last encoder distance

    def update_position(self):
        # Pobierz aktualny kąt z żyroskopu w stopniach
        self.current_angle = self.gyro.get_angle_z()

        # Pobierz dystans przebyty przez robota od enkodera w metrach
        current_distance = self.motor_controller.getEncoderDistance()

        # Oblicz różnicę dystansu od ostatniej aktualizacji (w metrach)
        distance = current_distance - self.last_encoder_distance
        self.last_encoder_distance = current_distance  # Zaktualizuj dystans

        # Konwertuj dystans na centymetry
        distance_cm = distance * 100  # 1 metr = 100 centymetrów

        # Konwertuj kąt na radiany
        angle_rad = math.radians(self.current_angle)

        # Oblicz zmianę w x i y na podstawie dystansu i kąta
        dx = distance_cm * math.cos(angle_rad)
        dy = distance_cm * math.sin(angle_rad)

        # Zaktualizuj aktualną pozycję
        self.x += dx
        self.y += dy

        # Dodaj nową pozycję (zaokrągloną do cm) do listy
        self.positions.append((round(self.x, 2), round(self.y, 2)))

        # Pobierz dystans z sensora odległości
        distance_from_sensor = self.distance_sensor.get_distance()

        # Oblicz pozycję wykrytego punktu, jeśli sensor zarejestruje dystans
        if distance_from_sensor is not None and distance_from_sensor > 0:
            # Przelicz dystans z sensora na centymetry
            distance_from_sensor_cm = distance_from_sensor  # Zakładamy, że sensor zwraca w cm

            detected_x = self.x + distance_from_sensor_cm * math.cos(angle_rad)
            detected_y = self.y + distance_from_sensor_cm * math.sin(angle_rad)

            # Dodaj wykryty punkt do listy
            self.detected_points.append((round(detected_x, 2), round(detected_y, 2)))

    def create_map(self, filename="robot_map.png", zoom_level=100):
        # Convert positions to numpy arrays for plotting
        positions = np.array(self.positions)
        x_positions = positions[:, 0]  # X coordinates (in cm)
        y_positions = positions[:, 1]  # Y coordinates (in cm)

        # Convert detected points to numpy arrays for plotting
        detected_points = np.array(self.detected_points)
        if len(detected_points) > 0:
            x_detected = detected_points[:, 0]  # X coordinates of detected points (in cm)
            y_detected = detected_points[:, 1]  # Y coordinates of detected points (in cm)

        # Calculate the end of the orientation arrow
        orientation_length = 10  # Length of the orientation arrow in cm
        angle_rad = math.radians(self.current_angle)
        dx_arrow = orientation_length * math.cos(angle_rad)
        dy_arrow = orientation_length * math.sin(angle_rad)

        # Create the plot
        plt.figure(figsize=(8, 8))  # Bigger figure for better visibility
        plt.plot(x_positions, y_positions, marker="o", color="b", markersize=3)  # Path of the robot

        # Highlight the current position
        plt.plot(x_positions[-1], y_positions[-1], marker="o", color="r", markersize=10,
                 label="Current Position")  # Red marker for current position

        # Draw orientation arrow
        plt.arrow(x_positions[-1], y_positions[-1], dx_arrow, dy_arrow, head_width=2, head_length=2, fc='k', ec='k',
                  label="Orientation")

        # Plot the detected points (e.g. obstacles)
        if len(detected_points) > 0:
            plt.scatter(x_detected, y_detected, color='g', label="Detected Points",
                        s=30)  # Green dots for detected points

        # Set the limits of the plot to center the robot and provide zoom
        center_x, center_y = x_positions[-1], y_positions[-1]
        plt.xlim(center_x - zoom_level, center_x + zoom_level)
        plt.ylim(center_y - zoom_level, center_y + zoom_level)

        plt.title("Robot Movement Path with Detected Points (in cm)")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True)
        plt.legend()  # Show legend

        # Save the figure as a PNG file
        plt.savefig(filename)
        plt.close()
