import matplotlib.pyplot as plt
import numpy as np
import math


class Mapper:
    def __init__(self, motor_controller, gyro):
        self.motor_controller = motor_controller
        self.gyro = gyro
        self.positions = [(0, 0)]  # Starting at the origin (0,0)
        self.current_angle = 0  # Initial angle is 0 degrees
        self.x = 0
        self.y = 0
        self.angle_history = []

    def update_position(self):
        # Get current angle from the gyro in degrees
        angle = self.gyro.get_angle_z()
        self.angle_history.append(angle)
        if len(self.angle_history) > 10:  # Average over last 10 readings
            self.angle_history.pop(0)
        self.current_angle = np.mean(self.angle_history)

        # Get distance traveled in meters from the motor controller
        distance = self.motor_controller.getEncoderDistance()

        # Convert angle to radians for trigonometric functions
        angle_rad = math.radians(self.current_angle)

        # Calculate the change in x and y using the distance and angle
        dx = distance * math.cos(angle_rad)
        dy = distance * math.sin(angle_rad)

        # Update the current position with the calculated values
        self.x += dx
        self.y += dy

        # Append the new position (rounded to centimeters) to the list
        self.positions.append((round(self.x * 100, 2), round(self.y * 100, 2)))

    def create_map(self, filename="robot_map.png"):
        # Convert positions to numpy arrays for plotting
        positions = np.array(self.positions)
        x_positions = positions[:, 0]  # X coordinates (in cm)
        y_positions = positions[:, 1]  # Y coordinates (in cm)

        # Calculate the end of the orientation arrow
        orientation_length = 5  # Length of the orientation arrow in cm
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
        plt.arrow(x_positions[-1], y_positions[-1], dx_arrow, dy_arrow, head_width=1, head_length=1, fc='k', ec='k',
                  label="Orientation")

        plt.title("Robot Movement Path (in cm)")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True)
        plt.legend()  # Show legend

        # Save the figure as a PNG file
        plt.savefig(filename)
        plt.close()
