import matplotlib.pyplot as plt
import numpy as np
import math

class Mapper:
    def __init__(self, motor_controller, gyro):
        self.motor_controller = motor_controller
        self.gyro = gyro
        self.positions = [(0, 0)]  # Initial position
        self.current_angle = 0  # Starting angle
        self.x = 0
        self.y = 0

    def update_position(self):
        # Get current angle from the gyro (in degrees)
        self.current_angle = self.gyro.get_angle_z()

        # Get distance traveled from the motor controller
        distance = self.motor_controller.getEncoderDistance()

        # Convert angle to radians for calculation
        angle_rad = math.radians(self.current_angle)

        # Calculate change in position
        dx = distance * math.cos(angle_rad)
        dy = distance * math.sin(angle_rad)

        # Update x and y positions
        self.x += dx
        self.y += dy

        # Append the new position to the list
        self.positions.append((self.x, self.y))

    def create_map(self, filename="robot_map.png"):
        # Convert positions to numpy arrays for plotting
        positions = np.array(self.positions)
        x_positions = positions[:, 0]
        y_positions = positions[:, 1]

        # Create a plot of the path
        plt.figure(figsize=(6, 6))
        plt.plot(x_positions, y_positions, marker="o")
        plt.title("Robot Movement Path")
        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.grid(True)

        # Save the plot as a PNG file
        plt.savefig(filename)
        plt.close()