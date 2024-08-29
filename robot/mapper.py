import numpy as np
import matplotlib.pyplot as plt

class Mapper:
    def __init__(self, map_size=(100, 100), resolution=1):
        self.map_size = map_size
        self.resolution = resolution
        self.map = np.zeros(map_size, dtype=int)
        self.position = np.array([map_size[0] // 2, map_size[1] // 2], dtype=float)  # Zmieniono dtype na float
        self.orientation = 0  # Robot's heading angle in radians

    def update_position(self, distance):
        # Calculate the change in x and y based on the distance and orientation
        delta_x = distance * np.cos(self.orientation)
        delta_y = distance * np.sin(self.orientation)

        # Update position
        self.position += np.array([delta_x, delta_y]) * self.resolution
        self.position = np.clip(self.position, [0, 0], np.array(self.map_size) - 1)

    def update_orientation(self, angle):
        # Update robot's orientation based on the given angle
        self.orientation = (self.orientation + angle) % (2 * np.pi)

    def update_map(self, distance, angle_offset=0):
        # Calculate the position of the obstacle based on the current position, orientation, and distance
        angle = self.orientation + angle_offset
        obstacle_x = int(self.position[0] + distance * np.cos(angle))
        obstacle_y = int(self.position[1] + distance * np.sin(angle))

        if 0 <= obstacle_x < self.map_size[0] and 0 <= obstacle_y < self.map_size[1]:
            self.map[obstacle_x, obstacle_y] = 1  # Mark the obstacle on the map

    def get_map(self):
        return self.map

    def save_map_as_png(self, filename='map.png'):
        plt.imshow(self.map, cmap='Greys')
        plt.axis('off')  # Hide axes
        plt.savefig(filename, bbox_inches='tight', pad_inches=0)
        plt.close()
