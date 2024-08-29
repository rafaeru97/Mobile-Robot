import numpy as np
import matplotlib.pyplot as plt


class Mapper:
    def __init__(self, map_size=(100, 100), resolution=0.1):
        # map_size jest w liczbie kratek
        self.map_size = map_size
        self.resolution = resolution  # Rozdzielczość w metrach
        self.map = np.zeros(map_size, dtype=int)
        self.position = np.array([map_size[0] // 2, map_size[1] // 2], dtype=float)  # Zmieniono dtype na float
        self.orientation = 0  # Robot's heading angle in radians

    def update_position(self, distance):
        # Zamień metry na jednostki mapy (krateczki)
        delta_x = distance * np.cos(self.orientation) / self.resolution
        delta_y = distance * np.sin(self.orientation) / self.resolution

        # Zaktualizuj pozycję
        self.position += np.array([delta_x, delta_y], dtype=float)

        # Zaokrąglij pozycję do najbliższej jednostki
        self.position = np.round(self.position).astype(int)

        # Upewnij się, że pozycja nie wykracza poza granice mapy
        self.position = np.clip(self.position, [0, 0], np.array(self.map_size) - 1)

    def update_orientation(self, angle):
        self.orientation = (self.orientation + angle) % (2 * np.pi)

    def update_map(self, distance, angle_offset=0):
        angle = self.orientation + angle_offset
        # Zamień metry na jednostki mapy (krateczki)
        obstacle_x = int(self.position[0] + (distance * np.cos(angle)) / self.resolution)
        obstacle_y = int(self.position[1] + (distance * np.sin(angle)) / self.resolution)

        if 0 <= obstacle_x < self.map_size[0] and 0 <= obstacle_y < self.map_size[1]:
            self.map[obstacle_x, obstacle_y] = 1  # Mark the obstacle on the map

    def get_map(self):
        return self.map

    def save_map_as_png(self, filename='map.png'):
        plt.imshow(self.map, cmap='Greys')
        plt.axis('off')  # Hide axes
        plt.savefig(filename, bbox_inches='tight', pad_inches=0)
        plt.close()
