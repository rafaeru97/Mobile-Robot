import numpy as np

class Mapper:
    def __init__(self, map_size):
        self.map = np.zeros(map_size)
        self.position = (map_size[0] // 2, map_size[1] // 2)

    def update_position(self, delta_x, delta_y):
        self.position = (self.position[0] + delta_x, self.position[1] + delta_y)

    def update_map(self, distance, angle):
        x = int(self.position[0] + distance * np.cos(angle))
        y = int(self.position[1] + distance * np.sin(angle))
        self.map[x, y] = 1

    def get_map(self):
        return self.map
