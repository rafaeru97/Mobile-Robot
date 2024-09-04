import numpy as np
import matplotlib.pyplot as plt


class Mapper:
    def __init__(self, map_size=(100, 100), resolution=0.1):
        # map_size jest w liczbie kratek
        self.map_size = map_size
        self.resolution = resolution  # Rozdzielczość w metrach
        self.map = np.zeros(map_size, dtype=int)
        self.position = np.array([map_size[0] // 2, map_size[1] // 2], dtype=float)  # Początkowa pozycja na środku mapy
        self.orientation = 0  # Kąt orientacji robota w radianach

    def update_position(self, distance):
        print("update_position called")
        print(f"Current position: {self.position}")
        print(f"Distance: {distance}, Orientation: {self.orientation}")

        # Zamień metry na jednostki mapy (krateczki)
        delta_x = distance * np.cos(self.orientation) / self.resolution
        delta_y = distance * np.sin(self.orientation) / self.resolution

        print(f"Delta X: {delta_x}, Delta Y: {delta_y}")

        # Zaktualizuj pozycję
        self.position = self.position.astype(np.float64)  # Zapewnij, że pozycja jest typu float64
        self.position += np.array([delta_x, delta_y], dtype=np.float64)

        # Zaokrąglij pozycję do najbliższej jednostki
        self.position = np.round(self.position).astype(int)

        # Upewnij się, że pozycja nie wykracza poza granice mapy
        self.position = np.clip(self.position, [0, 0], np.array(self.map_size) - 1)

        print(f"Updated position: {self.position}")

    def update_orientation(self, angle):
        print("update_orientation called")
        print(f"Current orientation: {self.orientation}")
        print(f"Angle to add: {angle}")

        # Zaktualizuj orientację
        self.orientation = (self.orientation + angle) % (2 * np.pi)

        print(f"Updated orientation: {self.orientation}")

    def update_map(self, distance, angle_offset=0):
        print("update_map called")
        print(f"Current position: {self.position}")
        print(f"Distance: {distance}, Angle offset: {angle_offset}")

        angle = self.orientation + angle_offset

        # Zamień metry na jednostki mapy (krateczki)
        obstacle_x = int(self.position[0] + (distance * np.cos(angle)) / self.resolution)
        obstacle_y = int(self.position[1] + (distance * np.sin(angle)) / self.resolution)

        print(f"Obstacle position: X={obstacle_x}, Y={obstacle_y}")

        # Sprawdź, czy pozycja przeszkody mieści się w granicach mapy
        if 0 <= obstacle_x < self.map_size[0] and 0 <= obstacle_y < self.map_size[1]:
            self.map[obstacle_x, obstacle_y
