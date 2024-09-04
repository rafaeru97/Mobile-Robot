import numpy as np
import matplotlib.pyplot as plt


class Mapper:
    def __init__(self, map_size=(100, 100), resolution=0.1):
        # map_size jest w liczbie kratek
        self.map_size = map_size
        self.resolution = resolution  # Rozdzielczość w metrach
        self.map = np.zeros(map_size, dtype=int)  # Inicjalizacja mapy jako puste
        self.position = np.array([map_size[0] // 2, map_size[1] // 2], dtype=float)  # Początkowa pozycja robota
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
        self.position += np.array([delta_x, delta_y], dtype=float)

        # Zaokrąglij pozycję do najbliższej jednostki
        self.position = np.round(self.position).astype(float)

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

    def update_map(self):
        print("update_map called")
        print(f"Current position: {self.position}")

        # Wyczyść mapę (opcjonalnie, jeśli chcesz, żeby tylko pozycja robota była widoczna)
        self.map.fill(0)

        # Zamień metry na jednostki mapy (krateczki)
        pos_x, pos_y = self.position.astype(int)

        # Upewnij się, że pozycja nie wykracza poza granice mapy
        if 0 <= pos_x < self.map_size[0] and 0 <= pos_y < self.map_size[1]:
            self.map[pos_x, pos_y] = 1  # Zaznacz pozycję robota na mapie
        else:
            print("Robot position out of bounds, not marked on map.")

    def get_map(self):
        return self.map

    def save_map_as_txt(self, filename='map.txt'):
        print(f"Saving map to {filename}")
        with open(filename, 'w') as f:
            for row in self.map:
                line = ''.join(['R' if cell else '.' for cell in row])  # Zmieniamy 1 na 'R' (pozycja robota) i 0 na '.'
                f.write(line + '\n')
        print(f"Map saved as {filename}")

    def save_map_as_png(self, filename='map.png'):
        print(f"Saving map to {filename}")
        plt.imshow(self.map, cmap='Greys')
        plt.axis('off')  # Ukryj osie
        plt.savefig(filename, bbox_inches='tight', pad_inches=0)
        plt.close()
        print(f"Map saved as {filename}")
