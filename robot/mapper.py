import numpy as np
import matplotlib.pyplot as plt

class Mapper:
    def __init__(self, map_size=(100, 100), resolution=0.1):
        self.map_size = map_size
        self.resolution = resolution  # Rozdzielczość w metrach
        self.map = np.zeros(map_size, dtype=int)  # Inicjalizacja mapy jako puste
        self.path_map = np.zeros(map_size, dtype=int)  # Mapa śledzenia drogi
        self.position = np.array([map_size[0] // 2, map_size[1] // 2], dtype=float)  # Początkowa pozycja robota
        self.orientation = 0  # Kąt orientacji robota w radianach

    def update_position(self, distance):
        # Zamień metry na jednostki mapy (krateczki)
        delta_x = distance * np.cos(self.orientation) / self.resolution
        delta_y = distance * np.sin(self.orientation) / self.resolution

        # Zaktualizuj pozycję
        self.position += np.array([delta_x, delta_y], dtype=float)

        # Zaokrąglij pozycję do najbliższej jednostki
        self.position = np.round(self.position).astype(float)

        # Upewnij się, że pozycja nie wykracza poza granice mapy
        self.position = np.clip(self.position, [0, 0], np.array(self.map_size) - 1)

        # Aktualizuj mapę śledzenia drogi
        pos_x, pos_y = self.position.astype(int)
        self.path_map[pos_x, pos_y] = 1  # Oznacz trasę na mapie

    def update_orientation(self, angle):
        # Zaktualizuj orientację
        self.orientation = (self.orientation + angle) % (2 * np.pi)

    def update_map(self):
        # Zamień metry na jednostki mapy (krateczki)
        pos_x, pos_y = self.position.astype(int)

        # Upewnij się, że pozycja nie wykracza poza granice mapy
        if 0 <= pos_x < self.map_size[0] and 0 <= pos_y < self.map_size[1]:
            self.map.fill(0)  # Wyczyść mapę robota
            self.map[pos_x, pos_y] = 1  # Zaznacz pozycję robota na mapie

    def get_map(self):
        return self.map

    def save_map_as_txt(self, filename='map.txt'):
        self.update_map()
        with open(filename, 'w') as f:
            for i in range(self.map_size[0]):
                line = ''.join(['R' if self.map[i, j] else 'o' if self.path_map[i, j] else '.' for j in range(self.map_size[1])])
                f.write(line + '\n')

    def save_map_as_png(self, filename='map.png'):
        self.update_map()
        combined_map = np.maximum(self.map * 255, self.path_map * 128)  # Ścieżka jako szary kolor, robot jako biały
        plt.imshow(combined_map, cmap='Greys')
        plt.axis('off')  # Ukryj osie
        plt.savefig(filename, bbox_inches='tight', pad_inches=0)
        plt.close()
