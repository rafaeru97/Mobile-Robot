import numpy as np
import heapq
import matplotlib.pyplot as plt


class AStarPathfinder:
    def __init__(self, map_grid):
        self.map_grid = map_grid  # map_grid to dwuwymiarowa tablica, gdzie 1 oznacza przeszkodę, a 0 wolne miejsce

    def heuristic(self, a, b):
        # Odległość euklidesowa
        return np.linalg.norm(np.array(a) - np.array(b))

    def astar(self, start, goal):
        # Kolejka priorytetowa
        open_list = []
        heapq.heappush(open_list, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor not in [i[1] for i in open_list]:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return []  # Nie znaleziono ścieżki

    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]  # Ruchy w 4 kierunkach
        return [n for n in neighbors if self.is_valid(n)]

    def is_valid(self, node):
        x, y = node
        return 0 <= x < self.map_grid.shape[0] and 0 <= y < self.map_grid.shape[1] and self.map_grid[x, y] == 0

    def distance(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def visualize_path(self, path, filename="path_visualization.png", resolution=1.0):
        """
        Visualize the path on the map grid and save the result to a file.
        :param path: List of tuples representing the path as [(x1, y1), (x2, y2), ...]
        :param filename: The name of the output image file.
        :param resolution: The resolution of the grid in the same units as the detected points.
        :return: None
        """
        # Generate the map grid
        map_grid = self.generate_map_grid(resolution=resolution)

        if map_grid is None:
            print("Brak danych do wizualizacji.")
            return

        # Plot the map grid
        plt.figure(figsize=(8, 8))
        plt.imshow(map_grid, cmap='Greys', origin='lower', interpolation='none')

        # Extract path coordinates
        path = np.array(path)
        if len(path) > 0:
            x_path, y_path = path[:, 0], path[:, 1]

            # Plot the path
            plt.plot(x_path, y_path, 'r-', marker='o', markersize=5, label='Path')

        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Path Visualization on Map Grid')
        plt.legend()
        plt.grid(True)
        plt.savefig(filename)
        plt.show()
