import numpy as np
import heapq
import matplotlib.pyplot as plt
import time
from typing import List, Tuple

import logging

# Skonfiguruj logger, jeśli jeszcze tego nie zrobiłeś
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot_debug.log'),
        logging.StreamHandler()
    ]
)
logging.getLogger('matplotlib').setLevel(logging.WARNING)

def interpolate_path(path: List[Tuple[float, float]], max_step_size: float = 10.0) -> List[Tuple[int, int]]:
    """
    Interpoluje ścieżkę, dodając dodatkowe punkty wzdłuż ścieżki na podstawie maksymalnego rozmiaru kroku.
    :param path: Lista punktów reprezentujących ścieżkę.
    :param max_step_size: Maksymalny rozmiar kroku między punktami interpolowanymi.
    :return: Lista punktów z interpolowaną ścieżką.
    """
    if len(path) < 2:
        return path  # Zwraca ścieżkę bez zmian, jeśli ma mniej niż 2 punkty

    interpolated_path = []
    for i in range(len(path) - 1):
        start = np.array(path[i])
        end = np.array(path[i + 1])
        segment_distance = np.linalg.norm(end - start)
        num_segments = int(np.ceil(segment_distance / max_step_size))
        for j in range(num_segments):
            ratio = j / num_segments
            new_point = start + ratio * (end - start)
            interpolated_path.append(tuple(np.round(new_point).astype(int)))  # Zaokrąglenie do najbliższej liczby całkowitej
    interpolated_path.append(path[-1])  # Dodanie ostatniego punktu

    return interpolated_path



def rdp(points: List[Tuple[float, float]], epsilon: float) -> List[Tuple[float, float]]:
    """
    Apply the Ramer-Douglas-Peucker algorithm to simplify a path.
    :param points: List of points representing the path.
    :param epsilon: The maximum distance between the original path and the simplified path.
    :return: The simplified path.
    """

    def perpendicular_distance(point: Tuple[float, float], line_start: Tuple[float, float],
                               line_end: Tuple[float, float]) -> float:
        if line_start == line_end:
            return np.linalg.norm(np.array(point) - np.array(line_start))
        else:
            line_vec = np.array(line_end) - np.array(line_start)
            point_vec = np.array(point) - np.array(line_start)
            line_len = np.linalg.norm(line_vec)
            proj = np.dot(point_vec, line_vec) / line_len
            proj = np.clip(proj, 0, line_len)
            proj_vec = proj * (line_vec / line_len)
            return np.linalg.norm(point_vec - proj_vec)

    def rdp_rec(points: List[Tuple[float, float]], epsilon: float) -> List[Tuple[float, float]]:
        if len(points) < 2:
            return points

        start, end = points[0], points[-1]

        if len(points) == 2:
            return [start, end]

        distances = [perpendicular_distance(p, start, end) for p in points[1:-1]]
        max_distance = max(distances, default=0)

        if max_distance > epsilon:
            index = distances.index(max_distance) + 1
            return rdp_rec(points[:index + 1], epsilon)[:-1] + rdp_rec(points[index:], epsilon)
        else:
            return [start, end]

    return rdp_rec(points, epsilon)

class AStarPathfinder:
    def __init__(self, map_grid, resolution=1.0, offset_x=100, offset_y=100):
        self.map_grid = map_grid
        self.resolution = resolution
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.mapper = None

    def set_mapper(self, mapper):
        self.mapper = mapper

    def heuristic(self, a, b):
        """Oblicza odległość Manhattan pomiędzy dwoma punktami na siatce."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, start, goal, output_file='path.txt'):
        """Główna funkcja A* z uwzględnieniem odbicia Y i offsetu."""
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)

        # Kolejka priorytetowa
        open_list = []
        heapq.heappush(open_list, (0, start_grid))

        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal_grid:
                path = self.reconstruct_path(came_from, current)

                # Zapisanie ścieżki do pliku
                with open(output_file, 'w') as f:
                    for point in path:
                        f.write(f"{point[0]}, {point[1]}\n")

                return path

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)

                    if neighbor not in dict(open_list):
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return []  # Nie znaleziono ścieżki

    def get_neighbors(self, node):
        """Znajduje sąsiadów danego węzła, uwzględniając ruch po skosie."""
        x, y = node
        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                     (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)]  # Ruchy po skosie
        return [n for n in neighbors if self.is_valid(n)]

    def is_valid(self, node):
        """Sprawdza, czy dany węzeł jest w granicach mapy i nie jest przeszkodą."""
        x, y = node
        return 0 <= x < self.map_grid.shape[1] and 0 <= y < self.map_grid.shape[0] and self.map_grid[y, x] == 0

    def distance(self, a, b):
        """Oblicza odległość Euklidesową pomiędzy dwoma punktami."""
        return np.linalg.norm(np.array(a) - np.array(b))

    def reconstruct_path(self, came_from, current):
        """Odtwarza ścieżkę na podstawie odwiedzonych węzłów."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return [self.grid_to_world(p) for p in path[::-1]]

    def world_to_grid(self, point):
        """Przekształca współrzędne świata na współrzędne siatki, uwzględniając offset i skalę."""
        world_x, world_y = point
        grid_x = int((world_x - self.offset_x) / self.resolution)
        grid_y = int((world_y - self.offset_y) / self.resolution)

        # Odbicie w osi Y (w stosunku do siatki)
        grid_y = self.map_grid.shape[0] - 1 - grid_y

        return grid_x, grid_y

    def grid_to_world(self, point):
        """Przekształca współrzędne siatki z powrotem na współrzędne świata."""
        grid_x, grid_y = point

        # Odbicie w osi Y
        world_y = self.map_grid.shape[0] - 1 - grid_y
        world_x = grid_x

        # Zamiana siatki na współrzędne świata
        world_x = world_x * self.resolution + self.offset_x
        world_y = world_y * self.resolution + self.offset_y

        return world_x, world_y

    def visualize_path(self, path, map_grid, robot_position=(100, 100), filename="path_visualization.png", center_x=100,
                       center_y=100):
        """
        Visualize the path on the map grid and optionally include the robot's position.
        :param path: The list of points representing the path.
        :param map_grid: The grid map array to overlay the path on.
        :param robot_position: A tuple (x, y) representing the robot's position in grid coordinates.
        :param filename: The name of the output image file.
        :param center_x: X-coordinate of the center of the grid in visualization.
        :param center_y: Y-coordinate of the center of the grid in visualization.
        """
        plt.figure(figsize=(8, 8))
        plt.imshow(map_grid, cmap='gray', origin='upper')  # Origin is 'lower' to match (0,0) at bottom-left

        if path:
            path = np.array(path)
            path[:, 1] = 2 * 100 - path[:, 1]  # Odbicie względem osi Y
            plt.plot(path[:, 0], path[:, 1], 'y-', lw=2, label='Path')

        robot_x, robot_y = robot_position
        robot_y = 2 * 100 - robot_y
        plt.plot(robot_x, robot_y, marker="s", color="r", markersize=25, label='Current Position')

        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Path Visualization')
        plt.legend()
        plt.grid(True)
        plt.savefig(filename)
        plt.close()

    def calculate_angle_and_distance(self, current_position, target_position):
        """
        Calculate the angle and distance from the current position to the target position.
        :param current_position: Tuple (x, y) representing the current position.
        :param target_position: Tuple (x, y) representing the target position.
        :return: Tuple (angle, distance) where angle is the direction to the target and distance is the straight-line distance.
        """
        if current_position is None or target_position is None:
            return None, None

        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = np.sqrt(dx ** 2 + dy ** 2)
        angle = np.degrees(np.arctan2(dy, dx))

        # Normalize angle to [0, 360) degrees
        angle = (angle + 360) % 360

        return angle, distance

    def move_robot_along_path(self, stdscr, motor_controller, path, gyro, resolution=1.0, angle_tolerance=5,
                              position_tolerance=1.15, final_position_tolerance=0.5):
        stdscr.clear()
        stdscr.addstr(0, 0, "Pathfinding...")
        current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
        stdscr.addstr(2, 0, f'Starting at grid position: {current_position}')

        # Wygładź i interpoluj ścieżkę
        simplified_path = rdp(path, epsilon=8.0)
        smoothed_path = interpolate_path(simplified_path, max_step_size=20.0)

        for i, target_position in enumerate(smoothed_path):
            target_position = tuple(
                map(int, target_position))  # Upewnij się, że target_position to tuple z liczbami całkowitymi
            target_angle, target_distance = self.calculate_angle_and_distance(current_position, target_position)
            stdscr.addstr(3, 0, f'Target grid position: {target_position}')
            stdscr.addstr(4, 0, f'Calculated angle: {target_angle:.2f}, distance: {target_distance:.2f}')

            current_angle = gyro.get_angle_z()
            angle_difference = (target_angle - current_angle + 360) % 360
            if abs(angle_difference) > angle_tolerance:
                stdscr.addstr(5, 0, f"Rotating to {target_angle:.2f}°")
                stdscr.refresh()
                motor_controller.rotate_to_angle(gyro, target_angle=target_angle)
                time.sleep(0.5)

            stdscr.addstr(6, 0, f"Moving forward {target_distance:.2f} cm")
            motor_controller.forward_with_encoders(target_distance * 0.01)

            current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
            stdscr.addstr(8, 0, f"Updated grid position: {current_position}")
            stdscr.refresh()

            # Jeśli to ostatni punkt w ścieżce, zastosuj mniejszą tolerancję
            if i == len(smoothed_path) - 1:
                final_distance = np.linalg.norm(np.array(target_position) - np.array(current_position))
                if final_distance > final_position_tolerance:
                    stdscr.addstr(9, 0, f"Compensating for final distance: {final_distance:.2f} cm")
                    motor_controller.forward_with_encoders(final_distance * 0.01)
                    stdscr.refresh()
                    time.sleep(0.5)
