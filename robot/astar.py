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

def interpolate_path(path, max_step_size=10.0):
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
        start, end = points[0], points[-1]
        if len(points) < 2:
            return [start]

        distances = [perpendicular_distance(p, start, end) for p in points[1:-1]]
        max_distance = max(distances, default=0)

        if max_distance > epsilon:
            index = distances.index(max_distance) + 1
            return rdp_rec(points[:index + 1], epsilon)[:-1] + rdp_rec(points[index:], epsilon)
        else:
            return [start, end]

    return rdp_rec(points, epsilon)


class AStarPathfinder:
    def __init__(self, map_grid):
        self.map_grid = map_grid  # map_grid to dwuwymiarowa tablica, gdzie 1 oznacza przeszkodę, a 0 wolne miejsce
        self.mapper = None

    def set_mapper(self, mapper):
        self.mapper = mapper

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

                    if neighbor not in dict(open_list):
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

    def visualize_path(self, path, map_grid, robot_position=(100, 100), filename="path_visualization.png"):
        """
        Visualize the path on the map grid and optionally include the robot's position, with the path offset by the robot position.
        :param path: The list of points representing the path.
        :param map_grid: The grid map array to overlay the path on.
        :param robot_position: A tuple (x, y) representing the robot's position in grid coordinates.
        :param filename: The name of the output image file.
        """
        # Stworzenie wykresu
        plt.figure(figsize=(8, 8))

        # Wyświetlenie mapy siatki
        plt.imshow(map_grid, cmap='gray', origin='lower')

        # Ustawienie wartości odniesienia
        ref_y = 100

        # Rozpoczęcie i zakończenie ścieżki
        if path:
            path = np.array(path)
            # path[:, 1] = 2 * ref_y - path[:, 1]  # Obrót wartości y
            path = path[::-1]  # Odwrócenie kolejności punktów w ścieżce
            plt.plot(path[:, 0], path[:, 1], 'r-', lw=2, label='Path')

        # Dodanie pozycji robota jako kropki
        robot_x, robot_y = robot_position
        # robot_y = 2 * ref_y - robot_y  # Obrót wartości y
        plt.plot(robot_x, robot_y, 'bo', markersize=10, label='Robot Position')

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
