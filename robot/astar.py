import numpy as np
import heapq
import matplotlib.pyplot as plt
import time

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

        # Rozpoczęcie i zakończenie ścieżki
        if path:
            path = np.array(path)
            plt.plot(path[:, 0], path[:, 1], 'r-', lw=2, label='Path')

        # Dodanie pozycji robota jako kropki
        robot_x, robot_y = robot_position
        # Przesunięcie pozycji robota na ścieżce
        plt.plot(robot_x, robot_y, 'bo', markersize=10,
                 label='Robot Position')  # (0,0) bo ścieżka jest przesunięta o robot_position

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

    def create_smoothed_path(self, path, segment_length_cm=1):
        smoothed_path = []
        for i in range(len(path) - 1):
            start = np.array(path[i])
            end = np.array(path[i + 1])
            segment_distance = np.linalg.norm(end - start)
            num_segments = int(np.ceil(segment_distance / segment_length_cm))
            for j in range(num_segments):
                ratio = j / num_segments
                new_point = start + ratio * (end - start)
                smoothed_path.append(tuple(new_point))
        smoothed_path.append(path[-1])
        return smoothed_path

    def move_robot_along_path(self, stdscr, motor_controller, path, gyro, resolution=1.0, segment_length_cm=1,
                              angle_tolerance=10):
        stdscr.clear()
        stdscr.addstr(0, 0, "Pathfinding...")
        current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
        stdscr.addstr(2, 0, f'Starting at grid position: {current_position}')

        smoothed_path = self.create_smoothed_path(path, segment_length_cm)
        stdscr.addstr(3, 0, f'Smoothed path value: {len(smoothed_path)}')
        for target_position in smoothed_path:
            target_angle, target_distance = self.calculate_angle_and_distance(current_position, target_position)
            stdscr.addstr(4, 0, f'Target grid position: {target_position}')
            stdscr.addstr(5, 0, f'Calculated angle: {target_angle:.2f}, distance: {target_distance:.2f}')

            target_distance_grid_units = target_distance
            current_angle = gyro.get_angle_z()
            angle_difference = (target_angle - current_angle + 360) % 360
            if abs(angle_difference) > angle_tolerance:
                stdscr.addstr(6, 0, f"Rotating to {target_angle:.2f}°")
                motor_controller.rotate_to_angle(gyro, target_angle=target_angle)
                stdscr.refresh()
                time.sleep(1)

            stdscr.addstr(7, 0, f'Moving forward segment distance (grid units): {target_distance_grid_units:.2f}')
            motor_controller.forward_with_encoders(target_distance_grid_units * 0.1)
            current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
            stdscr.addstr(8, 0, f"Updated grid position: {current_position}")
            stdscr.refresh()
            time.sleep(0.5)
