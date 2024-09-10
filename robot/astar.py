import numpy as np
import heapq
import matplotlib.pyplot as plt
import math
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

    def visualize_path(self, path, map_grid, robot_position=None, filename="path_visualization.png"):
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
        if path and robot_position:
            path = np.array(path)
            robot_x, robot_y = robot_position

            # Odjęcie współrzędnych robota od ścieżki, aby zcentrować na pozycji robota
            path[:, 0] -= robot_y
            path[:, 1] -= robot_x

            plt.plot(path[:, 1], path[:, 0], 'r-', lw=2, label='Path')

        # Dodanie pozycji robota jako kropki
        if robot_position:
            robot_x, robot_y = robot_position
            plt.plot(robot_x, robot_y, 'bo', markersize=10, label='Robot Position')

        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Path Visualization')
        plt.legend()
        plt.grid(True)
        plt.savefig(filename)
        plt.close()

    def calculate_angle_and_distance(self, start, end):
        """
        Calculate the angle and distance between two points.
        :param start: Tuple (x1, y1) representing the start position.
        :param end: Tuple (x2, y2) representing the end position.
        :return: Tuple (angle, distance) where angle is the direction to face and distance is the length to move.
        """
        dx = end[0] - start[0]
        dy = end[1] - start[1]

        distance = np.sqrt(dx ** 2 + dy ** 2) * 0.01  # Convert from cm to meters
        angle = np.degrees(np.arctan2(dy, dx))

        return angle, distance

    def forward_with_encoders(self, target_distance, base_speed=50, timeout=30):
        """
        Move the robot forward using encoders until the target distance is reached.
        :param target_distance: Distance to move in meters.
        :param base_speed: Base speed for the motors.
        :param timeout: Maximum time to move before stopping.
        """
        self.drive(base_speed)

        start_time = time.time()
        try:
            while True:
                left_distance = abs(self.leftEncoder.get_distance() - self.last_left_distance)
                right_distance = abs(self.rightEncoder.get_distance() - self.last_right_distance)

                # Calculate total distance covered
                total_distance = (left_distance + right_distance) / 2

                # Update last distances
                self.last_left_distance = self.leftEncoder.get_distance()
                self.last_right_distance = self.rightEncoder.get_distance()

                logging.debug(f"Left distance: {left_distance}, Right distance: {right_distance}")
                logging.debug(f"Total distance covered: {total_distance}")

                if total_distance >= target_distance:
                    self.mapper.update_position(target_distance)
                    logging.debug(f"Target distance reached: {total_distance}")
                    break

                elapsed_time = time.time() - start_time
                if elapsed_time > timeout:
                    self.mapper.update_position(total_distance)
                    logging.debug(f"Timeout reached, final distance covered: {total_distance}")
                    break

                time.sleep(0.02)

        finally:
            self.stop()
