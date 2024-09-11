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

    def move_robot_along_path(self, stdscr, motor_controller, path, gyro, resolution=1.0, angle_tolerance=10,
                              position_tolerance=1.5):
        """
        Move the robot along the specified path by navigating to each target position.
        :param stdscr: The curses screen object for updating the terminal interface.
        :param motor_controller: The motor controller for moving the robot.
        :param path: List of points representing the path.
        :param gyro: Gyroscope object for angle measurement.
        :param resolution: The resolution for grid position retrieval.
        :param angle_tolerance: The tolerance for angle adjustment.
        :param position_tolerance: The tolerance for position accuracy.
        """
        stdscr.clear()
        stdscr.addstr(0, 0, "Pathfinding...")
        current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
        stdscr.addstr(2, 0, f'Starting at grid position: {current_position}')

        for target_position in path:
            target_angle, target_distance = self.calculate_angle_and_distance(current_position, target_position)
            stdscr.addstr(3, 0, f'Target grid position: {target_position}')
            stdscr.addstr(4, 0, f'Calculated angle: {target_angle:.2f}, distance: {target_distance:.2f}')

            # Get the current angle from the gyroscope
            current_angle = gyro.get_angle_z()

            # Calculate the angle difference and determine the shortest rotation direction
            angle_difference = (target_angle - current_angle + 360) % 360
            if abs(angle_difference) > angle_tolerance:
                # Adjust the angle to the target angle
                stdscr.addstr(5, 0, f"Rotating to {target_angle:.2f}°")
                motor_controller.rotate_to_angle(gyro, target_angle=target_angle)
                stdscr.refresh()
                time.sleep(0.5)  # Allow some time for the rotation to complete

            # Move forward to the target position
            stdscr.addstr(6, 0, f"Moving forward {target_distance:.2f} cm")
            motor_controller.forward_with_encoders(target_distance * 0.01)

            # Update the current position
            current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
            stdscr.addstr(8, 0, f"Updated grid position: {current_position}")
            stdscr.refresh()

            # Ensure that the robot reaches the exact target position
            final_distance = np.linalg.norm(np.array(target_position) - np.array(current_position))
            if final_distance > position_tolerance:
                # Compensate for the final distance
                stdscr.addstr(9, 0, f"Compensating for final distance: {final_distance:.2f} cm")
                motor_controller.forward_with_encoders(final_distance * 0.01)
                stdscr.refresh()
                time.sleep(0.5)  # Allow some time for the compensation movement

            stdscr.refresh()



