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

    def calculate_angle_and_distance(self, current_position, target_position):
        """
        Calculate the angle and distance from the current position to the target position.
        :param current_position: Tuple (x, y) representing the current position.
        :param target_position: Tuple (x, y) representing the target position.
        :return: Tuple (angle, distance) where angle is the direction to the target and distance is the straight-line distance.
        """
        logging.debug(f'Current position: {current_position}')
        logging.debug(f'Target position: {target_position}')

        if current_position is None or target_position is None:
            logging.error('Current position or target position is None.')
            return None, None

        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = np.sqrt(dx ** 2 + dy ** 2)
        angle = np.degrees(np.arctan2(dy, dx))
        return angle, distance

    def move_robot_along_path(self, motor_controller, path, gyro, map_grid, resolution=1.0, segment_length_cm=10):
        """
        Move the robot along the path generated by the A* algorithm with consideration of map units.
        :param motor_controller: MotorController object to control robot movement.
        :param path: List of points [(x1, y1), (x2, y2), ..., (xN, yN)] representing the path.
        :param gyro: Gyroscope object to track current orientation.
        :param map_grid: Siatka mapy, na której pozycja robota jest przeliczana na kratki.
        :param resolution: Rozdzielczość siatki (np. 1.0 odpowiada 10 cm na kratkę).
        :param segment_length_cm: Length of each segment in centimeters.
        """
        current_position = self.get_robot_grid_position(map_grid, resolution)  # Get the grid position of the robot
        logging.debug(f'Starting at grid position: {current_position}')
        print(f"Robot starting at grid position: {current_position}")

        # Convert segment_length_cm to units of the grid (kratki)
        segment_length_grid_units = segment_length_cm / 10  # 1 kratka = 10 cm, więc segment_length_cm to liczba kratek
        logging.debug(f'Segment length (grid units): {segment_length_grid_units}')

        for target_position in path:  # Move towards each point in the path
            # Calculate angle and distance to the next point
            target_angle, target_distance = self.calculate_angle_and_distance(current_position, target_position)
            logging.debug(f'Target grid position: {target_position}')
            logging.debug(f'Calculated angle: {target_angle}, distance: {target_distance}')

            # Przelicz dystans z metrów na jednostki siatki
            target_distance_grid_units = target_distance / 10  # Dystans w siatce (kratki)
            print(
                f"Next target grid position: {target_position}, Angle: {target_angle:.2f}°, Distance in grid units: {target_distance_grid_units:.2f}")

            # Rotate the robot to face the target angle
            print(f"Rotating to {target_angle:.2f}°")
            motor_controller.rotate_to_angle(gyro, target_angle=target_angle)

            # Move forward the calculated distance in segments, ale w jednostkach siatki
            while target_distance_grid_units > 0:
                segment_distance_grid_units = min(segment_length_grid_units, target_distance_grid_units)
                logging.debug(f'Moving forward segment distance (grid units): {segment_distance_grid_units}')
                print(f"Moving forward: {segment_distance_grid_units:.2f} grid units")
                motor_controller.forward_with_encoders(
                    segment_distance_grid_units * 0.1)  # Przesuń o segment_length_cm na mapie
                target_distance_grid_units -= segment_distance_grid_units
                logging.debug(f'Remaining distance to move (grid units): {target_distance_grid_units}')
                print(f"Remaining distance: {target_distance_grid_units:.2f} grid units")

            # Update current grid position
            current_position = self.get_robot_grid_position(map_grid, resolution)
            logging.debug(f'Updated grid position: {current_position}')
            print(f"Updated grid position: {current_position}")

            # Small delay to simulate real robot movement
            time.sleep(0.5)



