import numpy as np
import heapq
import matplotlib.pyplot as plt
import time
from typing import List, Tuple
import logging

# Konfiguracja logowania
logging.basicConfig(filename='pathfinding.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')

def interpolate_path(path: List[Tuple[float, float]], max_step_size: float = 10.0) -> List[Tuple[int, int]]:
    logging.debug(f"Interpolating path with max_step_size: {max_step_size}")
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

    logging.debug(f"Interpolated path: {interpolated_path}")
    return interpolated_path

class AStarPathfinder:
    def __init__(self, map_grid, resolution=1.0, safety_margin=12):
        self.map_grid = map_grid
        self.resolution = resolution
        self.safety_margin = safety_margin
        self.mapper = None
        logging.info("Initialized AStarPathfinder")

    def set_mapper(self, mapper):
        self.mapper = mapper
        logging.info("Mapper set")

    def heuristic(self, a, b):
        """Oblicza odległość Euklidesową pomiędzy dwoma punktami na siatce."""
        return np.linalg.norm(np.array(a) - np.array(b))

    def astar(self, start, goal):
        """Główna funkcja A* z uwzględnieniem odbicia Y, offsetu i marginesu bezpieczeństwa."""
        logging.info(f"Starting A* algorithm from {start} to {goal}")
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)

        TIME_LIMIT = 60
        start_time = time.time()

        open_list = []
        heapq.heappush(open_list, (0, start_grid))

        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_list:
            if time.time() - start_time > TIME_LIMIT:
                logging.warning("Time limit exceeded")
                return []

            current = heapq.heappop(open_list)[1]

            if current == goal_grid:
                path = self.reconstruct_path(came_from, current)
                logging.info(f"Path found: {path}")
                return path

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor) + self.penalty(neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)

                    if neighbor not in dict(open_list):
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        logging.info("No path found")
        return []

    def get_neighbors(self, node):
        """Znajduje sąsiadów danego węzła, uwzględniając ruch po skosie."""
        x, y = node
        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                     (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)]  # Ruchy po skosie
        valid_neighbors = [n for n in neighbors if self.is_valid(n)]
        logging.debug(f"Neighbors of {node}: {valid_neighbors}")
        return valid_neighbors

    def is_valid(self, node):
        """Sprawdza, czy dany węzeł jest w granicach mapy i nie jest przeszkodą."""
        x, y = node
        is_valid = (0 <= x < self.map_grid.shape[1] and
                    0 <= y < self.map_grid.shape[0] and
                    self.map_grid[y, x] == 0)
        if not is_valid:
            logging.debug(f"Node {node} is invalid")
        return is_valid

    def distance(self, a, b):
        """Oblicza odległość Euklidesową pomiędzy dwoma punktami."""
        return np.linalg.norm(np.array(a) - np.array(b))

    def penalty(self, node):
        """Oblicza karność dla danego węzła w pobliżu przeszkód."""
        x, y = node
        penalty = 0
        for dx in range(-self.safety_margin, self.safety_margin + 1):
            for dy in range(-self.safety_margin, self.safety_margin + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map_grid.shape[1] and 0 <= ny < self.map_grid.shape[0]:
                    if self.map_grid[ny, nx] == 1:
                        penalty += 10  # Wartość karności można dostosować
        logging.debug(f"Penalty for node {node}: {penalty}")
        return penalty

    def reconstruct_path(self, came_from, current):
        """Odtwarza ścieżkę na podstawie odwiedzonych węzłów."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return [self.grid_to_world(p) for p in path[::-1]]

    def world_to_grid(self, world_coords):
        """Konwertuje współrzędne świata na współrzędne siatki z uwzględnieniem odbicia Y i offsetu."""
        x, y = world_coords
        x_grid = int(np.round(x))  # Offset X
        y_grid = int(np.round(200 - y))  # Offset Y i odbicie (200 = 2 * 100)
        return (x_grid, y_grid)

    def grid_to_world(self, point):
        """Przekształca współrzędne siatki z powrotem na współrzędne świata."""
        grid_x, grid_y = point
        world_y = 200 - grid_y
        world_x = grid_x
        return world_x, world_y

    def visualize_path(self, path, map_grid, robot_position=(100, 100), filename="path_visualization.png"):
        plt.figure(figsize=(8, 8))
        plt.imshow(map_grid, cmap='gray', origin='upper')

        if path:
            path = np.array(path)
            path[:, 1] = 2 * 100 - path[:, 1]  # Odbicie Y dla wizualizacji
            plt.plot(path[:, 0], path[:, 1], 'y-', lw=2, label='Original Path')

        simplified_path = rdp(path, epsilon=5.0)
        if simplified_path:
            simplified_path = np.array(simplified_path)
            simplified_path[:, 1] = 2 * 100 - simplified_path[:, 1]
            plt.plot(simplified_path[:, 0], simplified_path[:, 1], 'b--', lw=2, label='Simplified Path (RDP)')

        interpolated_path = interpolate_path(path, max_step_size=10.0)
        if interpolated_path:
            interpolated_path = np.array(interpolated_path)
            interpolated_path[:, 1] = 2 * 100 - interpolated_path[:, 1]
            plt.plot(interpolated_path[:, 0], interpolated_path[:, 1], 'g:', lw=2, label='Interpolated Path')

        for i in range(len(path) - 1):
            current_position = path[i]
            next_position = path[i + 1]
            angle, distance = self.calculate_angle_and_distance(current_position, next_position)

            plt.text((current_position[0] + next_position[0]) / 2,
                     (current_position[1] + next_position[1]) / 2,
                     f'{distance:.1f}cm, {angle:.1f}°',
                     fontsize=8, color='red')

        robot_x, robot_y = robot_position
        robot_y = 2 * 100 - robot_y
        plt.plot(robot_x, robot_y, marker="s", color="r", markersize=25, label='Current Position')

        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('Path Visualization with RDP and Interpolation')
        plt.legend()
        plt.grid(True)
        plt.savefig(filename)
        plt.close()

    def calculate_angle_and_distance(self, current_position, target_position):
        if current_position is None or target_position is None:
            logging.error("Current or target position is None")
            return None, None

        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = np.sqrt(dx ** 2 + dy ** 2)
        angle = np.degrees(np.arctan2(dy, dx))
        angle = (angle + 360) % 360

        logging.debug(f"Angle: {angle:.2f}°, Distance: {distance:.2f}cm")
        return angle, distance

    def move_robot_along_path(self, stdscr, motor_controller, path, gyro, resolution=1.0, angle_tolerance=5,
                              final_position_tolerance=1):
        stdscr.clear()
        stdscr.addstr(0, 0, "Pathfinding...")
        current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
        stdscr.addstr(2, 0, f'Starting at grid position: {current_position}')

        for i, target_position in enumerate(path):
            target_position = tuple(map(int, target_position))
            target_angle, target_distance = self.calculate_angle_and_distance(current_position, target_position)
            stdscr.addstr(3, 0, f'Previous grid position: {current_position}')
            stdscr.addstr(4, 0, f'Target grid position: {target_position}')

            current_angle = gyro.get_angle_z()
            angle_difference = (target_angle - current_angle + 360) % 360
            if abs(angle_difference) > angle_tolerance:
                stdscr.addstr(5, 0, f"Rotating to {target_angle:.2f}°")
                stdscr.refresh()
                motor_controller.rotate_to_angle(gyro, target_angle=target_angle)
                time.sleep(0.2)

            stdscr.addstr(6, 0, f"Moving forward {target_distance:.2f} cm")
            motor_controller.forward_with_encoders(target_distance * 0.01)

            current_position = self.mapper.get_robot_grid_position(self.map_grid, resolution)
            stdscr.addstr(8, 0, f"Updated grid position: {current_position}")
            stdscr.refresh()

            if i == len(path) - 1:
                final_distance = np.linalg.norm(np.array(target_position) - np.array(current_position))
                if final_distance > final_position_tolerance:
                    stdscr.addstr(9, 0, f"Compensating for final distance: {final_distance:.2f} cm")
                    motor_controller.forward_with_encoders(final_distance * 0.01)
                    stdscr.refresh()
                    time.sleep(0.2)
