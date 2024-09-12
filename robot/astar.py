import numpy as np
import matplotlib.pyplot as plt
import time
import heapq
import logging

# Konfiguracja logowania
logging.basicConfig(filename='pathfinding.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')

def rdp(points, epsilon):
    """
    Ramer-Douglas-Peucker algorithm for path simplification.
    :param points: List of points representing the path.
    :param epsilon: The simplification factor. Smaller values result in less simplification.
    :return: Simplified path.
    """
    if len(points) < 2:
        return points

    def distance(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def rdp_recursive(points, epsilon):
        if len(points) < 2:
            return points

        start, end = points[0], points[-1]
        dmax = 0
        index = 0

        for i in range(1, len(points) - 1):
            d = distance(points[i], start) + distance(points[i], end) - distance(start, end)
            if d > dmax:
                dmax = d
                index = i

        if dmax > epsilon:
            rec_results1 = rdp_recursive(points[:index + 1], epsilon)
            rec_results2 = rdp_recursive(points[index:], epsilon)
            return rec_results1[:-1] + rec_results2
        else:
            return [start, end]

    return rdp_recursive(points, epsilon)

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
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, start, goal):
        """A* algorithm implementation."""
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
                    if neighbor not in [n for _, n in open_list]:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return []

    def get_neighbors(self, node):
        """Returns neighbors of the node."""
        x, y = node
        neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [n for n in neighbors if self.is_valid(n)]

    def is_valid(self, node):
        """Checks if a node is within the grid and not an obstacle."""
        x, y = node
        return 0 <= x < self.map_grid.shape[1] and 0 <= y < self.map_grid.shape[0] and self.map_grid[y, x] == 0

    def distance(self, a, b):
        """Calculate Euclidean distance."""
        return np.linalg.norm(np.array(a) - np.array(b))

    def penalty(self, node):
        """Calculates penalty for the given node near obstacles."""
        x, y = node
        penalty = 0
        for dx in range(-self.safety_margin, self.safety_margin + 1):
            for dy in range(-self.safety_margin, self.safety_margin + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.map_grid.shape[1] and 0 <= ny < self.map_grid.shape[0]:
                    if self.map_grid[ny, nx] == 1:
                        penalty += 10  # Adjust penalty value as needed
        logging.debug(f"Penalty for node {node}: {penalty}")
        return penalty

    def reconstruct_path(self, came_from, current):
        """Reconstructs the path from the end to the start."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def world_to_grid(self, world_coords):
        """Converts world coordinates to grid coordinates considering Y-axis reflection and offset."""
        x, y = world_coords
        x_grid = int(np.round(x))  # X-offset
        y_grid = int(np.round(200 - y))  # Y-offset and reflection (200 = 2 * 100)
        return (x_grid, y_grid)

    def grid_to_world(self, point):
        """Converts grid coordinates back to world coordinates."""
        grid_x, grid_y = point
        world_y = 200 - grid_y
        world_x = grid_x
        return world_x, world_y


    def interpolate_path(self, path, max_step_size=10.0):
        """Interpoluje ścieżkę, aby zmniejszyć liczbę punktów i uzyskać płynniejsze przejście."""
        if len(path) < 2:
            return path

        interpolated_path = [path[0]]

        for i in range(1, len(path)):
            start = np.array(path[i - 1])
            end = np.array(path[i])
            distance = np.linalg.norm(end - start)

            if distance > max_step_size:
                num_steps = int(np.ceil(distance / max_step_size))
                for j in range(1, num_steps):
                    step = start + (end - start) * (j / num_steps)
                    interpolated_path.append(tuple(np.round(step).astype(int)))

            interpolated_path.append(path[i])

        return interpolated_path

    def visualize_path(self, path, map_grid, robot_position=(100, 100), filename="path_visualization.png"):
        plt.figure(figsize=(8, 8))
        plt.imshow(map_grid, cmap='gray', origin='upper')

        if path:
            path = np.array(path)
            path[:, 1] = 2 * 100 - path[:, 1]  # Odbicie Y dla wizualizacji
            plt.plot(path[:, 0], path[:, 1], 'y-', lw=2, label='Original Path')

        simplified_path = rdp(path, epsilon=1.0)
        if simplified_path:
            simplified_path = np.array(simplified_path)
            plt.plot(simplified_path[:, 0], simplified_path[:, 1], 'b--', lw=2, label='Simplified Path (RDP)')

        interpolated_path = self.interpolate_path(simplified_path, max_step_size=10.0)
        if interpolated_path:
            interpolated_path = np.array(interpolated_path)
            plt.plot(interpolated_path[:, 0], interpolated_path[:, 1], 'g:', lw=2, label='Interpolated Path')

        for i in range(len(interpolated_path) - 1):
            current_position = interpolated_path[i]
            next_position = interpolated_path[i + 1]
            angle, distance = self.calculate_angle_and_distance(current_position, next_position)

            plt.plot(current_position[0], current_position[1], 'o', color='magenta', markersize=6)
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
        path = rdp(path, epsilon=1.0)
        path = self.interpolate_path(path, max_step_size=10.0)
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
