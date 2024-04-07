import pygame
import numpy as np
from queue import PriorityQueue
import pygame.gfxdraw
import random

class Robot:
    def __init__(self, start_pos):
        self.current_pos = np.array(start_pos)
        self.tasks = []  # A list of tasks (tuples of numpy arrays for pickup and delivery)
        self.on_task = False
        self.current_task_index = 0

    def add_task(self, task):
        self.tasks.append(task)

    def move(self, obstacles, grid_size):
        if self.tasks:
            current_task = self.tasks[self.current_task_index]
            target = current_task[1] if self.on_task else current_task[0]
            path = self.calculate_path(target, obstacles, grid_size)
            if path:
                next_pos = path[0]
                self.current_pos = np.array(next_pos)
                if np.array_equal(self.current_pos, current_task[0]):
                    self.on_task = True
                elif np.array_equal(self.current_pos, current_task[1]):
                    if self.current_task_index < len(self.tasks) - 1:
                        self.current_task_index += 1
                        self.on_task = False
                    else:
                        self.tasks.pop(self.current_task_index)  # Task completed
                        if self.tasks:
                            self.current_task_index = 0  # Move to next task
                        else:
                            self.on_task = False  # No more tasks


    def destroy_and_repair_tasks(self, obstacles, grid_size, destroy_ratio=0.3):
        """
        Performs one iteration of destroy and repair on the robot's tasks.
        """
        # Destroy Phase: Randomly remove a percentage of tasks
        num_destroy = int(len(self.tasks) * destroy_ratio)
        destroyed_tasks = random.sample(self.tasks, num_destroy)
        for task in destroyed_tasks:
            self.tasks.remove(task)

        # Repair Phase: Reassign tasks based on marginal cost
        for task in destroyed_tasks:
            self.assign_marginal_cost_task([task], obstacles, grid_size)

    def run_lns(self, obstacles, grid_size, iterations=10):
        """
        Runs the LNS algorithm for a specified number of iterations.
        """
        for _ in range(iterations):
            self.destroy_and_repair_tasks(obstacles, grid_size)

    def calculate_task_cost(self, task, obstacles, grid_size):
        """
        Calculates the marginal cost of adding a new task based on the additional
        distance the robot will need to travel to complete the task.
        """
        # Cost from current position to task's pickup location
        pickup_cost = self.calculate_path_cost(self.current_pos, task[0], obstacles, grid_size)

        # Cost from task's pickup to delivery location
        delivery_cost = self.calculate_path_cost(task[0], task[1], obstacles, grid_size)

        if pickup_cost is None or delivery_cost is None:
            return float('inf')  # Task is unreachable
        return pickup_cost + delivery_cost

    def calculate_total_distance(self, obstacles, grid_size):
        """
        Calculates the total distance the robot would travel to complete all its tasks,
        moving from one task's delivery point to the next task's pickup point.
        """
        total_distance = 0
        current_position = self.current_pos

        for task in self.tasks:
            # Calculate distance from the current position to the task's pickup location
            pickup_cost = self.calculate_path_cost(current_position, task[0], obstacles, grid_size)
            if pickup_cost is None:
                continue  # Skip this task if the path to the pickup is blocked

            # Calculate distance from the task's pickup location to its delivery location
            delivery_cost = self.calculate_path_cost(task[0], task[1], obstacles, grid_size)
            if delivery_cost is None:
                continue  # Skip this task if the path from the pickup to the delivery is blocked

            total_distance += pickup_cost + delivery_cost
            current_position = task[1]  # Update current position to the delivery location of the current task

        return total_distance

    def calculate_path_cost(self, start, target, obstacles, grid_size):
        """
        Reuses the calculate_path method to find a path and then calculates its cost.
        Instead of returning the path, it returns the cost of the path.
        """
        path = self.calculate_path(target, obstacles, grid_size)
        if path:
            return len(path)  # The cost is the length of the path
        return None  # Indicate that the target is not reachable

    def assign_marginal_cost_task(self, available_tasks, obstacles, grid_size):
        """
        Assigns the task with the lowest marginal cost to the robot.
        """
        lowest_cost = float('inf')
        best_task = None
        for task in available_tasks:
            cost = self.calculate_task_cost(task, obstacles, grid_size)
            if cost < lowest_cost:
                lowest_cost = cost
                best_task = task

        if best_task:
            self.add_task(best_task)
            return best_task
        return None  # No task was assigned

    def is_path_blocked(self, next_pos, obstacles):
        for obs_start, obs_width, obs_height in obstacles:
            obs_cells = [(obs_start[0] + x, obs_start[1] + y) for x in range(obs_width) for y in range(obs_height)]
            if tuple(next_pos) in obs_cells:
                return True
        return False

    def calculate_path(self, target, obstacles, grid_size):
        def heuristic(a, b):
            # Manhattan distance on a square grid
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def get_neighbors(pos):
            neighbors = []
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-directional movement
                next_pos = (pos[0] + dx, pos[1] + dy)
                if 0 <= next_pos[0] < grid_size and 0 <= next_pos[1] < grid_size:
                    neighbors.append(next_pos)
            return neighbors

        def reconstruct_path(came_from, current):
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Return reversed path

        start = tuple(self.current_pos)
        target = tuple(target)

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}

        g_score = {start: 0}
        f_score = {start: heuristic(start, target)}

        while not open_set.empty():
            _, current = open_set.get()

            if current == target:
                return reconstruct_path(came_from, current)

            for neighbor in get_neighbors(current):
                if self.is_path_blocked(np.array(neighbor), obstacles):
                    continue

                tentative_g_score = g_score[current] + 1  # Assume cost between neighbors is 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, target)
                    if not any(neighbor == n[1] for n in open_set.queue):
                        open_set.put((f_score[neighbor], neighbor))

        return []  # Return an empty path if there is no path to target

class WarehouseSimulation:
    def __init__(self, grid_size=50, cell_size=15, num_robots=6, num_obstacles=30):
        pygame.init()
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.screen_size = grid_size * cell_size
        self.screen = pygame.display.set_mode((self.screen_size, self.screen_size))
        pygame.display.set_caption('Warehouse Simulation')
        self.obstacles = self.generate_obstacles(num_obstacles)
        self.robots = [Robot((np.random.randint(0, grid_size), np.random.randint(0, grid_size))) for _ in range(num_robots)]
        self.assign_tasks_globally(num_robots)

    def generate_obstacles(self, num_obstacles):
        # Generates obstacle positions
        return [((np.random.randint(0, self.grid_size - 3), np.random.randint(0, self.grid_size - 3)),
                 np.random.randint(1, 4), np.random.randint(1, 4)) for _ in range(num_obstacles)]

    def assign_tasks_globally(self, num_tasks):
        tasks = []
        for _ in range(num_tasks):
            pickup = self.find_valid_position()
            delivery = self.find_valid_position()
            color = self.generate_unique_color([])
            tasks.append((pickup, delivery, color))

        # Pre-calculate the total cost for the system before assignment
        initial_total_cost = sum(robot.calculate_total_distance(self.obstacles, self.grid_size) for robot in self.robots)

        # Iterate over tasks to assign them to the robots
        for task in tasks:
            assignment_options = []
            for robot in self.robots:
                robot.tasks.append(task)  # Temporarily assign task to robot
                new_total_cost = sum(r.calculate_total_distance(self.obstacles, self.grid_size) for r in self.robots)
                cost_increase = new_total_cost - initial_total_cost
                assignment_options.append((cost_increase, robot, task))
                robot.tasks.pop()  # Remove temporary task assignment


            # Assign the task to the robot that results in the smallest cost increase
            best_assignment = min(assignment_options, key=lambda x: x[0])
            best_assignment[1].tasks.append(best_assignment[2])  # Add task to the best robot's task list

            # Update the initial total cost to reflect the new assignment
            initial_total_cost += best_assignment[0]




    def find_valid_position(self):
        while True:
            position = (np.random.randint(0, self.grid_size), np.random.randint(0, self.grid_size))
            if not self.is_position_in_obstacle(position):
                return position

    def assign_tasks(self, num_tasks):
        used_colors = []  # Keep track of used colors for tasks
        for _ in range(num_tasks):
            while True:
                pickup = (np.random.randint(0, self.grid_size), np.random.randint(0, self.grid_size))
                delivery = (np.random.randint(0, self.grid_size), np.random.randint(0, self.grid_size))
                if not self.is_position_in_obstacle(pickup) and not self.is_position_in_obstacle(delivery):
                    break  # Found valid positions, break out of the loop

            task_color = self.generate_unique_color(used_colors)  # Assign a unique color
            min_cost = float('inf')
            best_robot = None
            for robot in self.robots:
                cost = self.calculate_marginal_cost(robot, (pickup, delivery))
                if cost < min_cost:
                    min_cost = cost
                    best_robot = robot
            if best_robot is not None:
                best_robot.add_task([np.array(pickup), np.array(delivery), task_color])
            print(f'Assigned task to robot at position {best_robot.current_pos} with cost {min_cost}')

    def calculate_marginal_cost(self, robot, task):
        # Simplified marginal cost calculation: distance from robot's last task (or current position if no tasks) to new task's pickup point
        if robot.tasks:
            last_task = robot.tasks[-1][1]  # Last task's delivery point
            cost = np.linalg.norm(last_task - np.array(task[0]))
        else:
            cost = np.linalg.norm(robot.current_pos - np.array(task[0]))

        return cost

    def run(self):
        clock = pygame.time.Clock()
        running = True
        optimize_interval = 100
        tick_count = 0

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.screen.fill((255, 255, 255))  # Fill screen with white
            self.draw_grid()
            self.draw_obstacles()
            self.place_tasks()

            # Call the move method for each robot
            for robot in self.robots:
                robot.move(self.obstacles, self.grid_size)

            self.animate_robots()
            pygame.display.flip()  # Update the full display Surface to the screen
            clock.tick(10)  # Control the framerate to slow down the movements

            if tick_count % optimize_interval == 0:
                for robot in self.robots:
                    robot.run_lns(self.obstacles, self.grid_size, iterations=10)

            tick_count += 1


    def draw_grid(self):
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                rect = pygame.Rect(x * self.cell_size, y * self.cell_size, self.cell_size, self.cell_size)
                pygame.draw.rect(self.screen, (200, 200, 200), rect, 1)

    def draw_obstacles(self):
        for start_pos, width, height in self.obstacles:
            for x in range(width):
                for y in range(height):
                    obstacle_rect = ((start_pos[0] + x) * self.cell_size, (start_pos[1] + y) * self.cell_size, self.cell_size, self.cell_size)
                    pygame.draw.rect(self.screen, (128, 0, 128), obstacle_rect)

    def place_tasks(self):
        for robot in self.robots:
            for pickup, delivery, color in robot.tasks:
                # Draw pickup
                pickup_rect = (pickup[0] * self.cell_size, pickup[1] * self.cell_size, self.cell_size, self.cell_size)
                pygame.draw.rect(self.screen, color, pickup_rect)
                pygame.draw.circle(self.screen, (0, 0, 0), (
                pickup[0] * self.cell_size + self.cell_size // 2, pickup[1] * self.cell_size + self.cell_size // 2),
                                   self.cell_size // 4)

                # Draw delivery
                delivery_rect = (
                delivery[0] * self.cell_size, delivery[1] * self.cell_size, self.cell_size, self.cell_size)
                pygame.draw.rect(self.screen, color, delivery_rect)
                pygame.draw.line(self.screen, (0, 0, 0), (
                delivery[0] * self.cell_size + self.cell_size // 4, delivery[1] * self.cell_size + self.cell_size // 4),
                                 (delivery[0] * self.cell_size + 3 * self.cell_size // 4,
                                  delivery[1] * self.cell_size + 3 * self.cell_size // 4), 3)
                pygame.draw.line(self.screen, (0, 0, 0), (delivery[0] * self.cell_size + 3 * self.cell_size // 4,
                                                          delivery[1] * self.cell_size + self.cell_size // 4), (
                                 delivery[0] * self.cell_size + self.cell_size // 4,
                                 delivery[1] * self.cell_size + 3 * self.cell_size // 4), 3)

    def generate_unique_color(self, used_colors):
        import random
        while True:
            color = (random.randint(64, 255), random.randint(64, 255), random.randint(64, 255))
            if color not in used_colors:
                used_colors.append(color)
                return color

    def is_position_in_obstacle(self, pos):
        for obs_start, obs_width, obs_height in self.obstacles:
            if obs_start[0] <= pos[0] < obs_start[0] + obs_width and obs_start[1] <= pos[1] < obs_start[1] + obs_height:
                return True
        return False

    def animate_robots(self):
        for robot in self.robots:
            # Draw the robot at its current position
            robot_rect = (robot.current_pos[0] * self.cell_size, robot.current_pos[1] * self.cell_size, self.cell_size,
                          self.cell_size)
            pygame.draw.rect(self.screen, (0, 0, 255), robot_rect)  # Draw robot in blue

            # Optionally, draw a line representing the path for the robot's current task to the pickup/delivery points
            if robot.tasks:
                current_task = robot.tasks[robot.current_task_index]
                pickup_pos, delivery_pos, color = current_task
                if not robot.on_task:
                    # Draw a line from the robot to its pickup point if it's not on task
                    pygame.draw.line(self.screen, (255, 165, 0),
                                     (robot.current_pos[0] * self.cell_size + self.cell_size // 2,
                                      robot.current_pos[1] * self.cell_size + self.cell_size // 2),
                                     (pickup_pos[0] * self.cell_size + self.cell_size // 2,
                                      pickup_pos[1] * self.cell_size + self.cell_size // 2), 2)
                else:
                    # Draw a line from the robot to its delivery point if it's on task
                    pygame.draw.line(self.screen, (255, 20, 147),
                                     (robot.current_pos[0] * self.cell_size + self.cell_size // 2,
                                      robot.current_pos[1] * self.cell_size + self.cell_size // 2),
                                     (delivery_pos[0] * self.cell_size + self.cell_size // 2,
                                      delivery_pos[1] * self.cell_size + self.cell_size // 2), 2)


if __name__ == '__main__':
    sim = WarehouseSimulation()

    for robot in sim.robots:
        robot.run_lns(sim.obstacles, sim.grid_size, iterations=10)

    sim.run()