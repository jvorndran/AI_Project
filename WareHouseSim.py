from robot import Robot
import numpy as np
import pygame


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
        self.assign_tasks(num_robots)

    def generate_obstacles(self, num_obstacles):
        # Generates obstacle positions
        return [((np.random.randint(0, self.grid_size - 3), np.random.randint(0, self.grid_size - 3)),
                 np.random.randint(1, 4), np.random.randint(1, 4)) for _ in range(num_obstacles)]

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