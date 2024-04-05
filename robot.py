import numpy as np
from queue import PriorityQueue
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