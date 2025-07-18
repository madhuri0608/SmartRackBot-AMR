import math

class LidarSim:
    def __init__(self, grid, max_range=10):
        self.grid = grid
        self.max_range = max_range
        self.rows = len(grid)
        self.cols = len(grid[0])

        # 8 directions (N, NE, E, SE, S, SW, W, NW)
        self.directions = [
            (-1, 0), (-1, 1), (0, 1), (1, 1),
            (1, 0), (1, -1), (0, -1), (-1, -1)
        ]

    def is_in_bounds(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols

    def cast_ray(self, start_x, start_y, dx, dy):
        x, y = start_x, start_y
        for step in range(1, self.max_range + 1):
            x += dx
            y += dy
            if not self.is_in_bounds(x, y):
                return step  # Hit wall
            if self.grid[x][y] == 1:
                return step  # Hit obstacle
        return self.max_range

    def scan(self, robot_pos):
        distances = []
        for dx, dy in self.directions:
            dist = self.cast_ray(robot_pos[0], robot_pos[1], dx, dy)
            distances.append(dist)
        return distances
