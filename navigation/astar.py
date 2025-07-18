import heapq

class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    def get_neighbors(self, node):
        directions = [(0,1),(1,0),(0,-1),(-1,0)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.rows and 0 <= ny < self.cols and self.grid[nx][ny] == 0:
                neighbors.append((nx, ny))
        return neighbors

    def plan(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0 + self.heuristic(start, goal), 0, start, [start]))

        visited = set()
        while open_list:
            _, cost, current, path = heapq.heappop(open_list)
            if current in visited:
                continue
            if current == goal:
                return path
            visited.add(current)
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    new_cost = cost + 1
                    heapq.heappush(open_list, (
                        new_cost + self.heuristic(neighbor, goal),
                        new_cost,
                        neighbor,
                        path + [neighbor]
                    ))
        return []