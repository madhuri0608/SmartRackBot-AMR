from navigation.astar import AStarPlanner

def print_grid_path(grid, path):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if (i, j) in path:
                print(" * ", end="")
            elif grid[i][j] == 1:
                print(" X ", end="")
            else:
                print(" . ", end="")
        print()

# Define a simple 10x10 grid with obstacles
grid = [
    [0,0,0,0,1,0,0,0,0,0],
    [0,1,1,0,1,0,1,1,1,0],
    [0,0,0,0,0,0,0,0,1,0],
    [0,1,0,1,1,1,1,0,1,0],
    [0,0,0,0,0,0,1,0,0,0],
    [0,1,1,1,1,0,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,0]
]

start = (0, 0)
goal = (9, 9)

planner = AStarPlanner(grid)
path = planner.plan(start, goal)

if path:
    print("Path Found!")
    print_grid_path(grid, path)
else:
    print("No Path Found.")
