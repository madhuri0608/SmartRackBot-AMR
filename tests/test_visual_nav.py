import matplotlib.pyplot as plt
import matplotlib.patches as patches
from navigation.astar import AStarPlanner
from control.pid import PIDController
import time

# Define the same grid
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

# Define start and goal
start = (0, 0)
goal = (9, 9)

# Define rack/shelf positions
rack_locations = [(0, 4), (2, 8), (4, 6), (7, 0), (9, 5)]

planner = AStarPlanner(grid)
path = planner.plan(start, goal)

# PID controller
pid = PIDController(kp=2.0, ki=0.3, kd=0.1, dt=0.1)
target_velocity = 1.0
actual_velocity = 0.0

# Initialize robot position
robot_pos = list(start)
path_index = 0

# Create the plot
fig, ax = plt.subplots()
plt.ion()
ax.set_xlim(-0.5, len(grid[0]) - 0.5)
ax.set_ylim(-0.5, len(grid) - 0.5)
ax.set_aspect('equal')
ax.invert_yaxis()
ax.set_title("AMR Navigation with PID Control")

# Draw grid and racks
for i in range(len(grid)):
    for j in range(len(grid[0])):
        if (i, j) in rack_locations:
            rack = patches.Rectangle((j-0.5, i-0.5), 1, 1, linewidth=1, edgecolor='k', facecolor='saddlebrown')
            ax.add_patch(rack)
        elif grid[i][j] == 1:
            wall = patches.Rectangle((j-0.5, i-0.5), 1, 1, linewidth=1, edgecolor='k', facecolor='gray')
            ax.add_patch(wall)

# Draw path
for node in path:
    ax.plot(node[1], node[0], 'b.', markersize=4)

# Draw start and goal
ax.plot(start[1], start[0], 'go', label='Start')
ax.plot(goal[1], goal[0], 'ro', label='Goal')
robot_dot, = ax.plot(robot_pos[1], robot_pos[0], 'ro', markersize=8, label='Robot')

plt.legend()

# Movement simulation with animation
while path_index < len(path):
    next_pos = path[path_index]

    dx = next_pos[0] - robot_pos[0]
    dy = next_pos[1] - robot_pos[1]
    dist_to_next = (dx**2 + dy**2) ** 0.5

    if dist_to_next < 0.1:
        path_index += 1
        continue

    control_signal = pid.compute(target_velocity, actual_velocity)
    actual_velocity += control_signal * 0.1

    dx /= dist_to_next
    dy /= dist_to_next
    robot_pos[0] += dx * actual_velocity * 0.1
    robot_pos[1] += dy * actual_velocity * 0.1

    # Detect and simulate pickup at shelf
    robot_cell = (round(robot_pos[0]), round(robot_pos[1]))
    if robot_cell in rack_locations:
        ax.set_title(f"Picking item from Rack at {robot_cell}")
        print(f"[INFO] Robot picking item at {robot_cell}")
        rack_locations.remove(robot_cell)

    # Update robot dot
    robot_dot.set_data([robot_pos[1]], [robot_pos[0]])
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.05)

plt.ioff()
plt.show()
