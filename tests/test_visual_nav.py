import matplotlib.pyplot as plt
import matplotlib.patches as patches
from navigation.astar import AStarPlanner
from control.pid import PIDController
import time

# Grid layout
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

# Start point
start = (0, 0)

# Rack locations (Pickups)
rack_locations = [(0, 4), (2, 8), (4, 6)]

# Drop zones (Deliveries)
drop_locations = [(9, 0), (9, 9)]

# PID Controller
pid = PIDController(kp=2.0, ki=0.3, kd=0.1, dt=0.1)

# Init robot state
robot_pos = list(start)
actual_velocity = 0.0
target_velocity = 1.0
carrying = False

# Create the plot
fig, ax = plt.subplots()
plt.ion()
ax.set_xlim(-0.5, len(grid[0]) - 0.5)
ax.set_ylim(-0.5, len(grid) - 0.5)
ax.set_aspect('equal')
ax.invert_yaxis()
ax.set_title("SmartRackBot Task Simulation")

# Draw grid + racks + drop zones
for i in range(len(grid)):
    for j in range(len(grid[0])):
        if (i, j) in rack_locations:
            ax.add_patch(patches.Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='saddlebrown'))
        elif (i, j) in drop_locations:
            ax.add_patch(patches.Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='lightgreen'))
        elif grid[i][j] == 1:
            ax.add_patch(patches.Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='gray'))

# Start point marker
ax.plot(start[1], start[0], 'go', label='Start')
robot_dot, = ax.plot(robot_pos[1], robot_pos[0], 'ro', markersize=8, label='Robot')
plt.legend()

# Instantiate path planner
planner = AStarPlanner(grid)

# Define full task list (pick -> drop for each)
task_queue = []
for i, rack in enumerate(rack_locations):
    drop = drop_locations[i % len(drop_locations)]  # Rotate drop zones
    task_queue.append((rack, drop))

# Task Loop
for pickup, drop in task_queue:
    # ------------------ Go to Pickup -------------------
    path = planner.plan((round(robot_pos[0]), round(robot_pos[1])), pickup)
    print(f"[TASK] Moving to pick from rack at {pickup}")
    ax.set_title(f"Heading to RACK at {pickup}")
    
    path_index = 0
    while path_index < len(path):
        next_pos = path[path_index]
        dx = next_pos[0] - robot_pos[0]
        dy = next_pos[1] - robot_pos[1]
        dist_to_next = (dx**2 + dy**2)**0.5

        if dist_to_next < 0.1:
            path_index += 1
            continue

        control_signal = pid.compute(target_velocity, actual_velocity)
        actual_velocity += control_signal * 0.1

        dx /= dist_to_next
        dy /= dist_to_next
        robot_pos[0] += dx * actual_velocity * 0.1
        robot_pos[1] += dy * actual_velocity * 0.1

        # Update robot dot
        robot_dot.set_data([robot_pos[1]], [robot_pos[0]])
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.05)

    print(f"[ACTION] Picked item at rack {pickup}")
    ax.set_title(f"Picked item from {pickup}")
    carrying = True
    time.sleep(0.5)

    # ------------------ Go to Drop Zone -------------------
    path = planner.plan((round(robot_pos[0]), round(robot_pos[1])), drop)
    print(f"[TASK] Moving to drop zone at {drop}")
    ax.set_title(f"Heading to DROP at {drop}")
    
    path_index = 0
    while path_index < len(path):
        next_pos = path[path_index]
        dx = next_pos[0] - robot_pos[0]
        dy = next_pos[1] - robot_pos[1]
        dist_to_next = (dx**2 + dy**2)**0.5

        if dist_to_next < 0.1:
            path_index += 1
            continue

        control_signal = pid.compute(target_velocity, actual_velocity)
        actual_velocity += control_signal * 0.1

        dx /= dist_to_next
        dy /= dist_to_next
        robot_pos[0] += dx * actual_velocity * 0.1
        robot_pos[1] += dy * actual_velocity * 0.1

        # Update robot dot
        robot_dot.set_data([robot_pos[1]], [robot_pos[0]])
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.05)

    print(f"[ACTION] Dropped item at zone {drop}")
    ax.set_title(f"Dropped item at {drop}")
    carrying = False
    time.sleep(0.5)

plt.ioff()
plt.title("All Tasks Completed!")
plt.show()
