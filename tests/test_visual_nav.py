import matplotlib.pyplot as plt
import matplotlib.patches as patches
from navigation.astar import AStarPlanner
from control.pid import PIDController
import time, math, random

# ---------------- Grid and Environment ---------------- #
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
rack_locations = [(0, 4), (2, 8), (4, 6)]
drop_locations = [(9, 0), (9, 9)]
dynamic_obstacles = [(3, 3)]  # Moving obstacle

# ---------------- Robot & Controller ---------------- #
pid = PIDController(kp=2.0, ki=0.3, kd=0.1, dt=0.1)
robot_pos = list(start)
actual_velocity = 0.0
target_velocity = 1.0
carrying = False

# ---------------- Matplotlib Setup ---------------- #
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

# Start point
ax.plot(start[1], start[0], 'go', label='Start')
robot_dot, = ax.plot(robot_pos[1], robot_pos[0], 'ro', markersize=8, label='Robot')
obstacle_dots, = ax.plot([], [], 'bs', markersize=10, label='Obstacle')
plt.legend()

planner = AStarPlanner(grid)
task_queue = [(rack, drop_locations[i % len(drop_locations)]) for i, rack in enumerate(rack_locations)]

# ------------- LiDAR Detection Logic ------------- #
def is_obstacle_detected(pos, obstacles, detection_range=0.5):
    for obs in obstacles:
        dist = math.hypot(obs[0] - pos[0], obs[1] - pos[1])
        if dist <= detection_range:
            print(f"[⚠️] Obstacle at {obs} detected near {tuple(round(p, 1) for p in pos)} (dist {dist:.2f})")
            return True
    return False

def move_obstacles(obstacles, grid):
    new_obs = []
    for obs in obstacles:
        r = random.choice([(0,1), (0,-1), (1,0), (-1,0), (0,0)])
        new_pos = (obs[0] + r[0], obs[1] + r[1])
        if 0 <= new_pos[0] < len(grid) and 0 <= new_pos[1] < len(grid[0]) and grid[new_pos[0]][new_pos[1]] == 0:
            new_obs.append(new_pos)
        else:
            new_obs.append(obs)
    return new_obs

# ------------- Task Loop (Pick → Drop) ------------- #
for pickup, drop in task_queue:
    for target in [pickup, drop]:
        goal_type = "RACK" if target == pickup else "DROP"
        print(f"[TASK] Moving to {goal_type} at {target}")
        ax.set_title(f"Heading to {goal_type} at {target}")
        path = planner.plan((round(robot_pos[0]), round(robot_pos[1])), target)
        path_index = 0

        while path_index < len(path):
            next_pos = path[path_index]
            dx = next_pos[0] - robot_pos[0]
            dy = next_pos[1] - robot_pos[1]
            dist = math.hypot(dx, dy)
            if dist < 0.1:
                path_index += 1
                continue

            if is_obstacle_detected(robot_pos, dynamic_obstacles):
                print("[⚠️] Obstacle detected! Waiting...")
                ax.set_title("Waiting for obstacle to move...")
                time.sleep(0.5)
                dynamic_obstacles = move_obstacles(dynamic_obstacles, grid)
                continue

            control = pid.compute(target_velocity, actual_velocity)
            actual_velocity += control * 0.1
            dx /= dist
            dy /= dist
            robot_pos[0] += dx * actual_velocity * 0.1
            robot_pos[1] += dy * actual_velocity * 0.1

            # Plot updates
            robot_dot.set_data([robot_pos[1]], [robot_pos[0]])
            obs_x = [o[1] for o in dynamic_obstacles]
            obs_y = [o[0] for o in dynamic_obstacles]
            obstacle_dots.set_data(obs_x, obs_y)

            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.05)

        if goal_type == "RACK":
            print(f"[ACTION] Picked item at {target}")
            carrying = True
        else:
            print(f"[ACTION] Dropped item at {target}")
            carrying = False
        time.sleep(0.5)

plt.ioff()
plt.title("All Tasks Completed!")
plt.show()
