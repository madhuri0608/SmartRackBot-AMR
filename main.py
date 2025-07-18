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

from sensors.lidar_sim import LidarSim

# Simulate Lidar at robot start position
lidar = LidarSim(grid, max_range=6)
distances = lidar.scan(start)

directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

print("\nLIDAR SCAN from Start Position ({}):".format(start))
for d, dist in zip(directions, distances):
    print(f"{d}: {dist} units")

from control.pid import PIDController

# Simulate speed control from 0 to 1 m/s using PID
pid = PIDController(kp=2.0, ki=0.5, kd=0.1, dt=0.1)

target_velocity = 1.0  # m/s
actual_velocity = 0.0
velocity_log = []

print("\n[PID SPEED CONTROL SIMULATION]")
for step in range(30):
    control_signal = pid.compute(target_velocity, actual_velocity)
    
    # Simulate basic motor response: apply control as acceleration
    actual_velocity += control_signal * 0.1  # adjust gain
    velocity_log.append(actual_velocity)
    print(f"Step {step:02d}: Velocity = {actual_velocity:.3f} m/s")

