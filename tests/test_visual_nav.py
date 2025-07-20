import time
from navigation.astar import AStarPlanner
from sensors.lidar_sim import LidarSim
from utils.visualizer import Visualizer
from perception.yolo_sim import YOLOSim

# Define the 10x10 grid
grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
    [1, 1, 0, 1, 0, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    [1, 1, 1, 0, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1, 1, 1, 1, 0]
]

# Initialize modules
planner = AStarPlanner(grid)
lidar = LidarSim(grid)
visualizer = Visualizer(grid)
yolo = YOLOSim(tag_locations=[(0, 4), (2, 8), (4, 6)], confidence_threshold=0.5)

# Define initial robot position
robot_pos = (9, 0)

# Define FSM task list: (pickup_rack, drop_location)
task_list = [((0, 4), (9, 0)), ((2, 8), (9, 9)), ((4, 6), (9, 0))]

def run_fsm_tasks():
    global robot_pos
    for i, (rack, drop) in enumerate(task_list):
        print(f"\n[🧠 FSM] Task {i+1}: Go to RACK {rack} then DROP {drop}")

        # Move to RACK
        print(f"[TASK] Moving to RACK at {rack}")
        path_to_rack = planner.plan(tuple(robot_pos), rack)
        for step in path_to_rack:
            robot_pos = step
            lidar_readings = lidar.scan(robot_pos)
            visualizer.update(robot_pos, path_to_rack, lidar_readings)
            time.sleep(0.05)

        # Simulate YOLO tag detection
        if yolo.detect_tag(robot_pos):
            print(f"[✅ YOLOv8 Vision] Tag detected at {robot_pos}")
            print("[✅] Picked up item!")
        else:
            print(f"[❌ YOLOv8 Vision] Tag not in sight at {robot_pos} (dist {round(yolo.distance_to_nearest_tag(robot_pos), 2)})")
            print(f"[❌] Failed to detect tag at {robot_pos} — Skipping pickup!")

        # Move to DROP
        print(f"[TASK] Moving to DROP at {drop}")
        path_to_drop = planner.plan(tuple(robot_pos), drop)
        for step in path_to_drop:
            robot_pos = step
            lidar_readings = lidar.scan(robot_pos)
            visualizer.update(robot_pos, path_to_drop, lidar_readings)
            time.sleep(0.05)

        print(f"[✅ ACTION] Dropped item at {drop}")

if __name__ == "__main__":
    run_fsm_tasks()
