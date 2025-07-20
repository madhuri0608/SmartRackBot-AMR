import matplotlib.pyplot as plt
import numpy as np

class Visualizer:
    def __init__(self, grid):
        self.grid = np.array(grid)
        self.fig, self.ax = plt.subplots()
        plt.ion()

    def update(self, robot_pos, path, lidar_distances):
        self.ax.clear()
        self.ax.imshow(self.grid, cmap="Greys", origin="upper")
        self.ax.plot(robot_pos[1], robot_pos[0], "bo", label="Robot")
        if path:
            for (r, c) in path:
                self.ax.plot(c, r, "g.")
        self.ax.set_title(f"Robot Position: {robot_pos}")
        self.ax.legend()
        plt.pause(0.01)
        