import numpy as np
import matplotlib.pyplot as plt

# Create a grid (0 = empty, 1 = obstacle)
grid = np.zeros((10, 10))
grid[3, 4] = 1  # Example obstacle

plt.imshow(grid, cmap='Greys', origin='upper')
plt.title("Grid-based Navigation Visualization")
plt.show()