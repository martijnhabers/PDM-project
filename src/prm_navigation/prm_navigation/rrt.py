import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import math
import os

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 +
                     (node1.y - node2.y) ** 2 +
                     (node1.z - node2.z) ** 2)

class RRT3D:
    def __init__(self, start, goal, grid, step_size=5, max_iter=1000):
        self.start = Node(start[0], start[1], start[2])
        self.goal = Node(goal[0], goal[1], goal[2])
        self.grid = grid
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]

    def get_random_point(self):
        depth, rows, cols = self.grid.shape
        return Node(random.randint(0, cols - 1),
                    random.randint(0, rows - 1),
                    random.randint(0, depth - 1))

    def is_in_collision(self, node):
        depth, rows, cols = self.grid.shape
        if 0 <= node.z < depth and 0 <= node.y < rows and 0 <= node.x < cols:
            return self.grid[node.z][node.y][node.x] == 1
        return True

    def nearest_node(self, random_node):
        return min(self.nodes, key=lambda node: distance(node, random_node))

    def steer(self, from_node, to_node):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        phi = math.atan2(to_node.z - from_node.z,
                         math.sqrt((to_node.x - from_node.x) ** 2 +
                                   (to_node.y - from_node.y) ** 2))
        new_x = int(from_node.x + self.step_size * math.cos(phi) * math.cos(theta))
        new_y = int(from_node.y + self.step_size * math.cos(phi) * math.sin(theta))
        new_z = int(from_node.z + self.step_size * math.sin(phi))
        new_node = Node(new_x, new_y, new_z)
        new_node.parent = from_node
        return new_node

    def path_found(self, node):
        return distance(node, self.goal) < self.step_size

    def get_path(self):
        path = []
        node = self.goal
        while node.parent is not None:
            path.append({'x': node.x, 'y': node.y, 'z': node.z})
            node = node.parent
        path.append({'x': self.start.x, 'y': self.start.y, 'z': self.start.z})
        return path[::-1]

    def plan(self):
        for _ in range(self.max_iter):
            random_node = self.get_random_point()
            nearest_node = self.nearest_node(random_node)
            new_node = self.steer(nearest_node, random_node)

            if not self.is_in_collision(new_node):
                self.nodes.append(new_node)

                if self.path_found(new_node):
                    self.goal.parent = new_node
                    return self.get_path()

        return None  # Path not found

# Example usage
if __name__ == "__main__":
    # Create a 3D grid (1: obstacle, 0: free space)
    # import grid from csv file
    #script_dir = os.path.dirname(__file__)  # Get the directory of the script
    #file_path = os.path.join(script_dir, 'occupancy_grid.csv')
    #grid = np.loadtxt(file_path, delimiter=',')
    
    grid = np.zeros((50, 50, 50))
    grid[20:30, 20:30, 20:30] = 1  # Add a cubic obstacle
    print(grid.shape)

    start = (5, 5, 5)
    goal = (45, 45, 45)

    rrt = RRT3D(start, goal, grid)
    path = rrt.plan()

    # Visualization
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the occupancy grid (obstacles)
    obstacle_coords = np.array(np.where(grid == 1)).T
    ax.scatter(obstacle_coords[:, 2], obstacle_coords[:, 1], obstacle_coords[:, 0], c='black', s=1, label="Obstacles")

    if path:
        path = np.array(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], '-r', label="Path")
        ax.scatter(start[0], start[1], start[2], c='green', s=100, label="Start")
        ax.scatter(goal[0], goal[1], goal[2], c='blue', s=100, label="Goal")
    else:
        print("Path not found")

    ax.legend()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()
