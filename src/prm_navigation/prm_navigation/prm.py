import random
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import os
import sys

sys.path.append(os.path.abspath("src/world_creator/test/"))

# Import the module
from WorldListExport import generate_ocupation_matrix

# set random seeds for reproducibility
random.seed(0)
np.random.seed(0)

class PRM:
    def __init__(self, num_samples, k_neighbors, occupancy_grid):
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.occupancy_grid = occupancy_grid
        # self.graph = {"nodes": [], "edges": {}}  # Graph representation
        self.graph = nx.Graph()

    def sample_free_space(self, bounds, layer):
        # Selects sample within dimension of bounds
        sample = [int(random.uniform(bound[0], bound[1])) for bound in bounds]
        sample.append(layer)
        return sample

    def distance(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def build_roadmap(self, bounds):
        # Step 1: Generate random samples until we have enough
        node_id = 0
        for layer in range(self.occupancy_grid.shape[0]):
            while len(self.graph.nodes) < self.num_samples*(layer+1):
                sample = self.sample_free_space(bounds, layer)
                if self.collision_checker(self.occupancy_grid[layer], sample[:2]):
                    self.graph.add_node(node_id, pos=sample)
                    node_id += 1

        # Step 2: Connect neighbouring nodes and add weights
        for node in self.graph.nodes:
            self.connect_to_nearest_neighbors(node, self.k_neighbors)
    
    def connect_to_nearest_neighbors(self, node, k):
        distances = []
        layer = self.graph.nodes[node]['pos'][2]
        for neighbour in self.graph.nodes:
            if node != neighbour and layer <= self.graph.nodes[neighbour]['pos'][2] <= layer + 1:
                distances.append((self.distance(self.graph.nodes[node]['pos'], self.graph.nodes[neighbour]['pos']), neighbour))
        distances.sort()

        while len(self.graph.edges(node)) < k and distances:
            possible_neighbour = distances.pop(0)
            node_coords = self.graph.nodes[node]['pos']
            possible_neighbour_coords = self.graph.nodes[possible_neighbour[1]]['pos']
            if possible_neighbour_coords[2] == layer:
                occupancy_grid = self.occupancy_grid[layer]
            else:
                occupancy_grid = self.occupancy_grid[layer] + self.occupancy_grid[layer+1]
            if self.collision_checker(occupancy_grid, node_coords[:2], possible_neighbour_coords[:2]):
                self.graph.add_edge(node, possible_neighbour[1], weight=float(possible_neighbour[0]))
    

    def find_path(self, start, goal):
        # if start or goal already in graph, remove them
        if 'start' in self.graph.nodes:
            self.graph.remove_node('start')
        if 'goal' in self.graph.nodes:
            self.graph.remove_node('goal')

        # Add start and goal nodes to the graph, connect them to their nearest neighbors
        self.graph.add_node('start', pos=start)
        self.connect_to_nearest_neighbors('start', self.k_neighbors)
        self.graph.add_node('goal', pos=goal)
        self.connect_to_nearest_neighbors('goal', self.k_neighbors)

        # Find shortest path
        return nx.dijkstra_path(self.graph, 'start', 'goal', weight='weight') 
    
    def collision_checker(self, occupancy_grid, p1, p2=None):
        # Function that checks if either a point collides
        # OR
        # if a line connecting two points collides with an obstacle

        p1 = np.round(p1)
        if p2 is not None:
            p2 = np.round(p2)
            num_steps = np.round(self.distance(p1, p2)).astype(int)
            points = np.linspace(p1, p2, num_steps).astype(int)

            for point in points:
                if occupancy_grid[point[0], point[1]] > 0:
                    return False
            return True
        else:
            return occupancy_grid[p1[0], p1[1]] == 0
        
    def export_to_file(self, filename):
        # Export graph to file
        nx.write_weighted_edgelist(self.graph, filename)
        # nx.write_gexf(self.graph, "graph.gexf")
        nx.write_gml(self.graph, "graph.gml")
        


def generate_dummy_grid(bounds):
    grid = np.zeros((bounds[0][1], bounds[1][1]))
    # generate 6 obstacles that fall within the bounds
    for i in range(6):
        x = random.randint(bounds[0][0], bounds[0][1] - 1)
        y = random.randint(bounds[1][0], bounds[1][1] - 1)
        width = random.randint(100, 500)
        height = random.randint(100, 500)
        grid[x:x + width, y:y + height] = 1
    return grid


def plot_prm(bounds, grid, prm, start_point, goal_point, shortest_path=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for edge in prm.graph.edges:
        node1 = prm.graph.nodes[edge[0]]['pos']
        node2 = prm.graph.nodes[edge[1]]['pos']
        ax.plot([node1[0], node2[0]], [node1[1], node2[1]], [node1[2], node2[2]], 'gray', alpha=0.2, linewidth=1)

    for node in prm.graph.nodes:
        ax.scatter(prm.graph.nodes[node]['pos'][0], prm.graph.nodes[node]['pos'][1], prm.graph.nodes[node]['pos'][2], c='r', alpha=1, s=2)

    ax.scatter(start_point[0], start_point[1], start_point[2], c='purple', marker='o', s=100)
    ax.scatter(goal_point[0], goal_point[1], goal_point[2], c='g', marker='o', s=100)

    if shortest_path is not None:
        for i in range(len(shortest_path) - 1):
            node1 = prm.graph.nodes[shortest_path[i]]['pos']
            node2 = prm.graph.nodes[shortest_path[i + 1]]['pos']
            ax.plot([node1[0], node2[0]], [node1[1], node2[1]], [node1[2], node2[2]], 'g', linewidth=3)

    # Plot obstacles
    for layer in [0]:  # range(grid.shape[0]):
        for x in range(0, grid.shape[1], 4):
            for y in range(0, grid.shape[2], 4):
                if grid[layer, x, y] == 1:
                    ax.scatter(x, y, layer, c='k', marker='o', s=10)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

if __name__ == '__main__':
    print(os.getcwd())
    # Bounds of the environment
    bounds = [(0, 200), (0, 200)]

    # Start and goal points
    start_point = (20, 20, 0)
    goal_point = (160, 160, 0)

    # Generate random grid with obstacles
    grid = generate_dummy_grid(bounds)

    # print current working directory
    print(os.getcwd())

    # import grid from csv file
    grid, conversion_matrix = generate_ocupation_matrix("filled_world.world", z_layers = 2, debug=False)
    # invert the grid, 0 becomes 1, 1 becomes 0
    # grid = np.abs(grid - 1)

    # Create PRM object
    prm = PRM(num_samples=40, k_neighbors=15, occupancy_grid=grid)
    prm.build_roadmap(bounds)
    
    prm.export_to_file("graph.txt")

    

    # # save shortest path to csv file, with x and y coordinates
    # shortest_path_coords = [prm.graph.nodes[node]['pos'] for node in shortest_path]
    # # convert list of tuple to numpy array
    # shortest_path_coords = np.array(shortest_path_coords)
    # shortest_path_coords = shortest_path_coords / 10 - 10
    # np.savetxt('shortest_path.csv', shortest_path_coords, delimiter=',')

    # # Save as a list of dictionaries, with x and y coordinates and z fixed at 2.0
    # shortest_path_coords = [{'x': float(coord[0]), 'y': float(coord[1]), 'z': 2.0} for coord in shortest_path_coords]
    # # save as it would be python code, add indentation as well
    # with open('shortest_path.py', 'w') as f:
    #     f.write(str(shortest_path_coords))

    # Uncomment to visualize the PRM
    shortest_path = None
    #shortest_path = prm.find_path(start_point, goal_point)
    plot_prm(bounds, grid, prm, start_point, goal_point, shortest_path)