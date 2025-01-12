import random
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import k3d

import os
import json
import sys
import math
# 
# sys.path.append(os.path.abspath("src/world_creator/test/"))

# Import the module
# from WorldListExport import generate_ocupation_matrix

# set random seeds for reproducibility
random.seed(0)
np.random.seed(0)

class PRM:
    def __init__(self, num_samples, k_neighbors, occupancy_grid, conversion_matrix=np.eye(4)):
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.occupancy_grid = occupancy_grid
        self.graph = nx.Graph()
        self.start_node_idx, self.goal_node_idx = None, None
        self.conversion_matrix = conversion_matrix

    def compute_distance_matrix(self, add_goal = False):

        points_matrix = np.array([[node, 
                                   self.graph.nodes[node]['pos'][0],
                                   self.graph.nodes[node]['pos'][1], 
                                   self.graph.nodes[node]['pos'][2]] for node in self.graph.nodes])

        if not add_goal:
            # Create a distance matrix
            self.distance_matrix = np.zeros((len(self.graph.nodes), len(self.graph.nodes)))
            list_nodes = self.graph.nodes
        else:
            list_nodes = self.graph.nodes[-2:]
        norm_weight = self.conversion_matrix[:3,:3]**2

        for node1 in list_nodes:
            layer = points_matrix[node1,3]
            valid_idx = (((points_matrix[:,3] == layer) | (points_matrix[:,3] == layer +1) | (points_matrix[:,3] == layer -1)) & (points_matrix[:,0] < node1))
            aux_matrix = points_matrix[valid_idx ,1:] - points_matrix[node1, 1:]
            self.distance_matrix[valid_idx, node1] = np.diag(aux_matrix @ norm_weight @aux_matrix.T)
        
        lower_tring_idx = np.tril_indices(len(self.graph.nodes))
        self.distance_matrix[lower_tring_idx] = (self.distance_matrix.T)[lower_tring_idx]
        self.distance_matrix[self.distance_matrix == 0] = np.inf 

    def sample_free_space(self, bounds, layer):
        # Selects sample within dimension of bounds
        sample = [int(random.uniform(bound[0], bound[1])) for bound in bounds]
        sample.append(layer)
        return sample

    def build_roadmap(self, bounds):
        # Step 1: Generate random samples until we have enough
        node_id = 0
        for layer in range(self.occupancy_grid.shape[0]):
            while len(self.graph.nodes) < self.num_samples*(layer+1):
                sample = self.sample_free_space(bounds, layer)
                if self.collision_checker(sample):
                    self.graph.add_node(node_id, pos=sample)
                    node_id += 1

        #Create Distance Matrix
        self.compute_distance_matrix()

        # Step 2: Connect neighbouring nodes and add weights
        for node in self.graph.nodes:
            self.connect_to_nearest_neighbors(node, self.k_neighbors)
    
    def connect_to_nearest_neighbors(self, node, k):

        distances = np.vstack((self.distance_matrix[node,:],np.array(self.graph.nodes))).T
        distances = distances[distances[:node, 0].argsort()]

        for i, neighbour in enumerate(distances[:,1]):
            if self.collision_checker(self.graph.nodes[node]['pos'], self.graph.nodes[neighbour]['pos']):
                self.graph.add_edge(node, neighbour, weight=float(distances[i, 0]))

            if len(self.graph.edges(node)) >= k:
                return
        return   

    def find_path(self, start, goal):
        # if start or goal already in graph, remove them
        if self.start_node_idx in self.graph.nodes:
            self.graph.remove_node(self.start_node_idx)
        if self.goal_node_idx in self.graph.nodes:
            self.graph.remove_node(self.goal_node_idx)

        # TODO: Make it so that previously added start and goal nodes are not removed, instead making a more detailed graph


        # Add start and goal nodes to the graph, connect them to their nearest neighbors
        self.start_node_idx = int(max(self.graph.nodes)) + 1
        self.goal_node_idx = int(max(self.graph.nodes)) + 2

        self.graph.add_node(self.start_node_idx, pos=start)
        self.graph.add_node(self.goal_node_idx, pos=goal)

        self.compute_distance_matrix()
        self.connect_to_nearest_neighbors(self.start_node_idx, self.k_neighbors)
        self.connect_to_nearest_neighbors(self.goal_node_idx, self.k_neighbors)

        # Find shortest path
        return nx.dijkstra_path(self.graph, self.start_node_idx, self.goal_node_idx, weight='weight') 
    
    def collision_checker(self, p1,p2=None):
        # Function that checks if either a point collides
        # OR
        # if a line connecting two points collides with an obstacle
        p1 = np.array(p1)
        p2 = np.round(p1 if p2 is None else p2)
        num_steps = np.round(np.sum(np.abs(p2-p1))*2+1).astype(int)
        points = np.linspace(p1, p2, num=num_steps).astype(int)

        layers = np.unique(points[:, 2])
        points = np.unique(points[:,:2], axis=0)
        # if np.all(p1 == self.graph.nodes[self.start_node_idx]['pos']):
        #     return False
        return np.all(np.array([self.occupancy_grid[layer, points[:, 0], points[:, 1]] == 0 for layer in layers]))

    def export_to_file(self, filename):
        print("Exporting to file")
        with open(os.path.join(os.path.dirname(__file__), filename + ".json"), "w", encoding='utf-8') as f:
            json.dump(nx.cytoscape_data(self.graph), f, indent=4)
        print(os.path.join(os.path.dirname(__file__), filename + ".gml"))
        nx.write_gml(self.graph, os.path.join(os.path.dirname(__file__), filename + ".gml"))

        gml_file_path = os.path.join(os.path.dirname(__file__), filename + ".gml")

        # Read the GML file, replace all quotation marks, and write back to the file
        with open(gml_file_path, "r") as f:
            file_contents = f.read()

        modified_contents = file_contents.replace('"', '')

        with open(gml_file_path, "w") as f:
            f.write(modified_contents)
            
        


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
    for edge in prm.graph.edges:
        node1 = prm.graph.nodes[edge[0]]['pos'][:2]
        node2 = prm.graph.nodes[edge[1]]['pos'][:2]
        if prm.graph.nodes[edge[0]]['pos'][2] == 0 and prm.graph.nodes[edge[1]]['pos'][2] == 0:
            plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'gray', alpha=0.2, linewidth=1)
    for node in prm.graph.nodes:
        plt.plot(prm.graph.nodes[node]['pos'][0], prm.graph.nodes[node]['pos'][1], 'ro', alpha=1, markersize=2)

    plt.imshow(grid.T, cmap='Greys', origin='lower', extent=(bounds[0][0], bounds[0][1], bounds[1][0], bounds[1][1]))

    plt.plot(start_point[0], start_point[1], 'purple', marker='o', markersize=10)
    plt.plot(goal_point[0], goal_point[1], 'go', marker='o', markersize=10)

    if shortest_path is not None:
        for i in range(len(shortest_path) - 1):
            node1 = prm.graph.nodes[shortest_path[i]]['pos']
            node2 = prm.graph.nodes[shortest_path[i + 1]]['pos']
            plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'g', linewidth=3)

    # # Plot obstacles
    # for x in range(grid.shape[0]):
    #     for y in range(grid.shape[1]):
    #         if grid[x, y] == 1:
    #             plt.scatter(x, y, c='k', marker='o', s=10)
    plt.show()

def plot_prm_3d_k3d(bounds, grid, prm, start_point, goal_point, shortest_path=None):
    plot = k3d.plot()

    # Plot the edges
    for edge in prm.graph.edges:
        node1 = prm.graph.nodes[edge[0]]['pos']
        node2 = prm.graph.nodes[edge[1]]['pos']
        line = k3d.line(np.array([node1, node2]), shader='simple', color=0x888888)
        plot += line

    # Plot the nodes
    nodes = np.array([prm.graph.nodes[node]['pos'] for node in prm.graph.nodes])
    points = k3d.points(nodes, point_size=0.1, color=0xff0000)
    plot += points

    # Plot the start and goal points
    start_point = np.array([start_point])
    goal_point = np.array([goal_point])
    start = k3d.points(start_point, point_size=0.2, color=0x800080, label='Start')
    goal = k3d.points(goal_point, point_size=0.2, color=0x00ff00, label='Goal')
    plot += start
    plot += goal

    # Plot the shortest path if available
    if shortest_path is not None:
        path_coords = np.array([prm.graph.nodes[node]['pos'] for node in shortest_path])
        path_line = k3d.line(path_coords, shader='simple', color=0x00ff00, width=0.1)
        plot += path_line

    plot.display()

if __name__ == '__main__':
    print(os.getcwd())
    # Bounds of the environment
    bounds = [(0, 200), (0, 200)]

    # Start and goal points
    start_point = (25, 25, 0)
    goal_point = (100, 100, 3)

    # Generate random grid with obstacles
    grid = generate_dummy_grid(bounds)
    
    # print current working directory
    print(os.getcwd())

    # import grid from csv file
    # grid, conversion_matrix = generate_ocupation_matrix("filled_world.world", z_layers = 2, debug=False)

    grid = np.load('src/prm_navigation/prm_navigation/occupancy_grid.npy')
    conversion_matrix = np.load("src/prm_navigation/prm_navigation/conversion_matrix.npy")

    # invert the grid, 0 becomes 1, 1 becomes 0
    # grid = np.abs(grid - 1)

    # Create PRM object
    prm = PRM(num_samples=150, k_neighbors=5, occupancy_grid=grid, conversion_matrix=conversion_matrix)
    prm.build_roadmap(bounds)
    
    prm.export_to_file("graph")


    dummy_prm = PRM(num_samples=0, k_neighbors=15, occupancy_grid=grid, conversion_matrix=np.eye(4))
    dummy_prm.graph = nx.read_gml(os.path.join(os.path.dirname(__file__), "graph" + ".gml"))

    shortest_path = dummy_prm.find_path(start_point, goal_point)
    

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
    # shortest_path = None
    # shortest_path = prm.find_path(start_point, goal_point)
    
    for i in shortest_path:
        print(dummy_prm.graph.nodes[i]['pos'])

    # plot_prm_3d_k3d(bounds, grid, dummy_prm, start_point, goal_point, shortest_path)
    plot_prm(bounds, grid[0], dummy_prm, start_point, goal_point, shortest_path)
    plot_prm(bounds, grid[1], dummy_prm, start_point, goal_point, shortest_path)
