import random
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import os

# set random seeds for reproducibility
# random.seed(0)
# np.random.seed(0)

class PRM:
    def __init__(self, num_samples, k_neighbors, occupancy_grid):
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.occupancy_grid = occupancy_grid
        # self.graph = {"nodes": [], "edges": {}}  # Graph representation
        self.graph = nx.Graph()

    def sample_free_space(self, bounds):
        # Selects sample within dimension of bounds
        sample = [int(random.uniform(bound[0], bound[1])) for bound in bounds]
        return sample

    def distance(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def build_roadmap(self, bounds):
        # Step 1: Generate random samples until we have enough
        node_id = 0
        while len(self.graph.nodes) < self.num_samples:
            sample = self.sample_free_space(bounds)
            if self.collision_checker(self.occupancy_grid, sample):
                self.graph.add_node(node_id, pos=sample)
                node_id += 1

        # Step 2: Connect neighbouring nodes and add weights
        for node in self.graph.nodes:
            self.connect_to_nearest_neighbors(node, self.k_neighbors)
    
    def connect_to_nearest_neighbors(self, node, k):
        distances = []
        for neighbour in self.graph.nodes:
            if node != neighbour:
                distances.append((self.distance(self.graph.nodes[node]['pos'], self.graph.nodes[neighbour]['pos']), neighbour))
        distances.sort()

        while len(self.graph.edges(node)) < k and distances:
            possible_neighbour = distances.pop(0)
            node_coords = self.graph.nodes[node]['pos']
            possible_neighbour_coords = self.graph.nodes[possible_neighbour[1]]['pos']
            if self.collision_checker(self.occupancy_grid, node_coords, possible_neighbour_coords):
                self.graph.add_edge(node, possible_neighbour[1], weight=possible_neighbour[0])
    

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
                if occupancy_grid[point[0], point[1]] == 1:
                    return False
            return True
        else:
            return occupancy_grid[p1[0], p1[1]] == 0
        
    def export_to_file(self, filename):
        # Export graph to file
        nx.write_weighted_edgelist(self.graph, filename)
        


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
        node1 = prm.graph.nodes[edge[0]]['pos']
        node2 = prm.graph.nodes[edge[1]]['pos']
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

    plt.show()

if __name__ == '__main__':
    print(os.getcwd())
    # Bounds of the environment
    bounds = [(0, 1000), (0, 1000)]

    # Start and goal points
    start_point = (10, 550)
    goal_point = (950, 500)

    # Generate random grid with obstacles
    grid = generate_dummy_grid(bounds)

    # Create PRM object
    prm = PRM(num_samples=250, k_neighbors=15, occupancy_grid=grid)
    prm.build_roadmap(bounds)

    prm.export_to_file("graph.txt")

    # Uncomment to visualize the PRM
    # shortest_path = prm.find_path(start_point, goal_point)
    # plot_prm(bounds, grid, prm, start_point, goal_point, shortest_path)