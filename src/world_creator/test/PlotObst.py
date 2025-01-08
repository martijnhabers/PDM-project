import random
import heapq
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from WorldListExport import extract_obstacles

def create_edges(points, obstacles, r):
    def is_line_intersecting_obstacle(p1, p2, obstacle):
        if obstacle["type"] == "square":
            x_min = obstacle["pose"][0]-obstacle["size"][0]/2
            y_min = obstacle["pose"][1]-obstacle["size"][1]/2
            x_max = x_min + obstacle["size"][0]/2
            y_max = y_min + obstacle["size"][1]/2
            
            # Check if the line intersects the square
            for x, y in [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]:
                if min(p1[0], p2[0]) <= x <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= y <= max(p1[1], p2[1]):
                    return True

        elif obstacle["type"] == "cylinder":
            ox = obstacle["pose"][0]
            oy = obstacle["pose"][1]
            radius = obstacle["radius"]
            
            # Check if the line intersects the cylinder
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            fx = p1[0] - ox
            fy = p1[1] - oy
            
            a = dx * dx + dy * dy
            b = 2 * (fx * dx + fy * dy)
            c = (fx * fx + fy * fy) - radius * radius
            
            discriminant = b * b - 4 * a * c
            if discriminant >= 0:
                discriminant = discriminant ** 0.5
                t1 = (-b - discriminant) / (2 * a)
                t2 = (-b + discriminant) / (2 * a)
                if 0 <= t1 <= 1 or 0 <= t2 <= 1:
                    return True
        return False

    edges = []
    for i, p1 in enumerate(points):
        for j, p2 in enumerate(points):
            if i != j:
                distance = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
                if distance <= r:
                    intersecting = False
                    for obstacle in obstacles:
                        if is_line_intersecting_obstacle(p1, p2, obstacle):
                            intersecting = True
                            break
                    if not intersecting:
                        edges.append((i, j))
    return edges

def generate_random_points(n, x_range, y_range, obstacles):
    def is_collision(x, y, obstacles):
        for obstacle in obstacles:
            ox = obstacle["pose"][0]
            oy = obstacle["pose"][1]
            if obstacle["type"] == "square":
                x_length = obstacle["size"][0]
                y_length = obstacle["size"][1]
                if ox <= x <= ox + x_length and oy <= y <= oy + y_length:
                    return True
            elif obstacle["type"] == "cylinder":
                radius = obstacle["radius"]
                if (x - ox) ** 2 + (y - oy) ** 2 <= radius ** 2:
                    return True
        return False

    points = []
    while len(points) < n:
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        if not is_collision(x, y, obstacles):
            points.append((x, y))
    return points

def plot_obstacles(results, points, edges, path=None):
    _, ax = plt.subplots()

    for obstacle in results:
        x = obstacle["pose"][0]
        y = obstacle["pose"][1]
        
        if obstacle["type"] == "square":
            x_length = obstacle["size"][0]
            y_length = obstacle["size"][1]
            square_patch = patches.Rectangle((x, y), x_length, y_length, edgecolor='blue', facecolor='lightblue', linewidth=0.1)
            ax.add_patch(square_patch)
        
        elif obstacle["type"] == "cylinder":
            radius = obstacle["radius"]
            circle_patch = patches.Circle((x, y), radius, edgecolor='red', facecolor='lightcoral', linewidth=0.1)
            ax.add_patch(circle_patch)
    
    # Plot the points
    points_x, points_y = zip(*points)
    ax.scatter(points_x, points_y, color='green', marker='o', s=1, label='Random Points')

    # Plot the edges
    for edge in edges:
        p1, p2 = edge
        ax.plot([points[p1][0], points[p2][0]], [points[p1][1], points[p2][1]], color='black', linewidth=0.5)

    # Plot the path if provided
    if path:
        path_points = [points[node] for node in path]
        path_x, path_y = zip(*path_points)
        ax.plot(path_x, path_y, color='orange', linewidth=2, label='A* Path')

    ax.set_aspect('equal', 'box')
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Cylinders, Squares, Random Points, and Edges')
    plt.legend()
    plt.grid(True)
    plt.show()

def heuristic(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

def a_star_search(start, goal, points, edges):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {i: float('inf') for i in range(len(points))}
    g_score[start] = 0
    f_score = {i: float('inf') for i in range(len(points))}
    f_score[start] = heuristic(points[start], points[goal])

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor in [j for i, j in edges if i == current] + [i for i, j in edges if j == current]:
            tentative_g_score = g_score[current] + heuristic(points[current], points[neighbor])
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(points[neighbor], points[goal])
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Usage
world_file = r'C:\Users\alexb\Desktop\Other\PMD\PDM-project\filled_world.world'
results = extract_obstacles(world_file)

# Generate 10 random points within the range of -10 to 10 for both x and y
random_points = generate_random_points(500, (-10, 10), (-10, 10), results)

# Create edges with a radius of 2
edges = create_edges(random_points, results, 2)

# Example usage of A* algorithm
start_node = 0
goal_node = len(random_points) - 1
path = a_star_search(start_node, goal_node, random_points, edges)

# Plot the obstacles, points, edges, and path
plot_obstacles(results, random_points, edges, path)
