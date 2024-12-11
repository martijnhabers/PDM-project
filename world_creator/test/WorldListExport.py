import xml.etree.ElementTree as ET
import numpy as np
import math
import matplotlib.pyplot as plt

def extract_obstacles(world_file):

    word_res=0.1
    padding = 0.1
    word_size= {"x_min": -10, "y_min": -10, "x_max": 10, "y_max": 10, "z_min": 0, "z_max": 10}

    """
    Extract all obstacles and store:
    - Cylinders: pose and radius
    - Squares: pose and side lengths
    """
    cylinders = []
    squares = []
    
    try:
        tree = ET.parse(world_file)
        root = tree.getroot()
        
        # Iterate through all models in the world
        for model in root.findall(".//model"):
            pose_element = model.find("pose")
            pose = [float(value) for value in pose_element.text.split()] if pose_element is not None else [0, 0, 0, 0, 0, 0]
            
            # Check for cylinder geometry
            cylinder = model.find(".//geometry/cylinder")
            if cylinder is not None:
                radius = float(cylinder.find("radius").text)
                cylinders.append({"pose": pose, "radius": radius})
                continue  # No need to check for box if it's a cylinder
            
            # Check for box geometry
            box = model.find(".//geometry/box")
            if box is not None:
                size = [float(value) for value in box.find("size").text.split()]
                squares.append({"pose": pose, "size": size})
    
    except ET.ParseError as e:
        print(f"Error parsing the .world file: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    
    results = []

    for cylinder in cylinders:
        results.append({"type": "cylinder", "pose": cylinder["pose"], "radius": cylinder["radius"]})

    for square in squares:
        results.append({"type": "square", "pose": square["pose"], "size": square["size"]})

    word_matrix = np.zeros((int((word_size["x_max"]-word_size["x_min"])/word_res), int((word_size["y_max"]-word_size["y_min"])/word_res)))
    
    for obstacle in results:

        if obstacle["type"] == "cylinder":

            x_idx =(obstacle["pose"][0] - word_size["x_min"])/word_res
            y_idx = (obstacle["pose"][1] - word_size["y_min"])/word_res
            r_idx = (obstacle["radius"] + padding)/word_res

            for x in range( math.ceil(x_idx-(r_idx+1)),  math.ceil(x_idx+(r_idx+1))):
                for y in range(math.ceil(y_idx-(r_idx+1)), math.ceil(y_idx+(r_idx+1))):
                    if ((x-x_idx)**2 + (y-y_idx)**2 <= (r_idx)**2) or ((x+1-x_idx)**2 + (y+1-y_idx)**2 <= (r_idx)**2) or ((x+1-x_idx)**2 + (y-y_idx)**2 <= (r_idx)**2) or ((x-x_idx)**2 + (y+1-y_idx)**2 <= (r_idx)**2):
                        
                        word_matrix[clamp(int(x))][clamp(int(y))] = 1

        elif obstacle["type"] == "square":
            x_idx = math.floor(obstacle["pose"][0]/word_res+100)
            y_idx = math.floor(obstacle["pose"][1]/word_res+100)

            x_min = clamp(x_idx - math.ceil((obstacle["size"][0]+padding)/(2*word_res)))
            y_min = clamp(y_idx - math.ceil((obstacle["size"][1]+padding)/(2*word_res)))
            x_max = clamp(x_idx + math.ceil((obstacle["size"][0]+padding)/(2*word_res)))
            y_max = clamp(y_idx + math.ceil((obstacle["size"][1]+padding)/(2*word_res)))

            for x in range(x_min, x_max+1):
                for y in range(y_min, y_max+1):
                    word_matrix[int(x)][int(y)] = 1

    return results, word_matrix

def clamp(n, lower=0, upper=199):
    return max(lower, min(n, upper))

def Test(results, word_matrix):
    # Create a plot
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')

    # Paint the background according to the occupancy matrix
    for i in range(200):
        for j in range(200):
            x= i*0.1-10
            y= j*0.1-10

            if word_matrix[i, j] == 1:
                color = 'gray'
                rect = plt.Rectangle((x, y), 0.1, 0.1, color=color, alpha=0.5)
                ax.add_patch(rect)

    for obstacle in results:
        if obstacle["type"] == "cylinder":
            circle = plt.Circle((obstacle["pose"][0], obstacle["pose"][1]), obstacle["radius"], color='red', alpha=0.5)
            ax.add_artist(circle)

        elif obstacle["type"] == "square":
            square = plt.Rectangle((obstacle["pose"][0] - obstacle["size"][0]/2, obstacle["pose"][1] - obstacle["size"][1]/2), obstacle["size"][0], obstacle["size"][1], color='red', alpha=0.5)
            ax.add_artist(square)

    plt.show()

results, word_matrix = extract_obstacles("./filled_world.world")

Test(results, word_matrix)