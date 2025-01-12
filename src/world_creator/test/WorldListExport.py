import xml.etree.ElementTree as ET
import numpy as np
import math
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
import os

def generate_ocupation_matrix(world_file, z_layers = None, world_res = None, padding = None, world_size = None, debug = False):

    if z_layers is None:
        z_layers = 1
    if world_res is None:
        world_res=0.1
    if padding is None:
        padding = 0.40
    if world_size is None:
        world_size= {"x_min": -10, "y_min": -10, "x_max": 10, "y_max": 10, "z_min": 0, "z_max": 5}

    ocupation_matrix = []

    z_scale = (world_size["z_max"] - world_size["z_min"]) / (z_layers+1)

    conversion_matrix = np.array([[world_res, 0,          0,         world_size["x_min"]], 
                                  [0,        world_res,   0,         world_size["y_min"]], 
                                  [0,        0,          z_scale,    z_scale], 
                                  [0,        0,          0,          1]])
    
    results = get_objects_from_world(world_file)

    for z_level in np.linspace(world_size["z_min"], world_size["z_max"], z_layers + 2)[1:-1]:
        print(f"Processing layer at z={z_level}...")
        results, world_matrix = extract_obstacles(results, world_res, padding, world_size, z_level)
        ocupation_matrix.append(world_matrix)

        if debug:
            Test(results, world_matrix, world_res, world_size) 
 
    return np.array(ocupation_matrix), conversion_matrix

def get_objects_from_world(world_file):

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
                cylinders.append({"pose": pose, "radius": radius, "height": float(cylinder.find("length").text)})
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
        results.append({"type": "cylinder", "pose": cylinder["pose"], "radius": cylinder["radius"], "height": cylinder["height"]})

    for square in squares:
        results.append({"type": "square", "pose": square["pose"], "size": square["size"]})

    return results

def extract_obstacles(results, world_res, padding, world_size, z_layer):

    world_matrix = np.zeros((int((world_size["x_max"]-world_size["x_min"])/world_res), int((world_size["y_max"]-world_size["y_min"])/world_res)))
    
    for obstacle in results:

        if (obstacle["type"] == "cylinder") and (obstacle["pose"][2] - obstacle["height"]/2 - padding < z_layer < obstacle["pose"][2] + obstacle["height"]/2 + padding):
            
            x_idx =(obstacle["pose"][0] - world_size["x_min"])/world_res
            y_idx = (obstacle["pose"][1] - world_size["y_min"])/world_res
            r_idx = (obstacle["radius"] + padding)/world_res

            for x in range( math.ceil(x_idx-(r_idx+1)),  math.ceil(x_idx+(r_idx+1))):
                for y in range(math.ceil(y_idx-(r_idx+1)), math.ceil(y_idx+(r_idx+1))):
                    if ((x-x_idx)**2 + (y-y_idx)**2 <= (r_idx)**2) or ((x+1-x_idx)**2 + (y+1-y_idx)**2 <= (r_idx)**2) or ((x+1-x_idx)**2 + (y-y_idx)**2 <= (r_idx)**2) or ((x-x_idx)**2 + (y+1-y_idx)**2 <= (r_idx)**2):
                        
                        world_matrix[clamp(int(x))][clamp(int(y))] = 1

        elif (obstacle["type"] == "square") and (obstacle["pose"][2] - obstacle["size"][2]/2 - padding < z_layer < obstacle["pose"][2] + obstacle["size"][2]/2 + padding):
            x_idx = math.floor(obstacle["pose"][0]/world_res - world_size["x_min"]/world_res)
            y_idx = math.floor(obstacle["pose"][1]/world_res - world_size["y_min"]/world_res)

            x_min = clamp(x_idx - math.ceil((obstacle["size"][0]+padding)/(2*world_res)), 0, world_matrix.shape[0]-1)
            y_min = clamp(y_idx - math.ceil((obstacle["size"][1]+padding)/(2*world_res)), 0, world_matrix.shape[1]-1)
            x_max = clamp(x_idx + math.ceil((obstacle["size"][0]+padding)/(2*world_res)), 0, world_matrix.shape[0]-1)
            y_max = clamp(y_idx + math.ceil((obstacle["size"][1]+padding)/(2*world_res)), 0, world_matrix.shape[1]-1)

            for x in range(x_min, x_max+1):
                for y in range(y_min, y_max+1):
                    world_matrix[int(x)][int(y)] = 1

    return results, world_matrix

def clamp(n, lower=0, upper=199):
    return max(lower, min(n, upper))

def Test(results, world_matrix, world_res, world_size):

    # Create a plot
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(world_size["x_min"], world_size["x_max"])
    ax.set_ylim(world_size["y_min"], world_size["y_max"])
    ax.set_aspect('equal')

    # Paint the background according to the occupancy matrix
    for i in range(world_matrix.shape[0]):
        for j in range(world_matrix.shape[1]):
            x= i*world_res+world_size["x_min"]
            y= j*world_res+world_size["y_min"]

            if world_matrix[i, j] == 1:
                color = 'gray'
                rect = plt.Rectangle((x, y), world_res, world_res, color=color, alpha=0.5)
                ax.add_patch(rect)

    for obstacle in results:
        if obstacle["type"] == "cylinder":
            circle = plt.Circle((obstacle["pose"][0], obstacle["pose"][1]), obstacle["radius"], color='red', alpha=0.5)
            ax.add_artist(circle)

        elif obstacle["type"] == "square":
            square = plt.Rectangle((obstacle["pose"][0] - obstacle["size"][0]/2, obstacle["pose"][1] - obstacle["size"][1]/2), obstacle["size"][0], obstacle["size"][1], color='red', alpha=0.5)
            ax.add_artist(square)

    plt.show()

#world_matrix, conversion_matrix = generate_ocupation_matrix("./filled_world.world", z_layers = 2, debug=False)

world_file = os.path.join(
            get_package_share_directory("sjtu_drone_description"),
            "worlds", "filled_world.world"
        )

if __name__ == "__main__":
    world_matrix, conversion_matrix = generate_ocupation_matrix(world_file, z_layers=4, debug=False)
    np.save('src/prm_navigation/prm_navigation/occupancy_grid.npy', world_matrix.astype(int))
    np.save("src/prm_navigation/prm_navigation/conversion_matrix.npy", conversion_matrix)