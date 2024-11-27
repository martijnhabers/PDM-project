import xml.etree.ElementTree as ET

def extract_obstacles(world_file):
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

    return results