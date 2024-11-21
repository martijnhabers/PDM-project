import random
import math

def generate_world(max_objects, area_size, world_file):
    """Generate a Gazebo world with randomly placed objects."""
    placed_objects = []
    obstacles = []

    for i in range(max_objects):
        attempts = 0
        while attempts < 100:  # Try up to 100 times to find a valid position
            x = random.uniform(-area_size, area_size)
            y = random.uniform(-area_size, area_size)
            size = random.uniform(0.5, 2.0)  # Object size (between 0.5m and 2m)
            shape = random.choice(["box", "cylinder"])
            height = 3.0  # Set height to always be 3 meters
            
            if is_valid_position(x, y, size, placed_objects):
                placed_objects.append({"x": x, "y": y, "size": size, "height": height})
                obstacles.append(generate_obstacle(f"obstacle_{i}", x, y, size, shape, height))
                break
            
            attempts += 1

        if attempts == 100:
            print(f"Warning: Could not place object {i} after 100 attempts.")

    # Write the world file
    try:
        with open(world_file, "w") as file:
            file.write(generate_world_file_content(obstacles))
        print(f"World file '{world_file}' generated successfully.")
    except Exception as e:
        print(f"Error writing world file: {e}")


def generate_obstacle(name, x, y, size, shape, height):
    """Generate the XML for a single obstacle."""
    if shape == "box":
        return f"""
        <model name="{name}">
            <pose>{x} {y} {height / 2} 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>{size} {size} {height}</size>
                        </box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>{size} {size} {height}</size>
                        </box>
                    </geometry>
                </visual>
            </link>
        </model>
        """
    elif shape == "cylinder":
        return f"""
        <model name="{name}">
            <pose>{x} {y} {height / 2} 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <cylinder>
                            <radius>{size / 2}</radius>
                            <length>{height}</length>
                        </cylinder>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <cylinder>
                            <radius>{size / 2}</radius>
                            <length>{height}</length>
                        </cylinder>
                    </geometry>
                </visual>
            </link>
        </model>
        """
    else:
        print(f"Error: Unknown shape '{shape}'.")
        return ""


def is_valid_position(x, y, size, placed_objects):
    """Check if the object can be placed at the given position without overlapping."""
    for obj in placed_objects:
        distance = math.sqrt((obj["x"] - x)**2 + (obj["y"] - y)**2)
        if distance < (obj["size"] + size):  # Ensure no overlap
            return False
    return True


def generate_world_file_content(obstacles):
    """Generate the XML content for the world file, including a ground plane."""
    ground_plane = """
        <include>
            <uri>model://ground_plane</uri>
        </include>
    """
    return f"""
    <sdf version="1.6">
        <world name="default">
            {ground_plane}
            {"".join(obstacles)}
        </world>
    </sdf>
    """



# Parameters for the world
if __name__ == "__main__":
    try:
        generate_world(
            max_objects=100,       # Max number of objects to try and place
            area_size=10,          # Size of the placement area (-10 to 10)
            world_file="filled_world.world"
        )
    except Exception as e:
        print(f"Unexpected error: {e}")

