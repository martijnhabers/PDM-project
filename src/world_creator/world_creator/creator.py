#!/usr/bin/env python3
import random
import math
from ament_index_python.packages import get_package_share_directory
import os

def main():
    try:
        world_file = os.path.join(
            get_package_share_directory("sjtu_drone_description"),
            "worlds", "filled_world.world"
        )

        generate_world(
            max_objects=100,       # Max number of objects to try and place
            area_size=10,          # Size of the placement area (-10 to 10)
            max_height=5.0,        # Maximum height for objects
            world_file=world_file
        )
    except Exception as e:
        print(f"Unexpected error: {e}")


def generate_world(max_objects, area_size, max_height, world_file):
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
            height = random.uniform(1.0, 3.0)  # Vary the height of the objects
            z = random.uniform(0, max_height - height)  # Ensure the object does not go below 0 or above max_height

            if is_valid_position(x, y, z, size, placed_objects):
                placed_objects.append({"x": x, "y": y, "z": z, "size": size, "height": height})
                obstacles.append(generate_obstacle(f"obstacle_{i}", x, y, z, size, shape, height))
                break
            
            attempts += 1

        if attempts == 100:
            print(f"Warning: Could not place object {i} after 100 attempts.")

    save_world(obstacles, world_file)


def is_valid_position(x, y, z, size, placed_objects):
    """Check if the position is valid (no overlap with existing objects)."""
    for obj in placed_objects:
        dist = math.sqrt((x - obj["x"])**2 + (y - obj["y"])**2 + (z - obj["z"])**2)
        if dist < (size + obj["size"]):
            return False
    return True


def generate_obstacle(name, x, y, z, size, shape, height):
    """Generate the SDF XML for an obstacle."""
    if shape == "box":
        return f"""
        <model name='{name}'>
            <static>true</static>
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>{size} {size} {height}</size>
                        </box>
                    </geometry>
                </collision>
                <visual name='visual'>
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
        <model name='{name}'>
            <static>true</static>
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <cylinder>
                            <radius>{size / 2}</radius>
                            <length>{height}</length>
                        </cylinder>
                    </geometry>
                </collision>
                <visual name='visual'>
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


def save_world(obstacles, world_file):
    """Save the generated world to a file."""
    world_template = f"""
    <sdf version='1.6'>
        <world name='default'>
            <include>
                <uri>model://ground_plane</uri>
            </include>
            <include>
                <uri>model://sun</uri>
            </include>
            {"".join(obstacles)}
        </world>
    </sdf>
    """
    with open(world_file, 'w') as f:
        f.write(world_template)


if __name__ == "__main__":
    main()