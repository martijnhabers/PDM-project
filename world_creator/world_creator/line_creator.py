import numpy as np
import rclpy
from rclpy.node import Node
from drone_msgs.msg import Pose3D
import math


class LineCreator:
    def __init__(self, x_bounds, y_bounds, z_bounds, num_waypoints, t_range=(0, 1)):
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.z_bounds = z_bounds
        self.num_waypoints = num_waypoints
        self.t_range = t_range

    def generate_random_smooth_curve(self):
        # Generate random coefficients for the parametric curve
        a_x, b_x, c_x = np.random.uniform(-1, 1, 3)
        a_y, b_y, c_y = np.random.uniform(-1, 1, 3)
        a_z, b_z = np.random.uniform(-1, 1, 2)
        
        # Define the parametric functions
        def parametric_curve(t):
            x = a_x * np.sin(b_x * t) + c_x * t  # Random sinusoidal + linear
            y = a_y * np.cos(b_y * t) + c_y * t  # Random sinusoidal + linear
            z = (a_z * np.sin(b_z * t) + t) * (self.z_bounds[1] - self.z_bounds[0]) + self.z_bounds[0]  # Keep z within bounds
            return np.array([x, y, z])

        # Discretize the curve
        t_values = np.linspace(*self.t_range, self.num_waypoints)
        waypoints = np.array([parametric_curve(t) for t in t_values])
        
        # Calculate orientation (yaw) for each waypoint
        tangents = np.gradient(waypoints, axis=0)  # Tangent vectors of the curve
        yaws = np.arctan2(tangents[:, 1], tangents[:, 0])  # Yaw is the angle in the xy-plane
        
        # Combine waypoints with orientation
        waypoints_with_orientation = np.hstack([waypoints, yaws[:, None]])
        
        return waypoints_with_orientation


class MinimalPublisher(Node):
    def __init__(self, line_creator):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose3D, 'drone_trajectory', 1000)  # Publish to 'drone_trajectory' topic
        self.line_creator = line_creator
        self.waypoints = self.line_creator.generate_random_smooth_curve()
        self.current_index = 0
        self.timer_period = 5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            x, y, z, yaw = waypoint[0], waypoint[1], waypoint[2], waypoint[3]
            
            # Create the Pose3D message
            msg = Pose3D()
            msg.x = x
            msg.y = y
            msg.z = z
            
            msg.yaw = yaw  # Cos of half yaw angle
            
            # Publish the Pose3D message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")
            self.current_index += 1
        else:
            self.get_logger().info("All waypoints have been published.")
            self.timer.cancel()


def main(args=None):
    try:
        rclpy.init(args=args)
        x_bounds = (-5, 5)
        y_bounds = (-5, 5)
        z_bounds = (0, 3)
        num_waypoints = 50

        line_creator = LineCreator(x_bounds, y_bounds, z_bounds, num_waypoints)
        minimal_publisher = MinimalPublisher(line_creator)

        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

