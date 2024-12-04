#!/usr/bin/env python3
import rclpy
from .drone_utils.drone_object import DroneObject  # Use relative import
import math

class DronePositionControl(DroneObject):
    def __init__(self, namespace='simple_drone'):
        super().__init__(node_name='drone_position_control', namespace=namespace)

        self.get_logger().info('DronePositionControl node initialized.')

        # Start the sequence after ensuring the drone has spawned
        self.timer = self.create_timer(1.0, self.check_drone_ready)

    def check_drone_ready(self):
        if self.drone_spawned:
            self.get_logger().info('Drone has spawned. Starting sequence.')
            self.start_sequence()
            # Cancel the timer to prevent further checks
            self.timer.cancel()
        else:
            self.get_logger().info('Waiting for drone to spawn...')

    def start_sequence(self):
        # Wait until gt_pose is received
        if not self.gt_pose_received:
            self.get_logger().info('Waiting for initial position data...')
            # Re-schedule this method to check again after a delay
            self.create_timer(1.0, self.start_sequence)
            return

        # Proceed with takeoff and movement
        if self.takeOff():
            self.get_logger().info('Drone takeoff command sent.')
        else:
            self.get_logger().info('Drone is already flying.')

        # Disable position control mode if previously enabled
        self.posCtrl(False)
        self.get_logger().info('Position control mode disabled.')

        # Enable velocity control mode
        self.velMode(True)
        self.get_logger().info('Velocity control mode enabled.')
        # Define waypoints with positions and yaw angles (in radians)
        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 3.0, 'yaw': 0.0},
            {'x': 2.0, 'y': 0.0, 'z': 3.0, 'yaw': math.pi / 2},
        ]

        self.current_waypoint_index = 0

        # Start navigating through waypoints
        self.navigate_waypoints()

    def navigate_waypoints(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.')
            # Optionally hover or perform other actions
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        x = waypoint['x']
        y = waypoint['y']
        z = waypoint['z']
        yaw = waypoint['yaw']

        # Move to the next waypoint
        if self.moveTo(x, y, z, yaw):
            self.get_logger().info(
                f'Moving to waypoint {self.current_waypoint_index}: x={x}, y={y}, z={z}, yaw={yaw}')
        else:
            self.get_logger().error('Failed to move to waypoint.')

        # Start timeout for waypoint
        self.start_waypoint_timeout(40.0)  # 5 seconds timeout

        # Wait and check if the waypoint is reached
        self.timer = self.create_timer(0.1, self.check_waypoint_reached)

    def start_waypoint_timeout(self, timeout):
        self.timeout_timer = self.create_timer(timeout, self.force_next_waypoint)

    def force_next_waypoint(self):
        self.get_logger().warn(f'Waypoint {self.current_waypoint_index} timed out. Moving to next waypoint.')
        self.timeout_timer.cancel()
        self.current_waypoint_index += 1
        self.navigate_waypoints()

    def check_waypoint_reached(self):
        waypoint = self.waypoints[self.current_waypoint_index]
        position_reached = self.is_position_reached(waypoint)
        yaw_reached = self.is_yaw_reached(waypoint)

        self.get_logger().info(
            f'Waypoint {self.current_waypoint_index} check: '
            f'Position reached={position_reached}, Yaw reached={yaw_reached}'
        )

        if position_reached and yaw_reached:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached.')
            self.timer.cancel()
            self.timeout_timer.cancel()  # Cancel timeout if waypoint is reached
            self.current_waypoint_index += 1
            self.navigate_waypoints()
        else:
            self.get_logger().info(
                f'Still approaching waypoint {self.current_waypoint_index}: '
                f'Position reached={position_reached}, Yaw reached={yaw_reached}'
            )



    def is_position_reached(self, waypoint, tolerance=0.9):
        if self.gt_pose is None:
            self.get_logger().error("No ground truth pose available.")
            return False
        dx = waypoint['x'] - self.gt_pose.position.x
        dy = waypoint['y'] - self.gt_pose.position.y
        dz = waypoint['z'] - self.gt_pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        self.get_logger().info(
            f'Position check: dx={dx}, dy={dy}, dz={dz}, distance={distance}, tolerance={tolerance}'
        )
        return distance < tolerance


    def is_yaw_reached(self, waypoint, tolerance=0.2):
        if self.gt_pose is None:
            self.get_logger().error("No ground truth pose available.")
            return False
        current_yaw = self.get_current_yaw()
        yaw_error = abs(waypoint['yaw'] - current_yaw)
        # Adjust yaw error for wrap-around at pi/-pi
        yaw_error = min(yaw_error, 2 * math.pi - yaw_error)

        self.get_logger().info(
            f'Yaw check: current_yaw={current_yaw}, target_yaw={waypoint["yaw"]}, '
            f'error={yaw_error}, tolerance={tolerance}'
        )
        return yaw_error < tolerance


    def get_current_yaw(self):
        orientation = self.gt_pose.orientation
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = DronePositionControl(namespace='simple_drone')
    rclpy.spin(drone_position_control_node)
    drone_position_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
