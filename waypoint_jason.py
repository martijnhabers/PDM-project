#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from .drone_utils.drone_object import DroneObject

class DronePositionControl(DroneObject):
    def __init__(self, namespace='simple_drone'):
        super().__init__(node_name='drone_position_control', namespace=namespace)
        self.get_logger().info('DronePositionControl node initialized.')

        # Waypoints in a square path
        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 3.0},
            {'x': 5.0, 'y': 0.0, 'z': 3.0},
            {'x': 5.0, 'y': 5.0, 'z': 3.0},
            {'x': 0.0, 'y': 5.0, 'z': 3.0},
            {'x': 0.0, 'y': 0.0, 'z': 3.0}
        ]

        # Construct piecewise linear path
        self.path_points = [(wp['x'], wp['y'], wp['z']) for wp in self.waypoints]
        self.segment_distances = []
        self.total_path_length = 0.0
        for i in range(len(self.path_points) - 1):
            dx = self.path_points[i+1][0] - self.path_points[i][0]
            dy = self.path_points[i+1][1] - self.path_points[i][1]
            dz = self.path_points[i+1][2] - self.path_points[i][2]
            seg_len = math.sqrt(dx*dx + dy*dy + dz*dz)
            self.segment_distances.append(seg_len)
            self.total_path_length += seg_len

        # Set final waypoint
        self.final_waypoint = self.path_points[-1]
        self.final_x, self.final_y, self.final_z = self.final_waypoint

        # A smaller lookahead for sharper corners
        self.lookahead_distance = 0.2
        self.current_path_position = 0.0

        # Control parameters
        self.kp_pos = 0.2
        self.kp_yaw = 0.2
        self.max_lin_vel = 0.5
        self.max_alt_vel = 0.5
        self.max_yaw_vel = 0.6
        self.position_tolerance = 0.3

        self.previous_px = None
        self.previous_py = None
        self.previous_pz = None

        self.ready_timer = self.create_timer(1.0, self.check_drone_ready)

    def check_drone_ready(self):
        if self.drone_spawned and self.gt_pose_received:
            self.ready_timer.cancel()
            self.start_sequence()
        else:
            self.get_logger().info('Waiting for drone to spawn and initial pose...')

    def start_sequence(self):
        if self.takeOff():
            self.get_logger().info('Drone takeoff command sent.')
        else:
            self.get_logger().info('Drone is already flying.')

        self.posCtrl(False)
        self.velMode(True)
        self.get_logger().info('Velocity control mode enabled.')

        self.timer = self.create_timer(0.1, self.follow_path)

    def follow_path(self):
        px = self.gt_pose.position.x
        py = self.gt_pose.position.y
        pz = self.gt_pose.position.z

        self.get_logger().info(f"Current Pose: x={px:.2f}, y={py:.2f}, z={pz:.2f}")

        # Check actual distance to the final waypoint
        dx_final = self.final_x - px
        dy_final = self.final_y - py
        dz_final = self.final_z - pz
        dist_to_final = math.sqrt(dx_final*dx_final + dy_final*dy_final + dz_final*dz_final)

        if self.current_path_position >= self.total_path_length - self.position_tolerance and dist_to_final < 0.3:
            self.get_logger().info('Reached end of path. Hovering...')
            self.move(Vector3(), Vector3())  # hover in place
            return

        target_s = min(self.current_path_position + self.lookahead_distance, self.total_path_length)
        tx, ty, tz = self.get_point_on_path(target_s)

        dx = tx - px
        dy = ty - py
        dz = tz - pz
        dist_to_target = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist_to_target > 0.05:
            self.current_path_position += 0.02

        dir_s = min(target_s + 0.1, self.total_path_length)
        dir_x, dir_y, dir_z = self.get_point_on_path(dir_s)
        path_dx = dir_x - tx
        path_dy = dir_y - ty
        desired_yaw = math.atan2(path_dy, path_dx)

        current_yaw = self.get_current_yaw()
        yaw_error = desired_yaw - current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        local_dx = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
        local_dy = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)

        vx = self.kp_pos * local_dx
        vy = self.kp_pos * local_dy
        vz = self.kp_pos * dz
        vyaw = self.kp_yaw * yaw_error

        remaining = self.total_path_length - self.current_path_position
        slowdown_factor = max(0.1, min(1.0, remaining / 2.0))
        vx *= slowdown_factor
        vy *= slowdown_factor
        vz *= slowdown_factor

        vx = max(min(vx, self.max_lin_vel), -self.max_lin_vel)
        vy = max(min(vy, self.max_lin_vel), -self.max_lin_vel)
        vz = max(min(vz, self.max_alt_vel), -self.max_alt_vel)
        vyaw = max(min(vyaw, self.max_yaw_vel), -self.max_yaw_vel)

        self.get_logger().info(
            f"Following path: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, vyaw={vyaw:.3f}, "
            f"dist_to_target={dist_to_target:.3f}, remaining={remaining:.3f}"
        )

        linear_vec = Vector3(x=vx, y=vy, z=vz)
        angular_vec = Vector3(z=vyaw)
        self.move(linear_vec, angular_vec)

    def get_point_on_path(self, s):
        remaining = s
        for i in range(len(self.path_points) - 1):
            seg_len = self.segment_distances[i]
            if remaining <= seg_len:
                x1, y1, z1 = self.path_points[i]
                x2, y2, z2 = self.path_points[i+1]
                ratio = remaining / seg_len
                x = x1 + ratio * (x2 - x1)
                y = y1 + ratio * (y2 - y1)
                z = z1 + ratio * (z2 - z1)
                return x, y, z
            else:
                remaining -= seg_len
        return self.path_points[-1]

    def get_current_yaw(self):
        orientation = self.gt_pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
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
