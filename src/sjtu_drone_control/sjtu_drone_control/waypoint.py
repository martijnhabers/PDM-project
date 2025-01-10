#!/usr/bin/env python3
from ament_index_python.packages import get_package_prefix
import math
import rclpy
from geometry_msgs.msg import Vector3, PoseArray
from std_msgs.msg import String
from drone_msgs.msg import Pose3D
from .drone_utils.drone_object_jason import DroneObject
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
import os

class DronePositionControl(DroneObject):
    def __init__(self, namespace='simple_drone'):
        super().__init__(node_name='drone_position_control', namespace=namespace)
        self.get_logger().info('DronePositionControl node initialized.')
        self.position_metrics = PositionMetrics(self)
        self.startup_flag = False

        self.action_publisher = self.create_publisher(String, 'action_topic', 10)

        # Listen for new waypoints
        self.traj_subscription = self.create_subscription(
            PoseArray,
            '/drone_trajectory',
            self.waypoint_callback,
            1024
        )

        self.path_type = 'dummy path'
        self.waypoints = [{'x': -8.0, 'y': -8.0, 'z': 1.0}, 
                          {'x': -7.1, 'y': -6.6, 'z': 3.0}] 
                          #{'x': -4.8, 'y': -5.4, 'z': 4.0}, 
                          #{'x': -4.3, 'y': -2.9000000000000004, 'z': 2.0}, {'x': -3.5, 'y': -1.8000000000000007, 'z': 2.0}, {'x': -3.2, 'y': -1.8000000000000007, 'z': 2.0}, {'x': -1.3000000000000007, 'y': -1.0999999999999996, 'z': 2.0}, {'x': -1.1999999999999993, 'y': 1.9000000000000004, 'z': 2.0}, {'x': -0.6999999999999993, 'y': 2.5999999999999996, 'z': 2.0}, {'x': 1.0, 'y': 4.1, 'z': 2.0}, {'x': 2.1999999999999993, 'y': 5.5, 'z': 2.0}, {'x': 5.6, 'y': 5.800000000000001, 'z': 2.0}, {'x': 6.0, 'y': 6.0, 'z': 2.0}]
                            
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
        self.max_lin_vel = 3
        self.max_alt_vel = 0.5
        self.max_yaw_vel = 1.8
        self.position_tolerance = 0.3

        self.previous_px = None
        self.previous_py = None
        self.previous_pz = None

        self.ready_timer = self.create_timer(1.0, self.check_drone_ready)

    def waypoint_callback(self, msg):
        self.path_type = msg.header.frame_id
        for pose in msg.poses:
            waypoint = {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            }
            self.waypoints.append(waypoint)
        self.get_logger().info(f'{len(msg.poses)} new waypoints added.')

    def check_drone_ready(self):
        if self.drone_spawned and self.gt_pose_received:
            self.ready_timer.cancel()
            self.start_sequence_timer = self.create_timer(1.0, self.start_sequence)
        else:
            self.get_logger().info('Waiting for drone to spawn and initial pose...')

    def start_sequence(self):
        if self.startup_flag == False:
            self.startup_flag = True
            if self.takeOff():
                self.get_logger().info('Drone takeoff command sent.')
            else:
                self.get_logger().info('Drone is already flying.')

            self.posCtrl(False)
            self.velMode(True)
            self.get_logger().info('Velocity control mode enabled.')

        if self.waypoints:
            self.start_sequence_timer.cancel()
            self.get_logger().info('Following waypoints...')
            self.timer = self.create_timer(0.1, self.follow_path)
        else:
            self.get_logger().info('Waiting for waypoints.')

    def follow_path(self):
        
        px = self.gt_pose.position.x
        py = self.gt_pose.position.y
        pz = self.gt_pose.position.z

        self.get_logger().info(f"Current Pose: x={px:.2f}, y={py:.2f}, z={pz:.2f}")
        self.position_metrics.log_positions()

        # Check actual distance to the final waypoint
        dx_final = self.final_x - px
        dy_final = self.final_y - py
        dz_final = self.final_z - pz
        dist_to_final = math.sqrt(dx_final*dx_final + dy_final*dy_final + dz_final*dz_final)

        if self.current_path_position >= self.total_path_length - self.position_tolerance and dist_to_final < 0.3:
            self.get_logger().info('Reached end of path. Hovering...')
            self.move(Vector3(), Vector3())  # hover in place
            self.position_metrics.log_positions()
            self.get_logger().info('Saving positions...')
            self.position_metrics.save_positions()
            self.waypoints = []
            self.action_publisher.publish(String(data="new waypoint"))
            self.timer.cancel()
            self.start_sequence_timer = self.create_timer(1.0, self.start_sequence)
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

        # Clamp velocities
        vx = max(min(vx, self.max_lin_vel), -self.max_lin_vel)
        vy = max(min(vy, self.max_lin_vel), -self.max_lin_vel)
        vz = max(min(vz, self.max_alt_vel), -self.max_alt_vel)
        vyaw = max(min(vyaw, self.max_yaw_vel), -self.max_yaw_vel)

        self.get_logger().info(
            f"Following path: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, vyaw={vyaw:.3f}, "
            f"dist_to_target={dist_to_target:.3f}, remaining={self.total_path_length - self.current_path_position:.3f}, waypoints={len(self.waypoints)}"
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


class PositionMetrics():
    def __init__(self, parent):
        self.parent = parent
        self.positions = []
        
    def log_positions(self):
        x, y, z = self.parent.gt_pose.position.x, self.parent.gt_pose.position.y, self.parent.gt_pose.position.z
        self.positions.append([x, y, z])

    def save_positions(self):
        name = f"path_{self.parent.waypoints[0]['x']},{self.parent.waypoints[0]['y']})_to_({self.parent.waypoints[-1]['x']},{self.parent.waypoints[-1]['y']}).npy"
        # Get the base workspace directory
        workspace_dir = os.path.dirname(get_package_prefix('sjtu_drone_control'))
        self.data_dir = os.path.join(workspace_dir, '..', 'data')
        

        array = np.array(self.positions)
        np.save(name, array)

        self.make_plot(array)
        self.parent.get_logger().info(f"Positions saved to {name}")
        self.positions = []
        distance_travelled = 0.0
        
        for i in range(1, len(array)):
            dx = array[i][0] - array[i-1][0]
            dy = array[i][1] - array[i-1][1]
            dz = array[i][2] - array[i-1][2]
            distance_travelled += math.sqrt(dx*dx + dy*dy + dz*dz)
        
        time_taken = len(array) * 0.1
        self.parent.get_logger().info(f"Metrics: distance_travelled={distance_travelled:.2f}, time_taken={time_taken:.2f}")
        
        #Save metrics to CSV
        csv_name = os.path.join(self.data_dir, 'metrics.csv')
        os.makedirs(os.path.dirname(csv_name), exist_ok=True)

        with open(csv_name, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.parent.waypoints[0]['x'], self.parent.waypoints[0]['y'], self.parent.waypoints[-1]['x'], self.parent.waypoints[-1]['y'], self.parent.path_type, distance_travelled, time_taken])
            self.parent.get_logger().info(f"Metrics saved: distance_travelled={distance_travelled:.2f}, time_taken={time_taken:.2f}")
        return
    
    def make_plot(self,array):
        # Extract X, Y, Z coordinates
        x_log = array[:, 0]
        y_log = array[:, 1]
        z_log = array[:, 2]

        # Extract reference path coordinates
        ref_x = [wp['x'] for wp in self.parent.waypoints]
        ref_y = [wp['y'] for wp in self.parent.waypoints]
        ref_z = [wp['z'] for wp in self.parent.waypoints]

        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the line
        ax.plot(x_log, y_log, z_log, color='b', linewidth=2, label="Sampled drone trajectory")
        
        # Plot the reference path
        ax.plot(ref_x, ref_y, ref_z, color='r', linestyle='--', linewidth=2, label="Reference Path")

        # Set labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()

        # Show the interactive plot
        plot_name = os.path.join(self.data_dir, f"plot_{self.parent.waypoints[0]['x']},{self.parent.waypoints[0]['y']}_to_{self.parent.waypoints[-1]['x']},{self.parent.waypoints[-1]['y']}.png")
        os.makedirs(os.path.dirname(plot_name), exist_ok=True)
        plt.savefig(plot_name)
        self.parent.get_logger().info('Plot displayed.')
        return
    

def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = DronePositionControl(namespace='simple_drone')
    rclpy.spin(drone_position_control_node)
    drone_position_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()