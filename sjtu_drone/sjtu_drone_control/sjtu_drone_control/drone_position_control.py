import rclpy
from rclpy.node import Node
from drone_msgs.msg import Pose3D  # Assuming Pose3D is from geometry_msgs
from geometry_msgs.msg import Pose
from .drone_utils.drone_object import DroneObject

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')

        self.takeOff()
        self.get_logger().info('Drone takeoff')

        # Set the m_posCtrl flag to True
        self.posCtrl(True)
        self.get_logger().info('Position control mode set to True')

        self.subscription = self.create_subscription(
                Pose3D,
                'drone_trajectory',
                self.listener_callback,
                1024
            )
            
        self.pos_sub = self.create_subscription(
        	Pose,
        	'simple_drone/gt_pose',
        	self.location_update,
        	5
        )

    def location_update(self, msg):
        x_gt = msg.position.x
        y_gt = msg.position.y
        z_gt = msg.position.z
        _, _, yaw_gt = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    
    def listener_callback(self, msg):
        # Extract x, y, z (position), and yaw (orientation.z) from Pose3D message
        x = msg.x
        y = msg.y
        z = msg.z
        yaw = msg.yaw  # yaw is directly provided by orientation.z as an Euler angle

        # Log the received pose
        self.get_logger().info(f'I heard: x={x}, y={y}, z={z}, yaw={yaw}')

        # Pass the values to move the drone
        self.move_drone_to_pose(x, y, z)

    def move_to_next_waypoint(self):
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("No more waypoints to move towards.")
            return

        target_x, target_y, target_z, target_yaw = self.waypoints[self.current_waypoint_index]

        if self.current_position is None:
            self.get_logger().info("Waiting for position update...")
            return

        # Calculate the distance to the current waypoint
        distance_to_waypoint = math.sqrt(
            (target_x - self.current_position.x)**2 +
            (target_y - self.current_position.y)**2 +
            (target_z - self.current_position.z)**2
            )

        # If within the threshold distance, move to the next waypoint
        distance_threshold = 1.0  # Adjust this threshold based on your requirement
        if distance_to_waypoint < distance_threshold:
            self.get_logger().info(f"Arrived at waypoint {self.current_waypoint_index + 1}")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index + 1}")
            else:
                self.get_logger().info("All waypoints reached.")
        else:
            # Move towards the current waypoint
            self.MoveTo(target_x, target_y, target_z, target_yaw)


def main(args=None):
    rclpy.init(args=args)

    # Instantiate DronePositionControl before passing to the subscriber
    drone_position_control_node = DronePositionControl()

    rclpy.spin(drone_position_control_node)

    drone_position_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

