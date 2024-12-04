#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Range, Image, Imu
import math

STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
}

MODES = ["velocity", "position"]


class DroneObject(Node):
    def __init__(self, node_name: str = "drone_node", namespace=''):
        # Initialize the node without a namespace to avoid additional nesting
        super().__init__(node_name)
        self._state = STATES[0]
        self._mode = MODES[0]
        self._hover_distance = 0.0
        self.isFlying = False
        self.isPosctrl = False
        self.isVelMode = False

        self.logger = self.get_logger()
        self.namespace = namespace

        self.gt_pose_received = False

        # Publishers - Construct topic names using the namespace
        self.pubTakeOff = self.create_publisher(Empty, f'/{self.namespace}/takeoff', 10)
        self.pubLand = self.create_publisher(Empty, f'/{self.namespace}/land', 10)
        self.pubReset = self.create_publisher(Empty, f'/{self.namespace}/reset', 10)
        self.pubPosCtrl = self.create_publisher(Bool, f'/{self.namespace}/posctrl', 10)
        self.pubCmd = self.create_publisher(Twist, f'/{self.namespace}/cmd_vel', 10)
        self.pubVelMode = self.create_publisher(Bool, f'/{self.namespace}/dronevel_mode', 10)

        # Subscribers - Construct topic names using the namespace
        self.sub_sonar = self.create_subscription(
            Range, f'/{self.namespace}/sonar', self.cb_sonar, 10)
        self.sub_imu = self.create_subscription(
            Imu, f'/{self.namespace}/imu', self.cb_imu, 10)
        self.sub_front_img = self.create_subscription(
            Image, f'/{self.namespace}/front/image_raw', self.cb_front_img, 10)
        self.sub_bottom_img = self.create_subscription(
            Image, f'/{self.namespace}/bottom/image_raw', self.cb_bottom_img, 10)
        self.sub_gt_pose = self.create_subscription(
            Pose, f'/{self.namespace}/gt_pose', self.cb_gt_pose, 10)
        self.sub_state = self.create_subscription(
            Int8, f'/{self.namespace}/state', self.cb_state, 10)
        self.sub_cmd_mode = self.create_subscription(
            String, f'/{self.namespace}/cmd_mode', self.cb_cmd_mode, 10)

        self._sonar = None
        self._imu = None
        self._front_img = None
        self._bottom_img = None
        self._gt_pose = None

        self.drone_spawned = False

        # Start a timer to check for the drone's presence
        self.create_timer(1.0, self.check_drone_spawned)

    def check_drone_spawned(self):
        # Check if the drone's takeoff topic has any subscriptions
        topic_names_and_types = self.get_topic_names_and_types()
        topics = [name for name, _ in topic_names_and_types]
        if f'/{self.namespace}/state' in topics:
            self.drone_spawned = True
            self.logger.info('Drone has spawned.')
        else:
            self.logger.info('Waiting for drone to spawn...')

    @property
    def state(self):
        return self._state

    @property
    def mode(self):
        return self._mode

    @property
    def hover_distance(self):
        return self._hover_distance

    @property
    def sonar(self):
        return self._sonar

    @property
    def imu(self):
        return self._imu

    @property
    def front_img(self):
        return self._front_img

    @property
    def bottom_img(self):
        return self._bottom_img

    @property
    def gt_pose(self):
        return self._gt_pose

    @state.setter
    def state(self, value):
        self._state = value

    @mode.setter
    def mode(self, value):
        self._mode = value

    @hover_distance.setter
    def hover_distance(self, value):
        self._hover_distance = value

    @sonar.setter
    def sonar(self, value):
        self._sonar = value

    @imu.setter
    def imu(self, value):
        self._imu = value

    @front_img.setter
    def front_img(self, value):
        self._front_img = value

    @bottom_img.setter
    def bottom_img(self, value):
        self._bottom_img = value

    @gt_pose.setter
    def gt_pose(self, value):
        self._gt_pose = value

    def takeOff(self):
        """
        Take off the drone
        :return: True if the command was sent successfully, False if drone is already flying
        """
        if self.isFlying:
            return False
        self.logger.info("Taking off")
        self.pubTakeOff.publish(Empty())
        self.isFlying = True
        return True

    def land(self):
        """
        Land the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        self.logger.info("Landing")
        self.pubLand.publish(Empty())
        self.isFlying = False
        return True

    def hover(self):
        """
        Hover the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def posCtrl(self, on):
        self.isPosctrl = on
        self._mode = MODES[1] if on else MODES[0]
        self.pubPosCtrl.publish(Bool(data=on))
        self.logger.info(f'Position control mode set to: {on}')
        return True

    def velMode(self, on):
        if not self.isFlying:
            return False
        self.isVelMode = on
        self._mode = MODES[0] if on else MODES[1]
        self.pubVelMode.publish(Bool(data=on))
        self.logger.info(f'Velocity control mode set to: {on}')
        return True



    def move(self, v_linear: Vector3 = Vector3(),
             v_angular: Vector3 = Vector3()):
        """
        Move the drone using velocity control along the linear x and z axis and rotation around
        the x, y and z axis
        :param v_linear: Linear velocity in m/s
        :param v_angular: Angular velocity in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist(linear=v_linear, angular=v_angular)
        self.pubCmd.publish(twist_msg)
        return True

    def moveTo(self, x: float, y: float, z: float, yaw: float):
        """
        Move the drone to a specific position and orientation with smooth control.
        :param x: Target X position in m
        :param y: Target Y position in m
        :param z: Target Z position in m
        :param yaw: Target yaw angle in radians
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False

        if self.gt_pose is None:
            self.logger.error('Current position not available.')
            return False

        # Calculate position errors
        error_x = x - self.gt_pose.position.x
        error_y = y - self.gt_pose.position.y
        error_z = z - self.gt_pose.position.z

        # Proportional gains for smoother motion
        Kp_position = 0.7  # Adjust this value to control speed

        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        scale = min(1.0, distance / 2.0)  # Scale velocities proportionally to the distance

        # Adjust velocities
        velocity_x = scale * Kp_position * error_x
        velocity_y = scale * Kp_position * error_y
        velocity_z = scale * Kp_position * error_z

        # Limit the linear velocities
        max_velocity = 0.5  # Maximum allowed velocity
        velocity_x = max(min(velocity_x, max_velocity), -max_velocity)
        velocity_y = max(min(velocity_y, max_velocity), -max_velocity)
        velocity_z = max(min(velocity_z, max_velocity), -max_velocity)

        # Calculate yaw error
        current_yaw = self.get_current_yaw()
        yaw_error = yaw - current_yaw
        # Adjust yaw error for wrap-around at pi/-pi
        if yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        # Proportional gain for yaw control
        Kp_yaw = 0.7  # Adjust this value to control yaw speed

        # Calculate angular velocity for yaw
        angular_z = Kp_yaw * yaw_error

        # Limit the angular velocity
        max_angular_velocity = 0.5  # Maximum allowed angular velocity
        angular_z = max(min(angular_z, max_angular_velocity), -max_angular_velocity)

        # Publish the command
        twist_msg = Twist()
        twist_msg.linear.x = velocity_x
        twist_msg.linear.y = velocity_y
        twist_msg.linear.z = velocity_z
        twist_msg.angular.z = angular_z
        self.pubCmd.publish(twist_msg)

        # Log the current motion command
        self.logger.info(
            f'Moving to ({x}, {y}, {z}, yaw={yaw}) with velocities (vx={velocity_x}, vy={velocity_y}, vz={velocity_z}, angular_z={angular_z})')
        return True

    def get_current_yaw(self):
        orientation = self.gt_pose.orientation
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def pitch(self, speed):
        """
        Pitch the drone
        :param speed: Pitch speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = speed
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def roll(self, speed: float):
        """
        Roll the drone
        :param speed: Roll speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = speed
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def rise(self, speed: float):
        """
        Rise or fall the drone
        :param speed: Rise speed in m/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def yaw(self, speed: float):
        """
        Rotate the drone around the z-axis
        :param speed: Rotation speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = speed
        self.pubCmd.publish(twist_msg)
        return True

    def cb_sonar(self, msg: Range):
        """Callback for the sonar sensor"""
        self._sonar = msg
        self._hover_distance = msg.min_range

    def cb_imu(self, msg: Imu):
        """Callback for the imu sensor"""
        self._imu = msg

    def cb_front_img(self, msg: Image):
        """Callback for the front camera"""
        self._front_img = msg

    def cb_bottom_img(self, msg: Image):
        """Callback for the rear camera"""
        self._bottom_img = msg

    def cb_gt_pose(self, msg: Pose) -> None:
        """Callback for the ground truth pose"""
        self.gt_pose = msg
        if not self.gt_pose_received:
            self.gt_pose_received = True
            self.get_logger().info('Received initial gt_pose.')

        self.logger.info(f'Received gt_pose: x={self.gt_pose.position.x}, y={self.gt_pose.position.y}, z={self.gt_pose.position.z}')


    def cb_state(self, msg: Int8):
        """Callback for the drone state"""
        self._state = STATES[msg.data]
        self.logger.info("State: {}".format(self._state), throttle_duration_sec=1)

    def cb_cmd_mode(self, msg: String):
        """Callback for the command mode"""
        if msg.data in MODES:
            self._mode = msg.data
            self.logger.info("Changed command mode to: {}".format(self._mode))
        else:
            self.logger.error("Invalid command mode: {}".format(msg.data))

    def reset(self):
        self.pubReset.publish(Empty())
