#!/usr/bin/env python3
# (Existing License and imports remain the same)

import math
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Range, Image, Imu
# Add tf_transformations to convert quaternion to Euler
from tf_transformations import euler_from_quaternion

STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
}

MODES = ["velocity", "position"]


class DroneObject(Node):
    def __init__(self, node_name: str = "drone_node", namespace='simple_drone'):
        super().__init__(node_name=node_name, namespace=namespace)
        self._state = STATES[0]
        self._mode = MODES[0]
        self._hover_distance = 0.0
        self.isFlying = False
        self.isPosctrl = False
        self.isVelMode = False

        self.logger = self.get_logger()
        self.namespace = namespace

        # Publishers - Construct topic names using the namespace
        self.pubTakeOff = self.create_publisher(Empty, f'/{self.namespace}/takeoff', 1024)
        self.pubLand = self.create_publisher(Empty, f'/{self.namespace}/land', 1024)
        self.pubReset = self.create_publisher(Empty, f'/{self.namespace}/reset', 1024)
        self.pubPosCtrl = self.create_publisher(Bool, f'/{self.namespace}/posctrl', 1024)
        self.pubCmd = self.create_publisher(Twist, f'/{self.namespace}/cmd_vel', 1024)
        self.pubVelMode = self.create_publisher(Bool, f'/{self.namespace}/dronevel_mode', 1024)

        # Subscribers - Construct topic names using the namespace
        self.sub_sonar = self.create_subscription(
            Range, f'/{self.namespace}/sonar', self.cb_sonar, 1024)
        self.sub_imu = self.create_subscription(
            Imu, f'/{self.namespace}/imu', self.cb_imu, 1024)
        self.sub_front_img = self.create_subscription(
            Image, f'/{self.namespace}/front/image_raw', self.cb_front_img, 1024)
        self.sub_bottom_img = self.create_subscription(
            Image, f'/{self.namespace}/bottom/image_raw', self.cb_bottom_img, 1024)
        self.sub_gt_pose = self.create_subscription(
            Pose, f'/{self.namespace}/gt_pose', self.cb_gt_pose, 1024)
        self.sub_state = self.create_subscription(
            Int8, f'/{self.namespace}/state', self.cb_state, 1024)
        self.sub_cmd_mode = self.create_subscription(
            String, f'/{self.namespace}/cmd_mode', self.cb_cmd_mode, 1024)

        self._sonar = Range()
        self._imu = Imu()
        self._front_img = Image()
        self._bottom_img = Image()
        self._gt_pose = Pose()

        self.gt_pose_received = False
        self.drone_spawned = False

        # Wait until the drone is ready (subscribers are connected)
        self.create_timer(1.0, self.check_subscribers)

    def check_subscribers(self):
        # Check if the drone has spawned by checking subscriber count
        if self.pubTakeOff.get_subscription_count() > 0:
            self.drone_spawned = True

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
        self.gt_pose_received = True

    def takeOff(self):
        if self.isFlying:
            return False
        self.logger.info("Taking off")
        self.pubTakeOff.publish(Empty())
        self.isFlying = True
        return True

    def land(self):
        if not self.isFlying:
            return False
        self.logger.info("Landing")
        self.pubLand.publish(Empty())
        self.isFlying = False
        return True

    def hover(self):
        if not self.isFlying:
            return False
        twist_msg = Twist()
        self.pubCmd.publish(twist_msg)
        return True

    def posCtrl(self, on):
        self.isPosctrl = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubPosCtrl.publish(bool_msg)
        return True

    def velMode(self, on):
        if not self.isFlying:
            return False
        self.isVelMode = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubVelMode.publish(bool_msg)
        return True

    def move(self, v_linear: Vector3 = Vector3(), v_angular: Vector3 = Vector3()):
        if not self.isFlying:
            return False
        twist_msg = Twist(linear=v_linear, angular=v_angular)
        self.pubCmd.publish(twist_msg)
        return True

    def moveTo(self, x: float, y: float, z: float, yaw: float = 0.0):
        """
        Send position + yaw setpoint in position control mode.
        Assuming that posCtrl mode interprets cmd_vel as absolute position and yaw commands.
        """
        if not self.isFlying or not self.isPosctrl:
            self.logger.error("Drone must be flying and in position control mode to moveTo a position!")
            return False
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z
        twist_msg.angular.z = yaw
        self.pubCmd.publish(twist_msg)
        return True

    def pitch(self, speed):
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.angular.y = speed
        self.pubCmd.publish(twist_msg)
        return True

    def roll(self, speed: float):
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.angular.x = speed
        self.pubCmd.publish(twist_msg)
        return True

    def rise(self, speed: float):
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.z = speed
        self.pubCmd.publish(twist_msg)
        return True

    def yaw(self, speed: float):
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.angular.z = speed
        self.pubCmd.publish(twist_msg)
        return True

    def cb_sonar(self, msg: Range):
        self._sonar = msg
        self._hover_distance = msg.min_range

    def cb_imu(self, msg: Imu):
        self._imu = msg

    def cb_front_img(self, msg: Image):
        self._front_img = msg

    def cb_bottom_img(self, msg: Image):
        self._bottom_img = msg

    def cb_gt_pose(self, msg: Pose) -> None:
        self._gt_pose = msg
        self.gt_pose_received = True

    def cb_state(self, msg: Int8):
        self._state = STATES.get(msg.data, "Unknown")
        self.logger.info("State: {}".format(self._state), throttle_duration_sec=1)
        # If state is Flying or Taking off, consider drone spawned
        if msg.data in [1, 2]:  
            self.drone_spawned = True

    def cb_cmd_mode(self, msg: String):
        if msg.data in MODES:
            self._mode = msg.data
            self.logger.info("Changed command mode to: {}".format(self._mode))
        else:
            self.logger.error("Invalid command mode: {}".format(msg.data))

    def reset(self):
        self.pubReset.publish(Empty())

    def get_current_yaw(self):
        orientation = self._gt_pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw
