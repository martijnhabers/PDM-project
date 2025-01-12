import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
import os

class SDFToMarkerArray(Node):
    def __init__(self):
        super().__init__('sdf_to_marker_array')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_array = self.parse_sdf_to_marker_array()
        self.publisher.publish(self.marker_array)
        self.drone_path = Path()
        self.drone_path.header.frame_id = 'map'

        # subscriber to pose array to get trajectory, drone_trajectory topic
        self.create_subscription(PoseArray, '/drone_trajectory', self.drone_trajectory_callback, 10)

        # publisher to publish markerarray to drone_marker_trajectory topic
        self.drone_marker_trajectory_publisher = self.create_publisher(MarkerArray, 'drone_marker_trajectory', 10)

        self.drone_path_publisher = self.create_publisher(Path, 'drone_path', 10)

        self.drone_marker_publisher = self.create_publisher(Marker, 'drone_marker', 10)
        self.create_subscription(Pose, '/simple_drone/gt_pose', self.drone_position_callback, 10)

    def parse_sdf_to_marker_array(self):
        # Locate the filled_world.world file
        sdf_file = os.path.join(
            get_package_share_directory('sjtu_drone_description'),
            'worlds', 'filled_world.world'
        )

        tree = ET.parse(sdf_file)
        root = tree.getroot()

        marker_array = MarkerArray()
        marker_id = 0

        for model in root.findall('.//model'):
            name = model.get('name')
            pose = model.find('pose').text.split()
            x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
            roll, pitch, yaw = float(pose[3]), float(pose[4]), float(pose[5])

            for link in model.findall('link'):
                for collision in link.findall('collision'):
                    geometry = collision.find('geometry')
                    if geometry.find('box') is not None:
                        size = geometry.find('box').find('size').text.split()
                        sx, sy, sz = float(size[0]), float(size[1]), float(size[2])
                        marker = self.create_marker(marker_id, x, y, z, roll, pitch, yaw, sx, sy, sz, Marker.CUBE)
                        marker_array.markers.append(marker)
                        marker_id += 1
                    elif geometry.find('cylinder') is not None:
                        radius = float(geometry.find('cylinder').find('radius').text)
                        length = float(geometry.find('cylinder').find('length').text)
                        marker = self.create_marker(marker_id, x, y, z, roll, pitch, yaw, radius * 2, radius * 2, length, Marker.CYLINDER)
                        marker_array.markers.append(marker)
                        marker_id += 1

        return marker_array

    def create_marker(self, marker_id, x, y, z, roll, pitch, yaw, sx, sy, sz, marker_type):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sdf_models'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = roll
        marker.pose.orientation.y = pitch
        marker.pose.orientation.z = yaw
        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        return marker

    def drone_trajectory_callback(self, msg):
        # Convert to marker array
        marker_array = MarkerArray()
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = 'map'
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.ns = 'drone_trajectory'
        line_strip_marker.id = 0
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.05  # Line width
        line_strip_marker.color.a = 0.35    
        line_strip_marker.color.r = 0.0
        line_strip_marker.color.g = 1.0
        line_strip_marker.color.b = 0.0

        for i, pose in enumerate(msg.poses):
            # Create sphere markers
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'drone_trajectory'
            marker.id = i + 1  # Ensure unique IDs for each marker
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = pose.position
            marker.pose.orientation = pose.orientation
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            # Add points to the line strip marker
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            line_strip_marker.points.append(point)

        # Add the line strip marker to the marker array
        marker_array.markers.append(line_strip_marker)

        # Publish the marker array
        self.drone_marker_trajectory_publisher.publish(marker_array)

    def drone_position_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'drone'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = 'package://sjtu_drone_description/models/sjtu_drone/quadrotor_4.dae'
        marker.action = Marker.ADD
        marker.pose.position = msg.position
        marker.pose.orientation = msg.orientation
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.drone_marker_publisher.publish(marker)

        # Update and publish path
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = msg.position
        pose_stamped.pose.orientation = msg.orientation
        self.drone_path.poses.append(pose_stamped)
        self.drone_path.header.stamp = self.get_clock().now().to_msg()
        self.drone_path_publisher.publish(self.drone_path)


def main(args=None):
    rclpy.init(args=args)
    node = SDFToMarkerArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()