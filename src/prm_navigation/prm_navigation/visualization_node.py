import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
import os

class SDFToMarkerArray(Node):
    def __init__(self):
        super().__init__('sdf_to_marker_array')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_array = self.parse_sdf_to_marker_array()
        self.publisher.publish(self.marker_array)

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
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker
    
    def drone_position_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'drone'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = msg.position
        marker.pose.orientation = msg.orientation
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.drone_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SDFToMarkerArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()