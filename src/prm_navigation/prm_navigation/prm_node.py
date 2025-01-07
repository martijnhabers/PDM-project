import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import yaml
import networkx as nx
from prm_navigation.prm import PRM
from ament_index_python.packages import get_package_share_directory

class PRMNode(Node):
    def __init__(self):
        super().__init__('prm_node')
        self.declare_parameter('roadmap_file', 'graph.txt')
        roadmap_file = self.get_parameter('roadmap_file').get_parameter_value().string_value
        roadmap_file = get_package_share_directory('prm_navigation') + roadmap_file

        # Load the roadmap
        self.roadmap = nx.read_weighted_edgelist(roadmap_file, nodetype=int)

        # Create a PRM object with the loaded graph
        self.prm = PRM(num_samples=0, k_neighbors=0, occupancy_grid=None)
        self.prm.graph = self.roadmap

        self.current_position = None
        self.goal_position = None

        self.create_subscription(Pose, '/current_position', self.current_position_callback, 10)
        self.create_subscription(Pose, '/goal_position', self.goal_position_callback, 10)

        self.get_logger().info('PRM Node has been started.')

    def current_position_callback(self, msg):
        self.current_position = (msg.position.x, msg.position.y)

    def goal_position_callback(self, msg):
        self.goal_position = (msg.position.x, msg.position.y)
        self.find_path()

    def find_path(self):
        if self.current_position is None or self.goal_position is None:
            self.get_logger().warn('Current or goal position not set.')
            return

        # Find shortest path
        shortest_path = self.prm.find_path(self.current_position, self.goal_position)
    

def main(args=None):
    rclpy.init(args=args)
    node = PRMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()