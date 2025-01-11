import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
import networkx as nx
from prm_navigation.prm import PRM
from ament_index_python.packages import get_package_share_directory
import numpy as np

class PRMNode(Node):
    def __init__(self):
        super().__init__('prm_node')
        self.declare_parameter('roadmap_file', 'graph.gml')
        roadmap_file = self.get_parameter('roadmap_file').get_parameter_value().string_value
        roadmap_file = get_package_share_directory('prm_navigation') + '/' + roadmap_file

        # Load the roadmap
        # self.roadmap = nx.read_weighted_edgelist(roadmap_file, nodetype=int)
        self.roadmap = nx.read_gml(roadmap_file)

        # Create a PRM object with the loaded graph
        grid = np.loadtxt('build/prm_navigation/prm_navigation/occupancy_grid.csv', delimiter=',')

        self.prm = PRM(num_samples=0, k_neighbors=15, occupancy_grid=grid)
        self.prm.graph = self.roadmap

        self.current_position = None
        self.goal_position = None
        self.msg_previous = None
        
        self.create_subscription(Pose, '/simple_drone/gt_pose', self.current_position_callback, 10)
        self.create_subscription(Pose, '/goal_position', self.goal_position_callback, 10)

        # occupancy grid publiser
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # format the grid to be published
        self.publish_occupancy_grid(grid)


        # create publisher for desired trajectory, posearray
        self.trajectory_publisher = self.create_publisher(PoseArray, '/drone_trajectory', 10)

        self.get_logger().info('PRM Node has been started.')

    def publish_occupancy_grid(self, grid):
        grid = np.where(grid == 1, 100, grid)
        grid = np.rot90(grid)
        grid = np.flipud(grid)

        occupancygrid_msg = OccupancyGrid()
        occupancygrid_msg.header.frame_id = 'map'
        occupancygrid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancygrid_msg.info.resolution = 0.1
        occupancygrid_msg.info.width = 200
        occupancygrid_msg.info.height = 200
        occupancygrid_msg.data = (grid.flatten().astype(int).tolist())
        occupancygrid_msg.info.origin.position.x = -10.0
        occupancygrid_msg.info.origin.position.y = -10.0
        occupancygrid_msg.info.origin.position.z = 2.0
        self.occupancy_grid_publisher.publish(occupancygrid_msg)

    def current_position_callback(self, msg):
        self.current_position = [msg.position.x, msg.position.y]
        self.get_logger().info(f'Current position set to {self.current_position}', throttle_duration_sec = 2)
    
    def goal_position_callback(self, msg):
        if msg == self.msg_previous:
            return
        self.msg_previous = msg

        self.goal_position = [msg.position.x, msg.position.y]
        self.get_logger().info(f'Goal position set to {self.goal_position}')
        shortest_path = self.find_path()
        if not shortest_path:
            return
        
        self.get_logger().info('Path found')
        
        # create PoseArray message to publish the trajectory
        pose_array = PoseArray()

        # set the frame id
        pose_array.header.frame_id = 'prm'

        pose_array.header.stamp = self.get_clock().now().to_msg()

        for node in shortest_path:
            pose = Pose()
            pose.position = Point(x = float(self.roadmap.nodes[node]['pos'][0])/10-10, y = float(self.roadmap.nodes[node]['pos'][1])/10-10, z = 2.0)
            # # set orientation towards the next node
            # # not necessary, only nice for visualization
            # if node < shortest_path[-1]:
            #     next_node = shortest_path[shortest_path.index(node) + 1]
            #     next_node_pos = self.roadmap.nodes[next_node]['pos']
            #     dx = next_node_pos[0] - self.roadmap.nodes[node]['pos'][0]
            #     dy = next_node_pos[1] - self.roadmap.nodes[node]['pos'][1]
            #     yaw = np.arctan2(dy, dx)
            #     pose.orientation.z = np.sin(yaw/2)
            #     pose.orientation.w = np.cos(yaw/2)
            pose_array.poses.append(pose)

        self.trajectory_publisher.publish(pose_array)

        

    def find_path(self):
        if self.current_position is None or self.goal_position is None:
            self.get_logger().warn('Current or goal position not set.')
            return

        # Find shortest path
        current_pos_index =  [(x + 10) * 10 for x in self.current_position]
        goal_pos_index = [(x + 10) * 10 for x in self.goal_position]

        try:
            shortest_path = self.prm.find_path(current_pos_index, goal_pos_index)
        except Exception as e:
            self.get_logger().error(f'Failed to find path: {e}')
            return None
        return shortest_path
    

def main(args=None):
    rclpy.init(args=args)
    node = PRMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
