import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
import networkx as nx
from prm_navigation.prm import PRM
from prm_navigation.rrt import RRT3D
from ament_index_python.packages import get_package_share_directory
import numpy as np
from random import randint

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
        grid = np.load('src/prm_navigation/prm_navigation/occupancy_grid.npy')
        conversion_matrix = np.load('src/prm_navigation/prm_navigation/conversion_matrix.npy')

        self.prm = PRM(num_samples=0, k_neighbors=15, occupancy_grid=grid, conversion_matrix=conversion_matrix)
        self.prm.graph = self.roadmap

        self.current_position = None
        self.goal_position = None
        self.msg_previous = None
        
        self.create_subscription(Pose, '/simple_drone/gt_pose', self.current_position_callback, 10)
        self.create_subscription(Pose, '/goal_position', self.goal_position_callback, 10)

        # occupancy grid publiser
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # format the grid to be published
        # self.publish_occupancy_grid(grid)


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
        self.current_position = [msg.position.x, msg.position.y, msg.position.z]
        self.get_logger().info(f'Current position set to {self.current_position}', throttle_duration_sec = 2)
    
    def goal_position_callback(self, msg):
        if msg == self.msg_previous:
            return
        self.msg_previous = msg

        self.goal_position = [msg.position.x, msg.position.y, msg.position.z]
        self.get_logger().info(f'Goal position set to {self.goal_position}')
        shortest_path = self.find_path()

        if not shortest_path:
            return
        
        self.get_logger().info('Path found')
        
        # create PoseArray message to publish the trajectory
        pose_array = PoseArray()

        # set the frame id
        pose_array.header.frame_id = 'map'

        pose_array.header.stamp = self.get_clock().now().to_msg()

        for node in shortest_path:
            pose = Pose()
            point_idx = np.array(self.roadmap.nodes[node]['pos'])
            point_idx = np.append(point_idx, 1.0)

            point_coord = np.dot(self.prm.conversion_matrix, point_idx)
            
            print("point_coord: ", point_coord)

            pose.position = Point(x = float(point_coord[0]),
                                  y = float(point_coord[1]),
                                  z = float(point_coord[2]))
            
            pose_array.poses.append(pose)

        self.trajectory_publisher.publish(pose_array)

    def clamp(self, n):
        for i, val in enumerate(n):
            if i == 2:
                n[i] = max(0, min(val, 3))
            else:
                n[i] = max(0, min(val, 199))
        return n

    def find_path(self):
        if self.current_position is None or self.goal_position is None:
            self.get_logger().warn('Current or goal position not set.')
            return

        current_pos = np.array(self.current_position)
        current_pos = np.append(current_pos, 1.0)

        current_pos_index = np.dot(np.linalg.inv(self.prm.conversion_matrix), current_pos)[:3].astype(int)
        current_pos_index = self.clamp(current_pos_index)

        goal_pos = np.array(self.goal_position)
        goal_pos = np.append(goal_pos, 1.0)

        goal_pos_index = np.dot(np.linalg.inv(self.prm.conversion_matrix), goal_pos)[:3].astype(int)
        goal_pos_index = self.clamp(goal_pos_index)

        print("current_pos_index: ", np.round(current_pos_index).astype(int))
        print("goal_pos_index: ", np.round(goal_pos_index).astype(int))

        try:
            shortest_path_prm = self.prm.find_path(np.round(current_pos_index).astype(int), np.round(goal_pos_index).astype(int))
        except Exception as e:
            self.get_logger().error(f'Failed to find path: {e}')
            return
            
        #RRT:
        """try:
            self.rrt = RRT3D(current_pos_index, goal_pos_index, self.occupancy_grid, step_size=5, max_iter=1000)
            shortest_path_rrt = self.rrt.plan()
        except Exception as e:
            self.get_logger().error(f'Failed to find RRT path: {e}')"""
    
        return shortest_path_prm
    

def main(args=None):
    rclpy.init(args=args)
    node = PRMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
