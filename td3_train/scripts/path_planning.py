import rclpy
from rclpy.node import Node
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2 as pc2
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class PathPlanning(Node):
    def __init__(self, size):
        super().__init__('path_planning')
        self.size = size
        self.pose = Point(x=0.0, y=0.0, z=0.0)
        self.orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.qos = QoSProfile(depth=10, reliability = ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.VOLATILE)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_cb, self.qos)
        self.lidar_subscription = self.create_subscription(PointCloud2, '/lidar/pointcloud', self.lidar_cb, self.qos)
        self.path_publisher = self.create_publisher(Path, '/path', self.qos)
        
        self.goal = np.array([self.size, self.size])
        
        self.path = Path()
        
        self.max_dist = 5.0
        self.min_z = -0.2
        self.max_z = 3.0
        
        self.path_action_client = ActionClient(self, ComputePathToPose, 'compute_path')
        
        self.create_timer(0.5, self.nav2_path)
    
    def odom_cb(self, msg):
        self.pose = msg.pose.pose.position
        self.orient = msg.pose.pose.orientation

    def lidar_cb(self, msg):
        data = list(pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True))
        for point in data:
            x, y, z = point
    
    def filter_lidar_data(self, data):
        filtered_data = []
        for point in data:
            x, y, z = point
            distance = math.sqrt(x**2 + y**2 + z**2)
            if z < self.min_z or z > self.max_z:
                continue
            if distance > self.max_dist:
                continue
            if math.isnan(z):
                z = self.max_z
            if math.isnan(x) or math.isnan(y):
                if math.isnan(x) and math.isnan(y):
                    x, y = math.sqrt(self.max_dist), math.sqrt(self.max_dist)
                elif math.isnan(x):
                    x = math.sqrt(self.max_dist**2 - y**2)
                elif math.isnan(y):
                    y = math.sqrt(self.max_dist**2 - x**2)
            filtered_data.append((x, y, z))
        return filtered_data
    
    def nav2_path(self):
        nav = BasicNavigator()
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal[0]
        goal_pose.pose.position.y = self.goal[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        current_pose = PoseStamped()
        current_pose.header.frame_id = 'odom'
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.pose.position.x = self.pose.x
        current_pose.pose.position.y = self.pose.y
        current_pose.pose.position.z = 0.0
        current_pose.pose.orientation.x = self.orient.x
        current_pose.pose.orientation.y = self.orient.y
        current_pose.pose.orientation.z = self.orient.z
        current_pose.pose.orientation.w = self.orient.w
        
        nav.setInitialPose(current_pose)
        # nav.waitUntilNav2Active()
        
        path = nav.getPath(self.pose, self.goal)
        
        if not path or len(path.poses) == 0:
            self.get_logger().info('경로가 비어있습니다. Nav2가 경로를 찾지 못했을 수 있습니다.')
            return
        
        smoothed_path = nav.smoothPath(path)
        self.path_publisher.publish(smoothed_path)
        self.get_logger().info(f'Path : {smoothed_path}')

        # 이제 바로 다음 지점(poses[1])을 통해 angular.z 계산
        if len(smoothed_path.poses) < 2:
            self.get_logger().info('경로가 너무 짧아서 회전값 계산 불가.')
            return

        # 현재 yaw
        current_yaw = self.quaternion_to_yaw(self.orient)

        # 다음 지점
        next_pose = smoothed_path.poses[1].pose.position

        dx = next_pose.x - self.pose.x
        dy = next_pose.y - self.pose.y
        desired_yaw = math.atan2(dy, dx)

        yaw_error = desired_yaw - current_yaw
        # -pi ~ pi 범위로 정규화
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # 간단한 P 제어(게인=1.0)
        angular_z = 1.0 * yaw_error

        # 최대 각속도 제한
        max_w = 0.5
        if angular_z > max_w:
            angular_z = max_w
        elif angular_z < -max_w:
            angular_z = -max_w

        # 출력
        self.get_logger().info(f'[PathPlanning] Next angular.z = {angular_z:.3f} rad/s')
        
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
if __name__ == '__main__':
    rclpy.init(args=None)
    
    size = 5.0
    path_planning = PathPlanning(size)
    rclpy.spin(path_planning)
    rclpy.shutdown()