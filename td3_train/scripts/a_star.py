import rclpy
from rclpy.node import Node

import math
import numpy as np
from collections import deque
import heapq  # 우선순위 큐(A*)에 사용

# 메시지/라이브러리
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs_py import point_cloud2 as pc2

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PathPlanning(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        # -------------------------
        # 사용자 설정 파라미터들
        # -------------------------
        # 맵 범위 및 해상도 (2D Occupancy Grid)
        self.resolution = 0.05      # [m/셀]  -> 1 셀당 5cm
        self.width = 200           # 그리드 가로 셀 수  -> 200 * 0.05 = 10m 범위
        self.height = 200          # 그리드 세로 셀 수  -> 10m 범위
        self.origin_x = -5.0       # 맵의 왼쪽 하단이 실제 좌표계에서 x=-5.0
        self.origin_y = -5.0       # 맵의 왼쪽 하단이 실제 좌표계에서 y=-5.0

        # 라이다 필터링 파라미터
        self.min_z = -0.2
        self.max_z = 3.0
        self.max_dist = 5.0  # 5m 넘어가는 포인트는 무시

        # 목표 위치 (예: (5.0, 5.0))
        self.goal = np.array([5.0, 5.0])

        # -------------------------
        # ROS 설정
        # -------------------------
        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Odometry 구독 (현재 로봇 위치)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            self.qos
        )

        # Lidar(PointCloud2) 구독
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/pointcloud',
            self.lidar_callback,
            self.qos
        )

        # Path 퍼블리셔
        self.path_pub = self.create_publisher(Path, '/path', self.qos)

        # 로봇 현재 위치/자세 초기값
        self.robot_position = Point(x=0.0, y=0.0, z=0.0)
        self.robot_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 최신 LiDAR 점들을 저장 (각 점: (x, y, z))
        self.lidar_points = []
        
        # occupancy grid 파라미터 (2D grid)
        self.grid_resolution = 0.5  # 한 셀 당 0.5m
        self.grid_width = 50        # x 방향 셀 개수
        self.grid_height = 50       # y 방향 셀 개수
        # grid의 원점(world 좌표): 여기서는 grid의 중앙을 (0,0)으로 두기 위해 아래와 같이 설정
        self.grid_origin = np.array([-self.grid_width * self.grid_resolution / 2.0,
                                    -self.grid_height * self.grid_resolution / 2.0])
        
        # 타이머 콜백을 통해 주기적으로 경로 계획 수행 (0.5초 주기)
        self.create_timer(0.5, self.timer_callback)

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose.position
        self.current_orient = msg.pose.pose.orientation

    def lidar_cb(self, msg):
        # LiDAR 데이터 수신: NaN 값은 skip
        self.lidar_points = list(pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True))

    def world_to_grid(self, pos):
        # world 좌표 (x,y)를 grid 인덱스로 변환
        grid_x = int((pos[0] - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((pos[1] - self.grid_origin[1]) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid):
        # grid 인덱스를 world 좌표 (x,y)로 변환 (셀 중심)
        world_x = grid[0] * self.grid_resolution + self.grid_origin[0] + self.grid_resolution/2.0
        world_y = grid[1] * self.grid_resolution + self.grid_origin[1] + self.grid_resolution/2.0
        return (world_x, world_y)

    def build_occupancy_grid(self):
        # grid 초기화: 0은 free, 1은 obstacle
        grid = np.zeros((self.grid_width, self.grid_height), dtype=np.int8)
        # LiDAR 점들을 grid에 반영
        for point in self.lidar_points:
            x, y, z = point
            # 필요시 높이(z) 필터 적용 (예: z가 -0.2~3.0 사이)
            if z < -0.2 or z > 3.0:
                continue
            gx, gy = self.world_to_grid((x, y))
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                grid[gx, gy] = 1
        return grid

    def a_star(self, grid, start, goal):
        # A* 알고리즘 구현
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 8방향 이웃 (상하좌우 및 대각선)
        neighbors = [(-1,0),(1,0),(0,-1),(0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                return self.reconstruct_path(came_from, current)
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0], neighbor[1]] == 1:
                        continue  # 장애물
                    tentative_g = g_score[current] + math.hypot(dx, dy)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []  # 경로 없음

    def heuristic(self, a, b):
        # 유클리드 거리
        return math.hypot(b[0]-a[0], b[1]-a[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def timer_callback(self):
        # 최신 LiDAR 데이터로 occupancy grid 생성
        grid = self.build_occupancy_grid()
        
        # 현재 위치와 목표를 grid 좌표로 변환
        start_world = (self.current_pose.x, self.current_pose.y)
        start_grid = self.world_to_grid(start_world)
        goal_grid = self.world_to_grid(self.goal)
        
        # A* 알고리즘으로 grid 경로 계산
        grid_path = self.a_star(grid, start_grid, goal_grid)
        if not grid_path:
            self.get_logger().info("경로를 찾지 못했습니다.")
            return
        
        # grid 경로를 world 좌표로 변환
        world_path = [self.grid_to_world(pt) for pt in grid_path]
        
        # nav_msgs/Path 메시지 생성 및 발행
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"
        for wp in world_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            # 여기서는 방향 정보는 나중에 따로 계산합니다.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info("경로가 발행되었습니다. 경로 점 개수: %d" % len(path_msg.poses))
        
        # 현재 위치와 경로상의 점들 중 가장 가까운 점을 찾아 yaw 계산
        current_pos = np.array([self.current_pose.x, self.current_pose.y])
        min_dist = float('inf')
        nearest_point = None
        for pt in world_path:
            dist = np.linalg.norm(np.array(pt) - current_pos)
            if dist < min_dist:
                min_dist = dist
                nearest_point = pt
        if nearest_point is not None:
            dx = nearest_point[0] - self.current_pose.x
            dy = nearest_point[1] - self.current_pose.y
            yaw = math.atan2(dy, dx)
            self.get_logger().info("현재 위치에서 가장 가까운 경로 점까지의 yaw: %.2f 라디안" % yaw)

def main(args=None):
    rclpy.init(args=args)
    size = 5.0
    node = PathPlanning(size)
    rclpy.spin(node)
    rclpy.shutdown()