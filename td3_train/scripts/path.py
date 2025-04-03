import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from collections import deque
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.maxlen = 10000
        self.odom_path = deque(maxlen=self.maxlen)
        self.window_name = 'Path Visualization'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        
        self.cylinder_sub = self.create_subscription(Float32MultiArray, 'cylinder_coords', self.cylinder_cb, 10)
    
        self.cylinder_coords = []

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        self.visualize_path()
    
    def tf_cb(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'base_link' and transform.child_frame_id == 'base_body':
                self.pose = transform.transform.translation
                self.visualize_path()
    
    def cylinder_cb(self, msg):
        data = msg.data
        coords = []
        for i in range(0, len(data), 2):
            coords.append([data[i], data[i+1]])
        self.cylinder_coords = coords
    
    def visualize_path(self):
        x = self.pose.x
        y = self.pose.y
        
        self.odom_path.append([x, y])
        
        if len(self.odom_path) >= 2:
            current_pos = np.array(self.odom_path[-1])
            last_pos = np.array(self.odom_path[-2])
            dt_dist = np.linalg.norm(current_pos - last_pos)
            if dt_dist >= 0.3:
                new_start = self.odom_path[-1]
                self.odom_path = deque([new_start], maxlen=self.maxlen)
        
        map_size = 1400
        width = 1200
        height = 1200
        img = np.zeros((map_size, map_size, 3), dtype=np.uint8)
        
        scale = 200
        for i in range(len(self.odom_path) - 1):
            start_x = int(width - self.odom_path[i][0] * scale)
            start_y = int(height - self.odom_path[i][1] * scale)
            end_x = int(width - self.odom_path[i+1][0] * scale)
            end_y = int(height - self.odom_path[i+1][1] * scale)
            cv2.line(img, (start_y, start_x), (end_y, end_x), (0, 255, 0), 2)
        
        current_x = int(width - x * scale)
        current_y = int(height - y * scale)
        cv2.circle(img, (current_y, current_x), int(0.2*scale), (0, 0, 255), 1)
        
        for coord in self.cylinder_coords:
            cyl_x, cyl_y = coord
            pixel_x = int(width - cyl_x * scale)
            pixel_y = int(height - cyl_y * scale)
            radius = int(0.1 * scale*2)
            cv2.circle(img, (pixel_y, pixel_x), radius, (255, 0, 0), 2)
        
        cv2.rectangle(img, (0, 0), (200, 200), (0, 0, 255), 3)
        
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rclpy.init(args=None)
    path_visualizer = PathVisualizer()
    rclpy.spin(path_visualizer)
    path_visualizer.destroy_node()
    rclpy.shutdown()