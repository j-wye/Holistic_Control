import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class JoystickBaseControl(Node):
    def __init__(self):
        super().__init__('base_control_node')
        self.qos_policy = QoSProfile(reliability = ReliabilityPolicy.RELIABLE,history = HistoryPolicy.KEEP_LAST, depth=1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, self.qos_policy)
        self.cmd_vel = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', self.qos_policy)

    def joy_cb(self, msg):
        accel = 0.5
        negative_sign = -1.0
        zero = 0.0
        base_control = Twist()
        # 전진 3방향
        if msg[10]:
            base_control.linear.x = msg[10] * accel
            base_control.angular.z = msg[10] * negative_sign
        elif msg[11]:
            base_control.linear.x = msg[11] * accel
            base_control.angular.z = msg[11] * zero
        elif msg[12]:
            base_control.linear.x = msg[12] * accel
            base_control.angular.z = msg[12]
        # 제자리 왼쪽 정지 오른쪽
        elif msg[2]:
            base_control.linear.x = msg[2] * zero
            base_control.angular.z = msg[2]
        elif msg[1]:
            base_control.linear.x = msg[1] * zero
            base_control.angular.z = msg[1] * zero
        elif msg[3]:
            base_control.linear.x = msg[3] * zero
            base_control.angular.z = msg[3] * negative_sign
        # 후진 3방향
        elif msg[13]:
            base_control.linear.x = msg[13] * accel * negative_sign
            base_control.angular.z = msg[13] * negative_sign
        elif msg[14]:
            base_control.linear.x = msg[14] * accel * negative_sign
            base_control.angular.z = msg[14] * zero
        elif msg[15]:
            base_control.linear.x = msg[15] * accel * negative_sign
            base_control.angular.z = msg[15]
        
        self.cmd_vel.publish(base_control)

class JoystickArmControl(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.qos_policy = QoSProfile(reliability = ReliabilityPolicy.RELIABLE,history = HistoryPolicy.KEEP_LAST, depth=1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, self.qos_policy)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', self.qos_policy)

    def joy_cb(self, msg):
        joint_traj = JointTrajectory()
        joint_traj.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6', 'right_finger_bottom_joint']
        joint_point = JointTrajectoryPoint()
        # joint_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # joint_point.time_from_start = 0.1
        # joint_traj.points.append(joint_point)
        # self.joint_pub.publish(joint_traj)
        
        # if msg.axes[0]:
            

def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickBaseControl()
    rclpy.spin(joystick_control_node)
    joystick_control_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()