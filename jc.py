import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateFilter(Node):
    def __init__(self):
        super().__init__('joint_state_filter')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.listener_callback, 10)
        
        self.arm_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6',
                                'right_finger_bottom_joint', 'left_finger_bottom_joint']

        self.publisher = self.create_publisher(JointState, '/filtered_joint_states', 10)

    def listener_callback(self, msg):
        filtered_msg = JointState()
        filtered_msg.header.stamp = msg.header.stamp
        
        for name in self.arm_joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                filtered_msg.name.append(name)
                filtered_msg.position.append(msg.position[idx])
                if len(msg.velocity) > idx:
                    filtered_msg.velocity.append(msg.velocity[idx])
                if len(msg.effort) > idx:
                    filtered_msg.effort.append(msg.effort[idx])

        if filtered_msg.name:
            self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
