import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class HumanoidControlAgent(Node):
    """
    A simple Python agent that publishes joint commands to make the robot wave.
    """
    def __init__(self):
        super().__init__('humanoid_control_agent')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Humanoid control agent started.')
        self.time = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Make the arm wave
        angle = 1.5 * math.sin(self.time)
        
        msg.name = ['torso_to_upper_arm']
        msg.position = [angle]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint angle: {angle:.2f}')
        
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    agent = HumanoidControlAgent()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
