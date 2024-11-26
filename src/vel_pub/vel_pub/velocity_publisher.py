import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Float32, '/robot/velocity', 10)
        timer_period = 1.0  # 1 Hz
        self.timer = self.create_timer(timer_period, self.publish_velocity)

    def publish_velocity(self):
        msg = Float32()
        msg.data = 2.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
