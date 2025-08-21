import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSub(Node):
    def __init__(self):
        super().__init__('simple_sub')
        self.sub = self.create_subscription(String, 'chatter', self.on_msg, 10)

    def on_msg(self, msg: String):
        self.get_logger().info(f"heard: {msg.data}")

def main():
    rclpy.init()
    node = SimpleSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
