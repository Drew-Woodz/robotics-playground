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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        # Only call shutdown if the context is still active
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
