import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePub(Node):
    def __init__(self):
        super().__init__('simple_pub')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.count = 0

    def tick(self):
        msg = String()
        msg.data = f"hello {self.count}"
        self.pub.publish(msg)
        self.get_logger().info(f"sent: {msg.data}")
        self.count += 1

def main():
    rclpy.init()
    node = SimplePub()
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
