#!/usr/bin/env bash
set -euo pipefail

# Usage: from repo root
#   chmod +x scripts/bootstrap_ros2_basics.sh
#   ./scripts/bootstrap_ros2_basics.sh

# Resolve the repo root even if running from a subfolder
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

# Put all ROS2 content under the ROS2 directory
ROS2_ROOT="$REPO_ROOT/ROS2"
WS="$ROS2_ROOT/ros2_ws"
PKG_NAME="basics_pubsub"
PKG_DIR="$WS/src/$PKG_NAME"

# Ensure expected directories exist
mkdir -p "$ROS2_ROOT/scripts" "$PKG_DIR/$PKG_NAME" "$PKG_DIR/resource" "$PKG_DIR/launch"


# .gitignore at repo root
cat > "$ROS2_ROOT/.gitignore" << 'EOF'
build/
install/
log/
**/__pycache__/
*.pyc
.vscode/
.idea/
EOF

# package.xml
cat > "$PKG_DIR/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>basics_pubsub</name>
  <version>0.0.1</version>
  <description>Simple ROS2 pub/sub demo package.</description>
  <maintainer email="andrew@example.com">Andrew Lockwood</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
</package>
EOF

# setup.py
cat > "$PKG_DIR/setup.py" << 'EOF'
from setuptools import setup

package_name = 'basics_pubsub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Lockwood',
    maintainer_email='andrew@example.com',
    description='Simple ROS2 pub/sub demo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_pub = basics_pubsub.simple_pub:main',
            'simple_sub = basics_pubsub.simple_sub:main',
        ],
    },
)
EOF

# setup.cfg
cat > "$PKG_DIR/setup.cfg" << 'EOF'
[develop]
script-dir=$base/lib/basics_pubsub
[install]
install-scripts=$base/lib/basics_pubsub
EOF

# resource marker
echo "basics_pubsub" > "$PKG_DIR/resource/basics_pubsub"

# __init__.py
cat > "$PKG_DIR/$PKG_NAME/__init__.py" << 'EOF'
# empty
EOF

# simple_pub.py
cat > "$PKG_DIR/$PKG_NAME/simple_pub.py" << 'EOF'
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# simple_sub.py
cat > "$PKG_DIR/$PKG_NAME/simple_sub.py" << 'EOF'
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
EOF

# launch file
cat > "$PKG_DIR/launch/demo.launch.py" << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='basics_pubsub', executable='simple_pub', output='screen'),
        Node(package='basics_pubsub', executable='simple_sub', output='screen'),
    ])
EOF

# Top-level colcon workspace metadata (optional, but nice to have)
mkdir -p "$WS/src"

# Build instruction hint
cat > "$ROS2_ROOT/README.md" << 'EOF'
# robotics-playground

Minimal ROS2 Humble workspace with a Python pub/sub demo.


## Quick start
1. Install ROS2 Humble on Ubuntu 22.04. Source ROS2:
```bash
source /opt/ros/humble/setup.bash