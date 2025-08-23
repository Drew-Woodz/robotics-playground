#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class SimplePub : public rclcpp::Node {
public:
  SimplePub() : Node("simple_pub_cpp"), count_(0) {
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter_cpp", 10);
    timer_ = this->create_wall_timer(500ms, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "hello " + std::to_string(count_++);
      pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "sent: %s", msg.data.c_str());
    });
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}