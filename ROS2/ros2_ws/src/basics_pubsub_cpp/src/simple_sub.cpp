#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



class SimpleSub : public rclcpp::Node {
public:
  SimpleSub() : Node("simple_sub_cpp") {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter_cpp", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "heard: %s", msg->data.c_str());
      }
    );
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}