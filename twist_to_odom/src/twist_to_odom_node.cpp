#include "twist_to_odom/twist_to_odom.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  polypod::twist_to_odom::TwistToOdom().spin();
  rclcpp::shutdown();
  return 0;
}
