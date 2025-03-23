#include "gait_generator/base_tf_broadcaster.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  penta_pod::kin::BaseTfBroadcaster().spin();
  rclcpp::shutdown();
  return 0;
}