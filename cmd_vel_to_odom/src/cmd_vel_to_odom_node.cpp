#include "cmd_vel_to_odom/cmd_vel_to_odom.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  polypod::cmd_vel_to_odom::CmdVelToOdom().spin();
  rclcpp::shutdown();
  return 0;
}
