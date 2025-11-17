#include "imu_stabilization/imu_stabilization.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("imu_stabilization_node");
  auto imu_stabilizer = std::make_shared<pentapod::imu::stabilizer::ImuStabilizer>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
