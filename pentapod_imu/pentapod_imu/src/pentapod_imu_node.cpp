#include "pentapod_imu/pentapod_imu.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("imu_node");
  auto imu_obejct = std::make_shared<penta_pod_imu::PentapodIMU>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}