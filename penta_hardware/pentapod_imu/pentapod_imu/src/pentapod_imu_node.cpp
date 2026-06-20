#include "pentapod_imu/pentapod_imu.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pentapod_imu_node");
  auto imu_object = std::make_shared<penta_pod_imu::PentapodIMU>(node);
  if (!imu_object->connect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to IMU. Exiting.");
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  try {
    while (rclcpp::ok()) {
      imu_object->readDataPublishCallback();
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main loop: %s", e.what());
    imu_object->disconnect();
  }
  rclcpp::shutdown();
  return 0;
}
