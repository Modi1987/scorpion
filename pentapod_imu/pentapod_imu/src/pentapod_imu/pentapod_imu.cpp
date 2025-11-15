#include "pentapod_imu/pentapod_imu.hpp"

namespace penta_pod_imu
{

PentapodIMU::PentapodIMU(rclcpp::Node::SharedPtr node) : node_(node)
{
    node_ = rclcpp::Node::make_shared("pentapod_imu_node");
    node_->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    std::string port_name = node_->get_parameter("port_name").as_string();
    imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    imu_serial_api_ = std::make_shared<ImuSerialApi>(port_name);
    if (!imu_serial_api_->connect()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open IMU serial port.");
    }
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PentapodIMU::timerCallback, this)
    );
}

void PentapodIMU::timerCallback() {
    imu_serial_api_->update();
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    if (imu_serial_api_->readImuData(imu_msg)) {
        imu_publisher_->publish(*imu_msg);
    }
}

} // namespace penta_pod_imu