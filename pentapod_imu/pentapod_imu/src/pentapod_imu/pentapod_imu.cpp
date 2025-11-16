#include "pentapod_imu/pentapod_imu.hpp"
#include <thread>
#include <chrono>

namespace penta_pod_imu
{

PentapodIMU::PentapodIMU(rclcpp::Node::SharedPtr node) : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "Pentapod IMU Node Started");
    node_->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    port_name_ = node_->get_parameter("port_name").as_string();
    RCLCPP_INFO(node_->get_logger(), "IMU specified port is %s", port_name_.c_str());
    imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    imu_serial_api_ = std::make_shared<ImuSerialApi>(port_name_);

    // timer_ = node_->create_wall_timer(
    //     std::chrono::milliseconds(40),
    //     std::bind(&PentapodIMU::readDataPublishCallback, this)
    // );
}

bool PentapodIMU::connect() {
    constexpr int max_retries = 5;
    for (int i = 0; i < max_retries; i++) {
        if (!imu_serial_api_->connect()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open IMU serial port.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully connected to IMU on port %s", port_name_.c_str());
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
    }
    return false;
}

bool PentapodIMU::disconnect()
{
    if (imu_serial_api_) {
        return imu_serial_api_->disconnect();
    }
    return true;
}

void PentapodIMU::readDataPublishCallback() {
    if (!imu_serial_api_ || !imu_serial_api_->is_connected() ){
        RCLCPP_ERROR(node_->get_logger(), "Could not connect to IMU on serial port %s", port_name_.c_str());
        return;
    } 
    imu_serial_api_->update();
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    if (imu_serial_api_->readImuData(imu_msg)) {
        imu_publisher_->publish(*imu_msg);
    }
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Publishing IMU data: Orientation [%.3f, %.3f, %.3f, %.3f], Angular Velocity [%.3f, %.3f, %.3f], Linear Acceleration [%.3f, %.3f, %.3f]",
        imu_msg->orientation.w,
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z,
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z
    );
}

} // namespace penta_pod_imu