#include "pentapod_imu/pentapod_imu.hpp"
#include <thread>
#include <chrono>

namespace penta_pod_imu
{

PentapodIMU::PentapodIMU(rclcpp::Node::SharedPtr node) : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "Pentapod IMU Node Started");
    declare_parameters();
    load_parameters();
    imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg_->header.frame_id = imu_params_.imu_frame_id;
    RCLCPP_INFO(node_->get_logger(), "IMU specified port is %s", imu_params_.port_name.c_str());
    imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    imu_serial_api_ = std::make_shared<ImuSerialApi>(imu_params_.port_name);

    kp_subscription_ = node_->create_subscription<Float32>(
        "imu/set_kp",
        10,
        [this](const Float32::SharedPtr msg) {
            imu_params_.kp = msg->data;
            if (imu_serial_api_ && imu_serial_api_->is_connected()) {
                RCLCPP_INFO(node_->get_logger(), "Setting IMU Kp to %.3f", imu_params_.kp);
                imu_serial_api_->write_imu_kp(imu_params_.kp);
            }
        }
    );

    ki_subscription_ = node_->create_subscription<Float32>(
        "imu/set_ki",
        10,
        [this](const Float32::SharedPtr msg) {
            imu_params_.ki = msg->data;
            if (imu_serial_api_ && imu_serial_api_->is_connected()) {
                RCLCPP_INFO(node_->get_logger(), "Setting IMU Ki to %.3f", imu_params_.ki);
                imu_serial_api_->write_imu_ki(imu_params_.ki);
            }
        }
    );

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
            delay(100);
            imu_serial_api_->write_imu_kp(imu_params_.kp);
            delay(100);
            imu_serial_api_->write_imu_ki(imu_params_.ki);
            delay(100);
            return true;
        }
        delay(750);
    }
    return false;
}

void PentapodIMU::declare_parameters() {
    node_->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    node_->declare_parameter<std::string>("imu_frame_id", "imu_link");
    node_->declare_parameter<float>("kp", 0.25);
    node_->declare_parameter<float>("ki", 0.0);
}

void PentapodIMU::load_parameters() {
    imu_params_.port_name = node_->get_parameter("port_name").as_string();
    RCLCPP_INFO(node_->get_logger(), "Loaded port_name: %s", imu_params_.port_name.c_str());
    imu_params_.imu_frame_id = node_->get_parameter("imu_frame_id").as_string();
    RCLCPP_INFO(node_->get_logger(), "Loaded imu_frame_id: %s", imu_params_.imu_frame_id.c_str());
    imu_params_.kp = node_->get_parameter("kp").as_double();
    RCLCPP_INFO(node_->get_logger(), "Loaded kp: %.3f", imu_params_.kp);
    imu_params_.ki = node_->get_parameter("ki").as_double();
    RCLCPP_INFO(node_->get_logger(), "Loaded ki: %.3f", imu_params_.ki);
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
    if (imu_serial_api_->readImuData(imu_msg_)) {
        imu_msg_->header.stamp = node_->now();
        imu_publisher_->publish(*imu_msg_);
    }
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Publishing IMU data: Orientation [%.3f, %.3f, %.3f, %.3f], Angular Velocity [%.3f, %.3f, %.3f], Linear Acceleration [%.3f, %.3f, %.3f]",
        imu_msg_->orientation.w,
        imu_msg_->orientation.x,
        imu_msg_->orientation.y,
        imu_msg_->orientation.z,
        imu_msg_->angular_velocity.x,
        imu_msg_->angular_velocity.y,
        imu_msg_->angular_velocity.z,
        imu_msg_->linear_acceleration.x,
        imu_msg_->linear_acceleration.y,
        imu_msg_->linear_acceleration.z
    );
}

} // namespace penta_pod_imu