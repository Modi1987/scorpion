#ifndef PENTAPOD_IMU_HPP
#define PENTAPOD_IMU_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pentapod_imu/imu_serial_api.hpp"
#include "std_msgs/msg/float32.hpp"

namespace penta_pod_imu
{

using std_msgs::msg::Float32;

class PentapodIMU
{
public:
    PentapodIMU(rclcpp::Node::SharedPtr node);
    ~PentapodIMU() {};
    void readDataPublishCallback();
    bool disconnect();
    bool connect();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string port_name_;
    std::shared_ptr<ImuSerialApi> imu_serial_api_;
    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_;

    rclcpp::Subscription<Float32>::SharedPtr kp_subscription_;
    rclcpp::Subscription<Float32>::SharedPtr ki_subscription_;

    void declare_parameters();
    void load_parameters();

    void delay(int milliseconds) {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }

    struct ImuParams {
        std::string port_name;
        std::string imu_frame_id = "imu_link";
        float kp = 0.25;
        float ki = 0.0;
    } imu_params_;
};

} // namespace penta_pod_imu

#endif // PENTAPOD_IMU_HPP
