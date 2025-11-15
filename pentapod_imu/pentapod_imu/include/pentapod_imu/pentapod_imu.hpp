#ifndef PENTAPOD_IMU_HPP
#define PENTAPOD_IMU_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pentapod_imu/imu_serial_api.hpp"

namespace penta_pod_imu
{

class PentapodIMU
{
public:
    PentapodIMU();
    ~PentapodIMU() {};

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<ImuSerialApi> imu_serial_api_;

    void timerCallback();
};

} // namespace penta_pod_imu

#endif // PENTAPOD_IMU_HPP
