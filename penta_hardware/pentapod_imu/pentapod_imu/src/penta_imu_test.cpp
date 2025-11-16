#include <iostream>
#include "pentapod_imu/imu_serial_api.hpp"
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    auto imu = std::make_shared<penta_pod_imu::ImuSerialApi>();
    imu->connect();

    while (true) {
        if (!imu->is_connected()) {
            std::cerr << "IMU not connected. Exiting." << std::endl;
            break;
        }
        imu->update();
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        if (imu->readImuData(imu_msg)) {
            std::cout << "Orientation: [" << imu_msg->orientation.w << ", "
                      << imu_msg->orientation.x << ", "
                      << imu_msg->orientation.y << ", "
                      << imu_msg->orientation.z << "]" << std::endl;
            std::cout << "Angular Velocity: [" << imu_msg->angular_velocity.x << ", "
                      << imu_msg->angular_velocity.y << ", "
                      << imu_msg->angular_velocity.z << "]" << std::endl;
            std::cout << "Linear Acceleration: [" << imu_msg->linear_acceleration.x << ", "
                      << imu_msg->linear_acceleration.y << ", "
                      << imu_msg->linear_acceleration.z << "]" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    imu->disconnect();
    return 0;
}