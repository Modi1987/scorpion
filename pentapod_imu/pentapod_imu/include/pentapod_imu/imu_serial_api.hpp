#ifndef IMU_SERIAL_API_HPP
#define IMU_SERIAL_API_HPP

#include "sensor_msgs/msg/imu.hpp"
#include <iostream>
#include <string>
#include <stdexcept>
#include <fcntl.h>      // open()
#include <termios.h>    // termios, tcgetattr(), tcsetattr()
#include <unistd.h>     // read(), write(), close()
#include <cstring>      // memset
#include <vector>
#include <thread>
#include <chrono>

namespace penta_pod_imu
{

class ImuSerialApi
{
public:
    ImuSerialApi(std::string port_name = "/dev/ttyUSB0", int baud_rate = B115200);
    ~ImuSerialApi();
    bool connect();
    bool disconnect();
    bool is_connected() const;

    bool readImuData(sensor_msgs::msg::Imu::SharedPtr imu_msg);
    bool update();

private:
    std::string port_name_;
    int baud_rate_;
    int fd_; // File descriptor
    bool is_connected_;
    // vars used to parse incoming data
    static constexpr size_t buffer_size_ = 256;
    char read_buffer_[buffer_size_];
    int index = -1;
    int sign = +1;
    int measurement_array_[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // IMU data
    float q_vec_[3] = {0.0f, 0.0f, 0.0f}; // Quaternion representing imaginary components
    float qw_ = 0.0f; // Quaternion w component
    float gyro_[3] = {0.0f, 0.0f, 0.0f}; // Gyroscope measurements
    float gravity_[3] = {0.0f, 0.0f, 0.0f}; // Gravity vector
};

} // namespace penta_pod_imu

#endif // IMU_SERIAL_API_HPP
