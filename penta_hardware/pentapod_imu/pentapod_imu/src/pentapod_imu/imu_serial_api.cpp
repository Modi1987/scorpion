#include "pentapod_imu/imu_serial_api.hpp"

namespace penta_pod_imu
{

ImuSerialApi::ImuSerialApi(std::string port_name, int baud_rate)
    : port_name_(port_name), baud_rate_(baud_rate)
{
}

ImuSerialApi::~ImuSerialApi()
{
    if (is_connected()) {
        disconnect();
    }
}

bool ImuSerialApi::disconnect()
{
    if (is_connected_) {
        close(fd_);
        is_connected_ = false;
    }
    std::cout << "ImuSerialApi disconnect called, serial port: " << port_name_ << " closed." << std::endl;
    return true;
}


bool ImuSerialApi::connect()
{
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "Error opening serial port " << port_name_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(fd_);
        return false;
    }

    // Configure serial port
    cfsetospeed(&tty, baud_rate_);
    cfsetispeed(&tty, baud_rate_);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // non-blocking read
    tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD);          // no parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // no hardware flow control

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd_);
        return false;
    }

    is_connected_ = true;
    std::cout << "ImuSerialApi connected to serial port: " << port_name_ << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for the connection to stabilize
    return true;
}

bool ImuSerialApi::is_connected() const
{
    return is_connected_;
}

bool ImuSerialApi::write_imu_kp(float kp) const
{
    if (!is_connected_) {
        std::cerr << "Not connected to serial port" << std::endl;
        return false;
    }
    std::string command = "Kp" + std::to_string(kp) + static_cast<char>(10);
    std::cout << "Writing Kp command: " << command;
    ssize_t bytes_written = write(fd_, command.c_str(), command.size());
    if (bytes_written < 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool ImuSerialApi::write_imu_ki(float ki) const
{
    if (!is_connected_) {
        std::cerr << "Not connected to serial port" << std::endl;
        return false;
    }
    std::string command = "Ki" + std::to_string(ki) + static_cast<char>(10);
    std::cout << "Writing Ki command: " << command;
    ssize_t bytes_written = write(fd_, command.c_str(), command.size());
    if (bytes_written < 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool ImuSerialApi::update()
{
    if (!is_connected_) {
        std::cerr << "Not connected to serial port" << std::endl;
        return false;
    }
    
    auto bytes_read = read(fd_, read_buffer_, buffer_size_);
    if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            disconnect();
        }
        return false;
    }

    constexpr float factor = 1000.0;

    for (ssize_t i = 0; i < bytes_read; ++i) {
        char chr = read_buffer_[i];

        if ((chr >= 'a') && (chr <= 'j')) {
            index = chr - 'a';
            measurement_array_[index] = 0;
            sign = 1;
        } else if (chr == '-') {
            sign = -1;
        } else if ((chr >= '0') && (chr <= '9')) {
            measurement_array_[index] = measurement_array_[index] * 10;
            measurement_array_[index] = measurement_array_[index] +  sign * (chr - '0');
        } else if ( chr == char(13) || chr == char(10)) { // flush data on '\r' or '\n'
            index = 0;
            sign = 1;
            // rotation quaternion
            qw_ = measurement_array_[0] / factor;
            q_vec_[0] = measurement_array_[1] / factor;
            q_vec_[1] = measurement_array_[2] / factor;
            q_vec_[2] = measurement_array_[3] / factor;
            // angular velocity rad/sec
            gyro_[0] = measurement_array_[4] / factor;
            gyro_[1] = measurement_array_[5] / factor;
            gyro_[2] = measurement_array_[6] / factor;
            // gravity normalized to gravity acceleration
            gravity_[0] = measurement_array_[7] / factor;
            gravity_[1] = measurement_array_[8] / factor;
            gravity_[2] = measurement_array_[9] / factor;
            new_measruement_ready_ = true;
        } else if (chr == ' ') {
            /* This charecter is used to make it more readable when printing */
        } else {
            std::cerr << "Unexpected character received: " << chr << std::endl;
        }
    }
    return true;
}

bool ImuSerialApi::readImuData(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    if (!new_measruement_ready_) return false;
    new_measruement_ready_ = false;
    imu_msg->orientation.w = qw_;
    imu_msg->orientation.x = q_vec_[0];
    imu_msg->orientation.y = q_vec_[1];
    imu_msg->orientation.z = q_vec_[2];

    imu_msg->angular_velocity.x = gyro_[0];
    imu_msg->angular_velocity.y = gyro_[1];
    imu_msg->angular_velocity.z = gyro_[2];

    constexpr float GRAVITY_MS2 = 9.80665f;
    imu_msg->linear_acceleration.x = GRAVITY_MS2 * gravity_[0];
    imu_msg->linear_acceleration.y = GRAVITY_MS2 * gravity_[1];
    imu_msg->linear_acceleration.z = GRAVITY_MS2 * gravity_[2];
    return true; // Return true if successful
}

} // namespace penta_pod_imu