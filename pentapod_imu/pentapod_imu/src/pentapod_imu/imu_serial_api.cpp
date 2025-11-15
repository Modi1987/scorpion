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
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for the connection to stabilize
    return true;
}

bool ImuSerialApi::is_connected() const
{
    return is_connected_;
}

int index = -1;
int sign = +1;
bool ImuSerialApi::update()
{
    auto packet_size = 1;
    int byte_count = 0;
    constexpr float factor = 1000.0;
    char chr;
    while (true) {
        auto bytesRead = read( fd_, &chr, packet_size);
        if (bytesRead <= 0) {
            return byte_count > 0;
        } else {
            byte_count += bytesRead;
        }
        if ((chr >= 'a') && (chr <= 'g')) {
            index = chr - 'a';
            measruement_array_[index] = 0;
            sign = 1;
            continue;
        } else if (chr == '-') {
            sign = -1;
        } else if ((chr >= '0') && (chr <= '9')) {
            measruement_array_[index] = measruement_array_[index] * 10;
            measruement_array_[index] = measruement_array_[index] +  sign * (chr - '0');
        } else if ( chr == char(13) || chr == char(10)) { // flush data on '\r' or '\n'
            index = 0;
            sign = 1;
            // rotation quaternion
            qw_ = measruement_array_[0] / factor;
            q_vec_[0] = measruement_array_[1] / factor;
            q_vec_[1] = measruement_array_[2] / factor;
            q_vec_[2] = measruement_array_[3] / factor;
            // angular velocity rad/sec
            gyro_[0] = measruement_array_[4] / factor;
            gyro_[1] = measruement_array_[5] / factor;
            gyro_[2] = measruement_array_[6] / factor;
            // gravity normalized to gravity acceleration
            gravity_[0] = measruement_array_[4] / factor;
            gravity_[1] = measruement_array_[5] / factor;
            gravity_[2] = measruement_array_[6] / factor;
        }

    }
}

bool ImuSerialApi::readImuData(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
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