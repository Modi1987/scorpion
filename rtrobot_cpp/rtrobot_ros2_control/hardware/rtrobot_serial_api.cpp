#include "rtrobot_ros2_control/rtrobot_serial_api.hpp"
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds
#include <termios.h> // For B115200

bool RtRobotSerial::connect() {
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening serial port " << portName << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(fd);
        return false;
    }

    // Configure serial port
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

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

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return false;
    }

    isConnected = true;
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for the connection to stabilize
    return true;
}

void RtRobotSerial::disconnect() {
    if (isConnected) {
        close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for the disconnection to stabilize
        isConnected = false;
    }
}


ssize_t RtRobotSerial::writeData(const std::vector<int>& pwm_commands_micro_sec, int update_time_ms) {
    if (!isConnected) {
        std::cerr << "Serial port not connected.\n";
        return -1;
    }

    // Convert PWM commands to a format suitable for writing
    std::string data;
    int count = 0;
    for (const auto& pwm : pwm_commands_micro_sec) {
        if (pwm != lastPwmCommand[count]) {
            data += "#" + std::to_string(count+1) + "P" + std::to_string(pwm);
        }
        lastPwmCommand[count] = pwm;
        count++;
    }

    if (data.empty()) {
        // No changes to send
        return 0;
    }

    if (update_time_ms < 0) {
        data += trajectoryParam; // Default trajectory parameter
    } else {
        update_time_ms = update_time_ms + (update_time_ms / 10); // Add 10% margin
        data += "T" + std::to_string(update_time_ms) + "D0";
    }
    data += "\r\n"; // Add newline to indicate end of command

    ssize_t bytesWritten = write(fd, data.c_str(), data.size());
    if (bytesWritten < 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
    } else {
        std::cout << "Sent: " << data << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Wait for the data to be sent
    // char buffer[100];
    // ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1);
    // if (bytesRead > 0) {
    //     buffer[bytesRead] = '\0'; // Null-terminate the string
    //     std::cout << "Received: " << buffer << std::endl;
    // }
    return bytesWritten;
}

