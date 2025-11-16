
#ifndef RTROBOT_SERIAL_API_HPP
#define RTROBOT_SERIAL_API_HPP

#include <iostream>
#include <string>
#include <stdexcept>
#include <fcntl.h>      // open()
#include <termios.h>    // termios, tcgetattr(), tcsetattr()
#include <unistd.h>     // read(), write(), close()
#include <cstring>      // memset
#include <vector>

#define DEFAULT_BAUD_RATE B115200

class RtRobotSerial {
private:
    int fd; // File descriptor
    std::string portName;
    int baudRate;
    bool isConnected;
    std::string trajectoryParam;
    std::vector<int> lastPwmCommand;

public:
    RtRobotSerial(
        const std::string& portName = "/dev/ttyUSB0",
        int baudRate = DEFAULT_BAUD_RATE,
        const std::string& trajectory_param = ""
    ) : fd(-1), portName(portName), baudRate(baudRate), isConnected(false), trajectoryParam(trajectory_param) {
        lastPwmCommand = std::vector<int>(32, -1);
    }
    bool connect();
    void disconnect();
    ssize_t writeData(const std::vector<int>& data, int update_time_ms = -1);

    std::string getPortName() const { return portName; }
    bool connected() const { return isConnected; }

    ~RtRobotSerial() {
        disconnect();
    }
};

#endif // RTROBOT_SERIAL_API_HPP
