#include "rtrobot_ros2_control/rtrobot_serial_api.hpp"
#include <memory>
#include <vector>
#include <iostream>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds
#include <termios.h> // For B115200

int main(int argc, char **argv) {
    std::string port = "/dev/ttyACM0";
    int baudRate = B115200;
    std::string trajectory_param = "T10D800";

    // Create an instance of RtRobotSerial
    std::shared_ptr<RtRobotSerial> rtrobot = std::make_shared<RtRobotSerial>(port, baudRate, trajectory_param);

    // Attempt to connect to the robot
    if (rtrobot->connect()) {
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Sleep for 2 seconds
        std::cout << "Connected to RtRobot on " << port << std::endl;

        // Define PWM commands
        std::vector<int> pwm_start = {1500, 1600, 1700}; // Example PWM commands in microseconds
        std::vector<int> pwm_end = {1000, 1000, 1000};

        // Lambda function to send PWM commands
        auto command_motors = [](std::shared_ptr<RtRobotSerial> rtrobot, const std::vector<int>& pwm_commands) {
            ssize_t bytes_written = rtrobot->writeData(pwm_commands);
            if (bytes_written > 0) {
                std::cout << "Wrote " << bytes_written << " bytes to RtRobot." << std::endl;
            } else {
                std::cerr << "Failed to write data to RtRobot." << std::endl;
            }
        };

        // Send commands in a loop
        for (int i = 0; i < 3; ++i) {
            command_motors(rtrobot, pwm_start);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            command_motors(rtrobot, pwm_end);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        // Disconnect from the robot
        rtrobot->disconnect();
    } else {
        std::cerr << "Failed to connect to RtRobot." << std::endl;
    }

    return 0;
}