#ifndef PCA9685_ROS2_API_HPP_
#define PCA9685_ROS2_API_HPP_

#include "pca9685_interfaces/msg/pca_channel_params.hpp"
#include <memory>
#include <vector>
#include <string>
#include <unistd.h> 
#include <fcntl.h>
#include <cstdint>

using MotorParams = pca9685_interfaces::msg::PcaChannelParams;

class MyPCA9685 {
public:
    MyPCA9685(const std::vector<MotorParams>& motor_params, 
        const std::string& i2cDevice = "/dev/i2c-1", int address = 0x40);
    ~MyPCA9685();

    bool init();
    void setPWMFreq(float freqHz);
    void setPWM(int channel, int on, int off);
    void commandOneMotorAngleDegree(int channel, float angleDeg);
    int setMotorCommand(int channel, float setpoint_deg);
    void flushInternalCommands2Motors();
    void close() {
        if (i2c_file_ >= 0) {
            ::close(i2c_file_);
            i2c_file_ = -1;
        }
    }
    int get_numer_of_motors() const { return static_cast<int>(motor_params_.size()); }

private:
    std::vector<MotorParams> motor_params_;
    std::string device_path_;
    int i2c_address_;
    int i2c_file_;
    std::vector<int> servo_command_ticks_;
    
    void writeRegister(unsigned char reg, unsigned char value);
    void reset();
    unsigned char readRegister(unsigned char reg);

};

#endif
