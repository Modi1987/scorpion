#include "pca9685_ros2_control/pca9685_ros2_api.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <stdexcept>
#include <iostream>

// Addresses
#define MODE1_REG_ADDRESS 0x00
#define PRESCALE 0xFE
#define LED0_ON_L_ADDRESS 0x06

// Flags for MODE1 register
#define MODE1_AI 0x20      // Auto-Increment
#define MODE1_RESTART 0x80 // Restart enabled

#define PWM_FREQUENCY_HZ 250.0 // 200Hz for high end servos 50 for all servos

MyPCA9685::MyPCA9685(const std::vector<MotorParams>& motor_params, const std::string& i2c_device, int address)
    : motor_params_(motor_params), device_path_(i2c_device), i2c_address_(address), i2c_file_(-1) {
        // initialize static data
        number_of_connected_motors_ = static_cast<int>(motor_params.size());
        i2c_buffer_.data[0] = LED0_ON_L_ADDRESS; // Start address for the first channel
        i2c_buffer_.actual_data_size = 1 + number_of_connected_motors_ * 4;
        for (int i=0; i < number_of_connected_motors_; i++) {
            std::cout << "[MyPca9685.cpp] motor params [" << i
                      << "]: " << motor_params[i].name
                      << " pwm_channel: " << motor_params[i].pwm_channel
                      << " min_pulse: " << motor_params[i].min_pulse
                      << " max_pulse: " << motor_params[i].max_pulse
                      << " min_angle: " << motor_params[i].min_angle
                      << " max_angle: " << motor_params[i].max_angle
                      << std::endl;
        }
    }

MyPCA9685::~MyPCA9685() {
    if (i2c_file_ >= 0) this->close();
}

bool MyPCA9685::init() {
    i2c_file_ = open(device_path_.c_str(), O_RDWR);
    if (i2c_file_ < 0) {
        perror("Failed to open I2C device");
        return false;
    }

    if (ioctl(i2c_file_, I2C_SLAVE, i2c_address_) < 0) {
        perror("Failed to acquire bus access");
        return false;
    }

    writeRegister(MODE1_REG_ADDRESS, MODE1_RESTART);
    usleep(5000); // Wait for the device to initialize
    setPWMFreq(PWM_FREQUENCY_HZ); // 200Hz for high end servos 50 for all servos
    // initiate all motors to 0 degrees
    for (int i = 0; i < number_of_connected_motors_; ++i) {
        setMotorCommand(i, 0.0f); // Set all motors to 0 degrees
    }
    return true;
}

void MyPCA9685::reset() {
  writeRegister(MODE1_REG_ADDRESS, MODE1_RESTART);
  usleep(5000); // Wait for the device to reset
}

void MyPCA9685::writeRegister(unsigned char reg, unsigned char value) {
    unsigned char buffer[2] = { reg, value };
    if (write(i2c_file_, buffer, 2) != 2) {
        perror("I2C: Failed to write to device");
    } else {
        std::cout << "I2C: Successfully wrote to register " << static_cast<int>(reg) << " with value " << static_cast<int>(value) << std::endl;
    }
}

unsigned char MyPCA9685::readRegister(unsigned char reg) {
    if (write(i2c_file_, &reg, 1) != 1) {
        perror("I2C: Failed to write register address");
        return 0;
    }

    unsigned char value;
    if (read(i2c_file_, &value, 1) != 1) {
        perror("I2C: Failed to read register");
        return 0;
    }

    return value;
}

void MyPCA9685::setPWMFreq(float freqHz) {
    constexpr float reference_colck_speed = 25000000.0; // 25MHz
    float prescaleval = reference_colck_speed;
    prescaleval /= 4096.0;
    prescaleval /= freqHz;
    prescaleval -= 1.0;
    std::cout << "Prescale value: " << prescaleval << std::endl;
    if (prescaleval < 3 || prescaleval > 255) {
        throw std::runtime_error("Prescale value out of range (3-255)");
    }
    unsigned char prescale = static_cast<unsigned char>(std::floor(prescaleval + 0.5));

    unsigned char oldmode = readRegister(MODE1_REG_ADDRESS);
    unsigned char sleep = (oldmode & 0x7F) | 0x10; // sleep
    writeRegister(MODE1_REG_ADDRESS, sleep);
    writeRegister(PRESCALE, prescale);
    writeRegister(MODE1_REG_ADDRESS, oldmode);
    usleep(5000);
    
    writeRegister(MODE1_REG_ADDRESS, oldmode | MODE1_RESTART | MODE1_AI); // Restart and Auto-Increment enabled
}

void MyPCA9685::setPWM(int channel, int on, int off) {
    auto temp = LED0_ON_L_ADDRESS + 4 * channel;
    if (temp > 255) {
        std::cerr << "Invalid address with value " << temp << " which is more than 255";
    }
    unsigned char start_mem_address = static_cast<unsigned char>(temp);
    unsigned char buffer[5] = {
        start_mem_address,
        static_cast<unsigned char>(on & 0xFF),
        static_cast<unsigned char>(on >> 8),
        static_cast<unsigned char>(off & 0xFF),
        static_cast<unsigned char>(off >> 8)
    };
    if (write(i2c_file_, buffer, 5) != 5) {
        throw std::runtime_error("I2C: Failed to set PWM");
    }
}

int MyPCA9685::setMotorCommand(int channel, float setpoint_deg) {
    if (channel < 0 || channel >= number_of_connected_motors_) {
        std::cerr << "Invalid channel index: " << channel << std::endl;
        return -1;
    }

    float min_angle = motor_params_[channel].min_angle;
    float max_angle = motor_params_[channel].max_angle;

    if (setpoint_deg < min_angle) setpoint_deg = min_angle;
    if (setpoint_deg > max_angle) setpoint_deg = max_angle;

    float angle_span = max_angle - min_angle;
    float min_pulse = motor_params_[channel].min_pulse;
    float max_pulse = motor_params_[channel].max_pulse;
    float pulse_span = max_pulse - min_pulse;

    float pulseMs = min_pulse + pulse_span*((setpoint_deg - min_angle) / angle_span);
    double period = 1000.0 / PWM_FREQUENCY_HZ; // in ms
    int pulseTicks = static_cast<int>((pulseMs / period) * 4096); // period = 5ms for 200Hz
    // update servo ticks when successful
    if (pulseTicks < 0 || pulseTicks > 4095) {
        std::cerr << "Pulse ticks out of range for channel " << channel << ": " << pulseTicks << std::endl;
        return -1;
    }
    servo_command_ticks_[channel] = pulseTicks;
    return pulseTicks;
}

void MyPCA9685::commandOneMotorAngleDegree(int channel, float setpoint_deg) {
    auto pulseTicks = setMotorCommand(channel, setpoint_deg);
    if (pulseTicks < 0) {
        std::cerr << "Failed to get pulse ticks for channel " << channel << " and angle " << setpoint_deg << std::endl;
        return;
    }    
    setPWM(channel, 0, pulseTicks);
}

void MyPCA9685::flushInternalCommands2Motors() {

    for(int i = 0; i < number_of_connected_motors_; ++i) {
        i2c_buffer_.data[1 + 4 * i] = 0;
        i2c_buffer_.data[2 + 4 * i] = 0; 
        i2c_buffer_.data[3 + 4 * i] = static_cast<unsigned char>(servo_command_ticks_[i] & 0xFF);
        i2c_buffer_.data[4 + 4 * i] = static_cast<unsigned char>(servo_command_ticks_[i] >> 8);    
    }

    if (write(i2c_file_, i2c_buffer_.data, i2c_buffer_.actual_data_size) != i2c_buffer_.actual_data_size) {
        std::cerr << "Failed to write all PWM commands to PCA9685" << std::endl;
    }
}

