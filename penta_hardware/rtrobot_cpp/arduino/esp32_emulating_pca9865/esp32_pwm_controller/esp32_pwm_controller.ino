// Copyright: Mohammad Safeea, 2025-Nov-07
// ESP32 PWM Controller - Replacement for PCA9685
// This program emulates PCA9685 behavior with 16 PWM channels using ESP32 LEDC
// Works with existing ROS2 PCA9685 control API

//-- Libraries Included --------------------------------------------------------------
#include <Wire.h>
//------------------------------------------------------------------------------------

// I2C Configuration
#define SDA_PIN 21  // Default SDA pin for ESP32
#define SCL_PIN 22  // Default SCL pin for ESP32
#define I2C_ADDRESS 0x40  // Default PCA9685 I2C address

// PCA9685 Register Addresses (for compatibility)
#define MODE1_REG_ADDRESS 0x00
#define PRESCALE 0xFE
#define LED0_ON_L_ADDRESS 0x06

// PCA9685 MODE1 register flags
#define MODE1_AI 0x20      // Auto-Increment
#define MODE1_RESTART 0x80 // Restart enabled

// PWM Configuration
#define PWM_CHANNELS 16
#define PWM_FREQUENCY 200  // 200Hz frequency (same as PCA9685 API)
#define PWM_RESOLUTION 12  // 12-bit resolution (4096 levels like PCA9685)
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE

// ESP32 GPIO pins for PWM output (16 channels)
const int pwm_pins[PWM_CHANNELS] = {
  2,   // Channel 0  - GPIO2
  4,   // Channel 1  - GPIO4
  5,   // Channel 2  - GPIO5
  12,  // Channel 3  - GPIO12
  13,  // Channel 4  - GPIO13
  14,  // Channel 5  - GPIO14
  15,  // Channel 6  - GPIO15
  16,  // Channel 7  - GPIO16
  17,  // Channel 8  - GPIO17
  18,  // Channel 9  - GPIO18
  19,  // Channel 10 - GPIO19
  23,  // Channel 11 - GPIO23
  25,  // Channel 12 - GPIO25
  26,  // Channel 13 - GPIO26
  27,  // Channel 14 - GPIO27
  32   // Channel 15 - GPIO32
};

// Global variables
uint8_t mode1_register = 0;
uint8_t prescale_register = 0;
uint16_t pwm_values[PWM_CHANNELS][2]; // [channel][on/off] values
bool i2c_initialized = false;

//====================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 PWM Controller Starting...");
  
  // Initialize PWM channels
  initPWMChannels();
  
  // Initialize I2C as slave
  initI2CSlave();
  
  // Set all channels to neutral position (1500μs pulse width)
  for (int i = 0; i < PWM_CHANNELS; i++) {
    setPWMDutyFromTicks(i, 0, 307); // ~1500μs at 200Hz
  }
  
  Serial.println("ESP32 PWM Controller Ready!");
}

void loop() {
  // Main loop - I2C handling is done via interrupts
  delay(10);
}

//====================================================================================
// PWM Channel Initialization
//====================================================================================

void initPWMChannels() {
  // Configure LEDC timer
  ledcSetup(LEDC_TIMER, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Setup PWM channels
  for (int i = 0; i < PWM_CHANNELS; i++) {
    ledcSetup(i, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(pwm_pins[i], i);
    
    // Initialize PWM values
    pwm_values[i][0] = 0;    // ON time (always 0 for servo control)
    pwm_values[i][1] = 0;    // OFF time (duty cycle)
    
    Serial.printf("PWM Channel %d initialized on GPIO %d\n", i, pwm_pins[i]);
  }
}

//====================================================================================
// I2C Slave Implementation
//====================================================================================

void initI2CSlave() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  i2c_initialized = true;
  Serial.printf("I2C Slave initialized at address 0x%02X\n", I2C_ADDRESS);
}

void onI2CReceive(int numBytes) {
  if (numBytes < 2) return;
  
  uint8_t reg_address = Wire.read();
  numBytes--;
  
  // Handle different register writes
  switch (reg_address) {
    case MODE1_REG_ADDRESS:
      if (numBytes >= 1) {
        mode1_register = Wire.read();
        handleMode1Register();
      }
      break;
      
    case PRESCALE:
      if (numBytes >= 1) {
        prescale_register = Wire.read();
        handlePrescaleRegister();
      }
      break;
      
    default:
      // Handle LED register writes (PWM channel control)
      if (reg_address >= LED0_ON_L_ADDRESS && reg_address <= (LED0_ON_L_ADDRESS + 4 * PWM_CHANNELS)) {
        handleLEDRegisterWrite(reg_address, numBytes);
      } else {
        // Consume remaining bytes
        while (Wire.available()) Wire.read();
      }
      break;
  }
}

void onI2CRequest() {
  // Handle register read requests
  Wire.write(0x00); // Default response
}

//====================================================================================
// Register Handlers
//====================================================================================

void handleMode1Register() {
  Serial.printf("MODE1 register set to 0x%02X\n", mode1_register);
  
  if (mode1_register & MODE1_RESTART) {
    // Handle restart
    Serial.println("Restart command received");
  }
}

void handlePrescaleRegister() {
  Serial.printf("PRESCALE register set to 0x%02X\n", prescale_register);
  
  // Calculate new frequency from prescale value
  // Formula: freq = 25MHz / (4096 * (prescale + 1))
  float new_freq = 25000000.0 / (4096.0 * (prescale_register + 1));
  
  Serial.printf("New PWM frequency: %.2f Hz\n", new_freq);
  
  // Update all PWM channels with new frequency
  for (int i = 0; i < PWM_CHANNELS; i++) {
    ledcSetup(i, new_freq, PWM_RESOLUTION);
  }
}

void handleLEDRegisterWrite(uint8_t start_reg, int numBytes) {
  int channel = (start_reg - LED0_ON_L_ADDRESS) / 4;
  int reg_offset = (start_reg - LED0_ON_L_ADDRESS) % 4;
  
  if (channel >= PWM_CHANNELS) {
    // Consume remaining bytes
    while (Wire.available()) Wire.read();
    return;
  }
  
  // Read the PWM values
  uint8_t data[4] = {0, 0, 0, 0};
  int bytes_to_read = min(numBytes, 4 - reg_offset);
  
  for (int i = 0; i < bytes_to_read && Wire.available(); i++) {
    data[reg_offset + i] = Wire.read();
  }
  
  // Parse ON and OFF values
  uint16_t on_value = data[0] | (data[1] << 8);
  uint16_t off_value = data[2] | (data[3] << 8);
  
  // Update PWM channel
  setPWMDutyFromTicks(channel, on_value, off_value);
  
  // Consume any remaining bytes
  while (Wire.available()) Wire.read();
}

//====================================================================================
// PWM Control Functions
//====================================================================================

void setPWMDutyFromTicks(int channel, uint16_t on_ticks, uint16_t off_ticks) {
  if (channel < 0 || channel >= PWM_CHANNELS) return;
  
  // Store values
  pwm_values[channel][0] = on_ticks;
  pwm_values[channel][1] = off_ticks;
  
  // Calculate duty cycle
  // PCA9685 uses 4096 ticks per cycle, ESP32 LEDC also uses 4096 with 12-bit resolution
  uint32_t duty_cycle;
  
  if (off_ticks == 0) {
    duty_cycle = 0; // Fully OFF
  } else if (off_ticks >= 4095) {
    duty_cycle = 4095; // Fully ON
  } else {
    // Calculate actual duty cycle considering on_ticks and off_ticks
    if (on_ticks == 0) {
      duty_cycle = off_ticks;
    } else {
      // More complex case where on_ticks != 0
      if (off_ticks > on_ticks) {
        duty_cycle = off_ticks - on_ticks;
      } else {
        duty_cycle = (4096 - on_ticks) + off_ticks;
      }
    }
  }
  
  // Apply duty cycle to LEDC
  ledcWrite(channel, duty_cycle);
  
  Serial.printf("Channel %d: ON=%d, OFF=%d, Duty=%d\n", channel, on_ticks, off_ticks, duty_cycle);
}

//====================================================================================
// Utility Functions
//====================================================================================

void setPWMPulseWidth(int channel, float pulse_width_us) {
  if (channel < 0 || channel >= PWM_CHANNELS) return;
  
  // Convert microseconds to ticks (assuming 200Hz = 5ms period)
  float period_us = 1000000.0 / PWM_FREQUENCY;  // Period in microseconds
  uint16_t ticks = (uint16_t)((pulse_width_us / period_us) * 4096.0);
  
  setPWMDutyFromTicks(channel, 0, ticks);
}

void setServoPulse(int channel, float pulse_width_us) {
  setPWMPulseWidth(channel, pulse_width_us);
}

//====================================================================================
// Debug Functions
//====================================================================================

void printStatus() {
  Serial.println("=== ESP32 PWM Controller Status ===");
  Serial.printf("I2C Address: 0x%02X\n", I2C_ADDRESS);
  Serial.printf("PWM Frequency: %d Hz\n", PWM_FREQUENCY);
  Serial.printf("PWM Resolution: %d bits\n", PWM_RESOLUTION);
  Serial.printf("Active Channels: %d\n", PWM_CHANNELS);
  
  Serial.println("\nChannel Status:");
  for (int i = 0; i < PWM_CHANNELS; i++) {
    Serial.printf("Ch%2d (GPIO%2d): ON=%4d, OFF=%4d\n", 
                  i, pwm_pins[i], pwm_values[i][0], pwm_values[i][1]);
  }
  Serial.println("=====================================");
}