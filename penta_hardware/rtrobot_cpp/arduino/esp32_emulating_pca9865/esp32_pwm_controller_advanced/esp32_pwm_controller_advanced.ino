// Copyright: Mohammad Safeea, 2025-Nov-07
// ESP32 High-Precision PWM Controller - PCA9685 Replacement
// Enhanced version with bulk updates and 1μs accuracy
// Compatible with existing ROS2 PCA9685 control API

//-- Libraries Included --------------------------------------------------------------
#include <Wire.h>
#include <driver/ledc.h>
#include <esp32-hal-ledc.h>
//------------------------------------------------------------------------------------

// I2C Configuration
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDRESS 0x40

// PCA9685 Register Compatibility
#define MODE1_REG_ADDRESS 0x00
#define PRESCALE 0xFE
#define LED0_ON_L_ADDRESS 0x06
#define MODE1_AI 0x20
#define MODE1_RESTART 0x80

// High Precision PWM Configuration
#define PWM_CHANNELS 16
#define DEFAULT_PWM_FREQUENCY 200.0  // Hz
#define PWM_RESOLUTION_BITS 16       // 16-bit for higher precision
#define PWM_MAX_VALUE 65535          // 2^16 - 1

// Timer configuration for high precision
#define LEDC_TIMER_NUM LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE

// ESP32 GPIO pins for PWM (choose pins carefully for your setup)
const int pwm_pins[PWM_CHANNELS] = {
  2, 4, 5, 12, 13, 14, 15, 16,     // Channels 0-7
  17, 18, 19, 23, 25, 26, 27, 32   // Channels 8-15
};

// PWM State Management
struct PWMChannel {
  uint16_t on_ticks;   // PCA9685 ON time (0-4095)
  uint16_t off_ticks;  // PCA9685 OFF time (0-4095) 
  uint32_t duty_cycle; // ESP32 duty cycle (0-65535)
  float pulse_width_us; // Actual pulse width in microseconds
  bool dirty;          // Needs update flag
};

// Global state
PWMChannel channels[PWM_CHANNELS];
float current_frequency = DEFAULT_PWM_FREQUENCY;
uint8_t mode1_register = 0;
uint8_t prescale_register = 0;
bool bulk_update_mode = false;
int bulk_update_count = 0;

//====================================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 High-Precision PWM Controller ===");
  Serial.println("PCA9685 Compatible Mode");
  
  initializePWMSystem();
  initializeI2CSlave();
  
  // Set all servos to neutral (1500μs)
  for (int i = 0; i < PWM_CHANNELS; i++) {
    setServoPulseWidth(i, 1500.0);
  }
  
  Serial.println("System Ready - Listening for I2C commands");
  printSystemInfo();
}

void loop() {
  // Handle any pending PWM updates
  updateDirtyChannels();
  
  // Optional: Add serial command interface for debugging
  handleSerialCommands();
  
  delay(1);
}

//====================================================================================
// System Initialization
//====================================================================================

void initializePWMSystem() {
  Serial.println("Initializing high-precision PWM system...");
  
  // Configure the timer for maximum precision
  ledc_timer_config_t timer_config = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION_BITS,
    .timer_num = LEDC_TIMER_NUM,
    .freq_hz = (uint32_t)current_frequency,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_config);
  
  // Initialize all PWM channels
  for (int i = 0; i < PWM_CHANNELS; i++) {
    ledc_channel_config_t channel_config = {
      .gpio_num = pwm_pins[i],
      .speed_mode = LEDC_MODE,
      .channel = (ledc_channel_t)i,
      .timer_sel = LEDC_TIMER_NUM,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&channel_config);
    
    // Initialize channel state
    channels[i].on_ticks = 0;
    channels[i].off_ticks = 0;
    channels[i].duty_cycle = 0;
    channels[i].pulse_width_us = 0.0;
    channels[i].dirty = false;
    
    Serial.printf("PWM Channel %2d: GPIO %2d configured\n", i, pwm_pins[i]);
  }
}

void initializeI2CSlave() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_ADDRESS);
  Wire.onReceive(handleI2CReceive);
  Wire.onRequest(handleI2CRequest);
  Serial.printf("I2C slave initialized at address 0x%02X\n", I2C_ADDRESS);
}

//====================================================================================
// I2C Protocol Handler
//====================================================================================

void handleI2CReceive(int numBytes) {
  if (numBytes < 2) {
    // Consume invalid data
    while (Wire.available()) Wire.read();
    return;
  }
  
  uint8_t reg_address = Wire.read();
  numBytes--;
  
  switch (reg_address) {
    case MODE1_REG_ADDRESS:
      handleMode1Write(numBytes);
      break;
      
    case PRESCALE:
      handlePrescaleWrite(numBytes);
      break;
      
    default:
      // Check if it's a LED register (PWM channel)
      if (reg_address >= LED0_ON_L_ADDRESS) {
        handlePWMRegisterWrite(reg_address, numBytes);
      } else {
        // Unknown register, consume remaining bytes
        while (Wire.available()) Wire.read();
      }
      break;
  }
}

void handleI2CRequest() {
  // Send back a status byte or register value
  Wire.write(mode1_register);
}

void handleMode1Write(int numBytes) {
  if (numBytes >= 1 && Wire.available()) {
    mode1_register = Wire.read();
    
    if (mode1_register & MODE1_RESTART) {
      Serial.println("I2C: Restart command received");
      // Perform any necessary restart actions
    }
    
    if (mode1_register & MODE1_AI) {
      bulk_update_mode = true;
      Serial.println("I2C: Auto-increment mode enabled");
    } else {
      bulk_update_mode = false;
    }
  }
  
  // Consume remaining bytes
  while (Wire.available()) Wire.read();
}

void handlePrescaleWrite(int numBytes) {
  if (numBytes >= 1 && Wire.available()) {
    prescale_register = Wire.read();
    
    // Calculate frequency from prescale (PCA9685 formula)
    float new_frequency = 25000000.0 / (4096.0 * (prescale_register + 1));
    updatePWMFrequency(new_frequency);
    
    Serial.printf("I2C: Frequency updated to %.2f Hz (prescale=%d)\n", 
                  new_frequency, prescale_register);
  }
  
  while (Wire.available()) Wire.read();
}

void handlePWMRegisterWrite(uint8_t start_reg, int numBytes) {
  // Determine channel and register offset
  int reg_offset_from_led0 = start_reg - LED0_ON_L_ADDRESS;
  int channel = reg_offset_from_led0 / 4;
  int byte_offset = reg_offset_from_led0 % 4;
  
  if (channel >= PWM_CHANNELS) {
    while (Wire.available()) Wire.read();
    return;
  }
  
  // Handle bulk write (auto-increment mode)
  if (bulk_update_mode && byte_offset == 0 && numBytes >= 4 * PWM_CHANNELS) {
    handleBulkPWMWrite(channel, numBytes);
  } else {
    handleSingleChannelWrite(channel, byte_offset, numBytes);
  }
}

void handleBulkPWMWrite(int start_channel, int numBytes) {
  Serial.printf("I2C: Bulk PWM update starting from channel %d\n", start_channel);
  
  int channels_to_update = min((numBytes / 4), (PWM_CHANNELS - start_channel));
  
  for (int i = 0; i < channels_to_update && Wire.available() >= 4; i++) {
    int ch = start_channel + i;
    
    // Read 4 bytes: on_low, on_high, off_low, off_high
    uint8_t on_low = Wire.read();
    uint8_t on_high = Wire.read();
    uint8_t off_low = Wire.read();
    uint8_t off_high = Wire.read();
    
    uint16_t on_ticks = on_low | (on_high << 8);
    uint16_t off_ticks = off_low | (off_high << 8);
    
    updateChannelFromTicks(ch, on_ticks, off_ticks);
  }
  
  // Consume any remaining bytes
  while (Wire.available()) Wire.read();
  
  Serial.printf("I2C: Bulk update completed for %d channels\n", channels_to_update);
}

void handleSingleChannelWrite(int channel, int byte_offset, int numBytes) {
  uint8_t data[4] = {0, 0, 0, 0};
  int bytes_to_read = min(numBytes, 4 - byte_offset);
  
  for (int i = 0; i < bytes_to_read && Wire.available(); i++) {
    data[byte_offset + i] = Wire.read();
  }
  
  // Parse complete ON/OFF values
  uint16_t on_ticks = data[0] | (data[1] << 8);
  uint16_t off_ticks = data[2] | (data[3] << 8);
  
  updateChannelFromTicks(channel, on_ticks, off_ticks);
  
  while (Wire.available()) Wire.read();
}

//====================================================================================
// PWM Control Functions
//====================================================================================

void updateChannelFromTicks(int channel, uint16_t on_ticks, uint16_t off_ticks) {
  if (channel < 0 || channel >= PWM_CHANNELS) return;
  
  channels[channel].on_ticks = on_ticks;
  channels[channel].off_ticks = off_ticks;
  
  // Convert PCA9685 ticks to pulse width
  float period_us = 1000000.0 / current_frequency;
  float pulse_width_us = (off_ticks / 4096.0) * period_us;
  
  channels[channel].pulse_width_us = pulse_width_us;
  
  // Calculate ESP32 duty cycle (16-bit resolution)
  uint32_t duty_cycle = 0;
  if (off_ticks > 0) {
    duty_cycle = (uint32_t)((off_ticks / 4096.0) * PWM_MAX_VALUE);
  }
  
  channels[channel].duty_cycle = duty_cycle;
  channels[channel].dirty = true;
  
  Serial.printf("Ch%2d: %4d ticks -> %6.1fμs (duty=%5d)\n", 
                channel, off_ticks, pulse_width_us, duty_cycle);
}

void setServoPulseWidth(int channel, float pulse_width_us) {
  if (channel < 0 || channel >= PWM_CHANNELS) return;
  
  // Convert microseconds to PCA9685-style ticks for consistency
  float period_us = 1000000.0 / current_frequency;
  uint16_t off_ticks = (uint16_t)((pulse_width_us / period_us) * 4096.0);
  
  updateChannelFromTicks(channel, 0, off_ticks);
}

void updateDirtyChannels() {
  for (int i = 0; i < PWM_CHANNELS; i++) {
    if (channels[i].dirty) {
      ledc_set_duty(LEDC_MODE, (ledc_channel_t)i, channels[i].duty_cycle);
      ledc_update_duty(LEDC_MODE, (ledc_channel_t)i);
      channels[i].dirty = false;
    }
  }
}

void updatePWMFrequency(float new_frequency) {
  if (new_frequency < 1.0 || new_frequency > 1000.0) {
    Serial.printf("Warning: Frequency %.2f Hz out of range, ignoring\n", new_frequency);
    return;
  }
  
  current_frequency = new_frequency;
  
  // Update timer frequency
  ledc_set_freq(LEDC_MODE, LEDC_TIMER_NUM, (uint32_t)new_frequency);
  
  // Recalculate all pulse widths with new frequency
  for (int i = 0; i < PWM_CHANNELS; i++) {
    if (channels[i].off_ticks > 0) {
      updateChannelFromTicks(i, channels[i].on_ticks, channels[i].off_ticks);
    }
  }
}

//====================================================================================
// Debug and Utility Functions
//====================================================================================

void printSystemInfo() {
  Serial.println("\n=== System Configuration ===");
  Serial.printf("PWM Frequency: %.2f Hz\n", current_frequency);
  Serial.printf("PWM Resolution: %d bits (%d levels)\n", PWM_RESOLUTION_BITS, PWM_MAX_VALUE + 1);
  Serial.printf("I2C Address: 0x%02X\n", I2C_ADDRESS);
  Serial.printf("Active Channels: %d\n", PWM_CHANNELS);
  
  Serial.println("\nGPIO Pin Mapping:");
  for (int i = 0; i < PWM_CHANNELS; i++) {
    Serial.printf("Channel %2d -> GPIO %2d\n", i, pwm_pins[i]);
  }
  Serial.println("============================\n");
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "status") {
      printChannelStatus();
    } else if (command == "info") {
      printSystemInfo();
    } else if (command.startsWith("servo ")) {
      // Command format: "servo <channel> <pulse_width_us>"
      int space1 = command.indexOf(' ', 6);
      if (space1 > 0) {
        int channel = command.substring(6, space1).toInt();
        float pulse_us = command.substring(space1 + 1).toFloat();
        setServoPulseWidth(channel, pulse_us);
        Serial.printf("Set channel %d to %.1f μs\n", channel, pulse_us);
      }
    } else if (command.startsWith("freq ")) {
      // Command format: "freq <frequency_hz>"
      float freq = command.substring(5).toFloat();
      updatePWMFrequency(freq);
      Serial.printf("Frequency set to %.2f Hz\n", freq);
    }
  }
}

void printChannelStatus() {
  Serial.println("\n=== Channel Status ===");
  for (int i = 0; i < PWM_CHANNELS; i++) {
    Serial.printf("Ch%2d: %4d ticks, %6.1fμs, duty=%5d, GPIO%2d\n",
                  i, channels[i].off_ticks, channels[i].pulse_width_us,
                  channels[i].duty_cycle, pwm_pins[i]);
  }
  Serial.println("======================\n");
}