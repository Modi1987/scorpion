// Copyright: Mohammad Safeea, 2025-Oct-18
// ESP32 serial to i2c program,
// It is used to control the PentaPod "Creature" over USB


// IN A NUTSHELL:
// ===============
// This program:
// 1- Receives joints pwm signals over USB using the format:
//   "#1P1500 #2P1300 #3P2000 ... #15P1700#16P1800T1000D100\r\n"
// 2- Streams the pulse width to the PWM generator (PCA9685)

// I2C wiring of ESP32 and PCA9685:
// 1- Connect pin GPIO22 of ESP32 to SCL of PCA9685.
// 2- Connect pin GPIO21 of ESP32 to SDA of PCA9685.
// Note: These are the default I2C pins for ESP32. You can change them by modifying SDA_PIN and SCL_PIN below.

//-- Libraries Included --------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//------------------------------------------------------------------------------------
// I2C pin configuration for ESP32
#define SDA_PIN 21  // Default SDA pin for ESP32
#define SCL_PIN 22  // Default SCL pin for ESP32

// USB parameters & buffer
#define SERIAL_BAUDRATE 115200
char usb_reply_message[] = "OK\r\n";       // a string to send back
//------------------------------------------------------------------------------------   
// Servo parameters for PCA9685
#define USMIN  500.0 // 500 micro seconds
#define USMAX  2500.0 // 2500 miro seconds
#define SERVO_FREQ 200 // Analog servos run at ~50 Hz updates

// Servo position 16 servos (joint angle) variables
bool is_ready = false;
long servo_current_position_micro[]={-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2};
long servo_last_position_micro[]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
//====================================================================================

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    disp_authors_message();
}

void initiate_i2c() {
    static bool is_initialized = false;
    if (is_initialized) {
        return;
    }
    // Initialize I2C with specific pins for ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Setting servo through PCA 9685
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at SERVO_FREQ Hz updates
    delay(10);
    is_initialized = true;
}

//====================================================================================
void disp_authors_message()
{
  Serial.println("");
  Serial.println("Copyright: Mohammad Safeea, 2025-Oct-18");
  Serial.println("This is a program used to control PentaPod 'Creature' (ESP32 version)");
  Serial.println("");
  Serial.println("You shall send data over serial in the format:");
  Serial.println("#1P1500 #2P1300 #3P2000 ... #15P1700#16P1800T1000");
}
//====================================================================================

  
void loop() {
  int servo_num = -1;
  long pwm_value = -1;
  bool is_servo_num_parsing = false;
  bool is_pwm_parsing = false;

  while (true) {
    if (!Serial.available()) continue;

    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      Serial.write(usb_reply_message);
      continue;
    }

    if (inChar == 'T') {
      pca_pwm_control(servo_num, pwm_value);
      servo_num = -1;
      pwm_value = -1;
      continue;
    }

    if (inChar == '#') {
      pca_pwm_control(servo_num, pwm_value);
      servo_num = 0;
      is_servo_num_parsing = true;
      is_pwm_parsing = false;
      continue;
    }

    if (inChar == 'P') {
      pwm_value = 0;
      is_servo_num_parsing = false;
      is_pwm_parsing = true;
      continue;
    }

    if (inChar >= '0' && inChar <= '9') {
      if (is_servo_num_parsing) {
        servo_num = servo_num * 10 + (inChar - '0');
        continue;
      } 
      if (is_pwm_parsing) {
        pwm_value = pwm_value * 10 + (inChar - '0');
        continue;
      } 
    }
    // Other wise reset parsing vars
    servo_num = -1;
    pwm_value = -1;
    is_servo_num_parsing = false;
    is_pwm_parsing = false;
  }
}


// Move the motor {servo_num}
void pca_pwm_control(uint8_t servo_num, long pwm_value)
{
  initiate_i2c();

  if (servo_num < 1 || servo_num > 16) {
    return;
  }

  if (pwm_value < USMIN) {
    Serial.println("Error, PWM value is below minimum.");
    return;
  }

  if (pwm_value > USMAX) {
    Serial.println("Error, PWM value is above maximum.");
    return;
  }

  uint8_t servo_index = servo_num - 1;
  
  // Send the pwm value to the servo
  servo_current_position_micro [servo_index] = pwm_value;
  if (servo_last_position_micro [servo_index] != servo_current_position_micro [servo_index]) {
    pwm.writeMicroseconds(servo_index, pwm_value);
    servo_last_position_micro [servo_index] = servo_current_position_micro [servo_index];
  }
}