# USB to PCA9685

This is the ESP32 version of USB to PCA9685 controller for the PentaPod "Creature".

## Hardware

### 1. I2C Pin Configuration
- **ESP32**: Uses GPIO22 for SCL and GPIO21 for SDA (default I2C pins)

### 2. Hardware Wiring
```
ESP32 → PCA9685
GPIO21 (SDA) → SDA
GPIO22 (SCL) → SCL
GND → GND
3.3V/5V → VCC
```

# Software

## Libraries Required

Same as ESP8266 version:
- `Wire` (built-in)
- `Adafruit_PWMServoDriver`

## Protocol

The serial communication protocol for servo motor control:
```
#1P1500 #2P1300 #3P2000 ... #15P1700#16P1800T1000
```

Where:
- `#N` = Servo number (1-16)
- `PXXXX` = PWM value in microseconds (500-2500)
- `T` = Trigger command to execute all movements