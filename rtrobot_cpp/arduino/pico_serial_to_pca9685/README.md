# Raspberry Pi Pico USB to PCA9685

This project has been migrated from ESP8266 to Raspberry Pi Pico for controlling a PentaPod "Creature" via USB serial commands.

## Hardware Changes

### ESP8266 → Raspberry Pi Pico Pin Mapping:
- **ESP8266 D1 (GPIO5) SCL** → **Pico GP5 (I2C0 SCL)**
- **ESP8266 D2 (GPIO4) SDA** → **Pico GP4 (I2C0 SDA)**

### Wiring Diagram:
```
Raspberry Pi Pico    PCA9685
GP4 (I2C0 SDA)   →  SDA
GP5 (I2C0 SCL)   →  SCL
3V3              →  VCC
GND              →  GND
```

### Alternative I2C Pins (if needed):
- GP20 (I2C1 SDA) and GP21 (I2C1 SCL)

## Setup Instructions

### Arduino IDE Setup:
1. Install Arduino IDE
2. Add Pico board support:
   - Go to File → Preferences
   - Add this URL to "Additional Boards Manager URLs":
     `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
   - Go to Tools → Board → Boards Manager
   - Search for "pico" and install "Raspberry Pi Pico/RP2040"

### Required Libraries:
1. **Adafruit PWM Servo Driver Library**:
   - Go to Sketch → Include Library → Manage Libraries
   - Search for "Adafruit PWM Servo Driver"
   - Install the library by Adafruit

2. **Wire Library**: Included by default with Pico core

### Board Selection:
- Board: "Raspberry Pi Pico"
- Port: Select the appropriate COM/ttyACM port

## Usage

The serial communication protocol remains the same:
```
#1P1500#2P1300#3P2000 ... #15P1700#16P1800T1000
```

Where:
- `#N` = Servo number (1-16)
- `PXXXX` = PWM value in microseconds (500-2500)
- `T` = Trigger command to execute

## Key Migration Benefits

1. **Cost**: Pico is generally less expensive than ESP8266
2. **Performance**: Dual-core ARM Cortex-M0+ at 133MHz
3. **I/O**: More GPIO pins available
4. **Power**: Better power efficiency
5. **Development**: Strong Arduino IDE support

## Troubleshooting

1. **I2C Issues**: 
   - Check wiring connections
   - Verify PCA9685 address (default 0x40)
   - Try alternative I2C pins if needed

2. **Compilation Errors**:
   - Ensure Pico board package is installed
   - Verify Adafruit PWM library is installed

3. **Serial Communication**:
   - Baud rate: 115200
   - Check correct COM port selection