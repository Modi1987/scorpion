# Raspberry Pi4 HW Interface

Hardware interface for controlling servo motors from Raspberry Pi4 and PCA9685 PWM controller.

# Requirements

## Hardware Requirements

- Raspberry Pi4
- PCA9685
- Servo motors (DSS-M15S)

## Wiring (Connection)

Connect Raspberry Pi4 to PCA9685 as in the following

| PCA9685 Pin | Raspberry Pi4 Pin |
|-------------|-------------------|
| GND         | Pin 6 (GND)       |
| SCL         | Pin 5 (SCL)       |
| SDA         | Pin 3 (SDA)       |
| VCC         | Pin 2 (5V)        |


Connect the power line of PCA9685 into external power supply, power supply shall be in range of the servo motors used

In the case of [DSS-M15S](https://www.dfrobot.com/product-1709.html) voltage range 4.8-7.2V can be used

Connect the servo motors into the pins of PCA9685 (you can test with any number of motors, no need to connect all 15 motors to the board)

## Software requirements

We will use Ubuntu 22.04 on Raspberry Pi4

I2C will be used to comunicate between the Raspberry Pi4 and PCA9685 

Python3 gui program inside the folder [scripts](https://github.com/Modi1987/scorpion/tree/main/rpi4_hw_interface/scripts) will be used to test the motors

### Setting up GPIO on Raspberry Pi4

```
# Update package list
sudo apt update

# Install pip if not already installed
sudo apt install python3-pip

# Install RPi.GPIO
sudo pip install RPi.GPIO

# Install adafruit-blinka
sudo pip install adafruit-blinka
```

### Setting up I2C on Raspberry Pi4

Required packages to use I2C on your raspberry-pi, run the following inside your terminal from raspberry-pi4 running Ubuntu 22.04

```
sudo apt-get install python3-smbus
sudo apt-get install i2c-tools
```

### Control the motors from Python GUI


Install the software required to comunicate bteween Raspberry Pi4 and PCA9685, we will use Python3 and the library [adafruit-circuitpython-servokit](https://pypi.org/project/adafruit-circuitpython-servokit/)


```
sudo pip3 install adafruit-circuitpython-servokit
```

The gui script is available in the [scripts](https://github.com/Modi1987/scorpion/tree/main/rpi4_hw_interface/scripts) folder. It uses Tkinter libarry comes with python3 by default (otherwise it is required to be installed)

The easiest way to use the gui is to Transfer the file [simple_limps_control_gui.py](https://github.com/Modi1987/scorpion/tree/main/rpi4_hw_interface/scripts/simple_limps_control_gui.py) into inside the raspberry-pi. You can use a USB pin drive to do so. Then connect a monitor to the raspberry-pi. Afterwards:

Open the terminal and then run:

```
python3 simple_limps_control_gui.py
```

Move the sliders to control the motors into certain position, then press the update Button

See the motors moving

# reference articles

https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/configuring-your-pi-for-i2c

