# penta_i2c_actuators python package

Control the pentapod, stream setpoints of PCA9685 PWM controller of the servo actuators over I2C from the raspberry pi

## setup

Make sure to update the servo and the i2c parameters in the file (general_config.yaml) inside the (penta_description) package

i2c smbus shall be installed, check the readme in the package (rpi4_hw_interface) for more info on how to do it

## buidling the package

after installing smbus, you can build this package

```
 colcon build --packages-select penta_i2c_actuators
```

## To test in simulation

```
ros2 launch penta_i2c_actuators penta_i2c_actuators.launch.py mode:=virtual
```


## To use on real-robot:

```
ros2 launch penta_i2c_actuators penta_i2c_actuators.launch.py mode:=real
```

