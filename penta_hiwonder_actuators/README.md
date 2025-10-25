# penta_hiwonder_actuators python package

Control the pentapod, stream setpoints of the servo actuators over serial bus to Hiwonder motors

## setup

Make sure to fix the permissions for the serialbus to be accessible from the user space, and note down the serial port path and baudrate

## buidling the package

You can build this package

```
 colcon build --packages-select penta_hiwonder_actuators
```

## To test in simulation

```
ros2 launch penta_i2c_actuators penta_i2c_actuators.launch.py mode:=virtual
```


## To use on real-robot:

```
ros2 launch penta_i2c_actuators penta_i2c_actuators.launch.py mode:=real
```

