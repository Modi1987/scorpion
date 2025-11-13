# penta_hiwonder_actuators python package

Control the pentapod, stream setpoints of the servo actuators over serial bus to Hiwonder motors

## setup

Make sure to fix the permissions for the serialbus to be accessible from the user space, and note down the serial port path and baudrate

## buidling the package

You can build this package

```
 colcon build --packages-select penta_hiwonder_actuators
```

## To use on real-robot:

```
ros2 launch penta_pod realhardware_bringup.launch.py motors_interface:=hiwonder
```

