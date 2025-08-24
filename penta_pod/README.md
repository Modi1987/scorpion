
# penta_pod main launcher



## To bring up in simulation

```
ros2 launch penta_pod penta_sim_full_rviz.launch.py

```

## To bring up on real-hardware with I2C actuators

```
ros2 launch penta_pod realhardware_bringup.launch.py
```

## To bring up real hardware using ros2_control

```
ros2 launch penta_pod ros2_control_penta.launch.py
```