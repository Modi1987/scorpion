# Gait generator


Generates the gait after listining on /cmd_vel


To change the gait patten, the following service is used

```
ros2 service call gait_generator/set_gait_pattern gait_generator_msgs/srv/SetGaitPattern "{"pattern": 1}"
```