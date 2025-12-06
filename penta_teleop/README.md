# Pentapod control with Xbox joystick controller


You can control the Pentapod from Xbox joystick controller

Tested with wirless Xbox contorller provided with a dongle

Connect the dongle to Raspberry PI, to control the robot as follows:


1- `Left joystick` for linear motion in the plane (x and y)

2- `Right joystick` for rotations

3- `D-Pad` up and down buttons, to move the robot up and down

4- `X button` to change walking pattern

5- `A button` to change twerk mode

6- `B button` to start executing the twerk motion

7- `D-pad + Right Trigger button (RT)` allow the robot to pitch

8- `Y button` is used to enable/disbale imu stabilization


Note, make sure to enable the motion using:

1- `Right Trigger button (RT)` to enable walking in turbo mode

2- `Left Trigger button (LT)` to enable body motion while feet in place

3- Walking pattern changes only when the robot is in stand-still