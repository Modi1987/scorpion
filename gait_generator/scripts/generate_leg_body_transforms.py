import math
from math import cos, sin, pi

r = 0.1
for i in range(5):
     theta = i*pi/5.
     x = r*cos(theta)
     y = r*sin(theta)
     z = 0.05
     qw = cos(theta/2.)
     qx = 0.
     qy = 0.
     qz = sin(theta/2.)
     trans = [x, y, z, qx, qy, qz, qw]
     print(trans)

print('***************')

r = 0.25
for i in range(5):
     theta = i*pi/5.
     x = r*cos(theta)
     y = r*sin(theta)
     z = -0.05
     pos = [x, y, z]
     print(pos)
