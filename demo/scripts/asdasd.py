#!/usr/bin/env python
# PointCloud2 color cube
import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


points = []
lim = 8
for i in range(lim):
    for j in range(lim):
        for k in range(lim):
            x = float(i) / lim
            y = float(j) / lim
            z = float(k) / lim
            pt = [x, y, z, 0]
            r = int(x * 255.0)
            g = int(y * 255.0)
            b = int(z * 255.0)
            a = 255
            print r, g, b, a
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            print hex(rgb)
            pt[3] = rgb
            points.append(pt)

print(points)
