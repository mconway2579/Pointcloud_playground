#!/usr/bin/env python
import rospy
import struct

from roslib import message

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from std_msgs.msg import Header
from sensor_msgs import point_cloud2

import cv2 as cv
import numpy as np

from shared import get_fields

def get_points_cube(num_x_points = 8, num_y_points = 8, num_z_points = 8):
    points = []

    x_points = np.arange(0, 1, 1/num_x_points)
    z_points = np.arange(0, 1, 1/num_z_points)
    y_points = np.arange(0, 1, 1/num_y_points)

    for x in x_points:
        for y in y_points:
            for z in z_points:
                r = int(x * 255.0)
                g = int(y * 255.0)
                b = int(z * 255.0)
                a = 255

                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x, y, z, rgb]
                points.append(pt)
    return points

def get_pc2_message_cube():
    header = Header()
    header.frame_id = "map"
    points = get_points_cube(4, 5, 6)
    fields = get_fields()

    message = point_cloud2.create_cloud(header, fields, points)
    return message

if __name__ == "__main__":
    rospy.init_node("dynamic_pc2_node")
    pub = rospy.Publisher("dynamic_pc2_topic", PointCloud2, queue_size=2)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        message = get_pc2_message_cube()
        pub.publish(message)
        rate.sleep()
