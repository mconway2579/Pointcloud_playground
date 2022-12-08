#!/usr/bin/env python
"""a ros node that creates a pc2 message, when displayed will be a cube colored along axis"""
import rospy

from roslib import message

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import numpy as np

from shared import get_fields, pack_colors

def get_points_cube(num_x_points = 8, num_y_points = 8, num_z_points = 8):
    """crerates the points for the cube colored along the axis where:
    x->red, y->green, z->b
    x, y, z axis are limited to >0 <=1"""
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

                rgb = pack_colors(r,g,b,a)
                pt = [x, y, z, rgb]
                points.append(pt)
    return points

def get_pc2_message_cube():
    """creates a message using points from get_points_cube"""
    header = Header()
    header.frame_id = "map"
    points = get_points_cube(4, 5, 6)
    fields = get_fields()

    pc2_message = point_cloud2.create_cloud(header, fields, points)
    return pc2_message

if __name__ == "__main__":
    rospy.init_node("dynamic_pc2_node")
    pub = rospy.Publisher("dynamic_pc2_topic", PointCloud2, queue_size=2)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        message = get_pc2_message_cube()
        pub.publish(message)
        rate.sleep()
