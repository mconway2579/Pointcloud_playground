#!/usr/bin/env python
import rospy
import struct

from roslib import message

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

from std_msgs.msg import Header
from sensor_msgs import point_cloud2

import cv2 as cv
from shared import get_fields

def get_points_from_depth_img(depth_image_path, z_start = 0, z_end_offset = 0, x_start = 0, x_end_offset = 0):
    img = cv.imread(depth_image_path)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    points = []

    for z in range(z_start, len(gray)-z_end_offset):
        z_coord = (len(gray)-z) / len(gray)
        for x in range(x_start, len(gray[z])-x_end_offset):
            x_coord = (len(gray[z]) - x) / len(gray[z])
            y_coord = (255-gray[z][x])/255

            r = int(x_coord * 255.0)
            g = int(y_coord * 255.0)
            b = int(z_coord * 255.0)
            a = 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

            pt = [x_coord, y_coord, z_coord, rgb]
            points.append(pt)

    return points

def get_pc2_message_from_depth_image(depth_image_path):
    header = Header()
    header.frame_id = "map"
    points = get_points_from_depth_img(depth_image_path)
    fields = get_fields()

    message = point_cloud2.create_cloud(header, fields, points)


    return message


if __name__ == "__main__":
    rospy.init_node("dynamic_pc2_node")
    pub = rospy.Publisher("dynamic_pc2_topic", PointCloud2, queue_size=2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        depth_image_path = "/home/max/Pointcloud_playground/pc_ws/src/pc_playground/test_images/middlebury/teddy/disp6.png"
        message = get_pc2_message_from_depth_image(depth_image_path)
        pub.publish(message)
        rate.sleep()
