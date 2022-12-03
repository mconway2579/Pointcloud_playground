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
from pc2_from_image import get_points_from_depth_img_path


def color_depth_image(depth_image_path, rgb_image_path):
    points = get_points_from_depth_img_path(depth_image_path, z_start = 0,
                                                        z_end_offset = 0,
                                                        x_start = 0,
                                                        x_end_offset = 0)
    img = cv.imread(rgb_image_path)
    if(len(img) * len(img[0])) != len(points):
        d_img = cv.imread(depth_image_path)
        raise ValueError(f"image pixels and points don't match in length \nimage shape = {img.shape}\ndepth image shape = {d_img.shape} \nnumpoints = {len(points)}")
    point_index = 0
    for z in range(len(img)):
        for x in range(len(img[z])):
            color_vec = img[z][x]
            r = color_vec[2]
            g = color_vec[1]
            b = color_vec[0]
            a = 255
            if points[point_index][1] == 1:
                r = 0
                b = 0
                g = 0
                a = 0
            rgb = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), a))[0]
            points[point_index][3] = rgb
            point_index += 1
    return points

def get_pc2_message_from_depth_and_rgb_paths(depth_image_path, rgb_image_path):
    header = Header()
    header.frame_id = "map"
    points = color_depth_image(depth_image_path, rgb_image_path)
    fields = get_fields()
    message = point_cloud2.create_cloud(header, fields, points)
    return message

if __name__ == "__main__":
    import sys
    rospy.init_node("dynamic_pc2_node")
    pub = rospy.Publisher("dynamic_pc2_topic", PointCloud2, queue_size=2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        depth_image_path = sys.argv[1]
        rgb_image_path = sys.argv[2]
        message = get_pc2_message_from_depth_and_rgb_paths(depth_image_path,rgb_image_path)
        pub.publish(message)
        rate.sleep()
