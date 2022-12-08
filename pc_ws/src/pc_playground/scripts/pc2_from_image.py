#!/usr/bin/env python
"""ros node that converts rep118 depth images to point clouds colored along axis"""
import math
import rospy

from roslib import message

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from std_msgs.msg import Header

import cv2 as cv
from shared import get_fields, pack_colors


def get_points_from_depth_img_path(depth_image_path,
                                  z_start = 0, z_end_offset = 0,
                                  x_start = 0, x_end_offset = 0):
    """given a path to a rep118 depth image read it with open cv
     pass the np_array to the get points function"""
    depth_image = cv.imread(depth_image_path)
    gray = cv.cvtColor(depth_image, cv.COLOR_BGR2GRAY)
    return get_points_from_depth_img(gray,
                                    z_start = z_start, z_end_offset = z_end_offset,
                                    x_start = x_start, x_end_offset = x_end_offset)

def get_points_from_depth_img(depth_image,
                            z_start = 0, z_end_offset = 0,
                            x_start = 0, x_end_offset = 0):
    """given a np_array containing a rep118 depth image
    create points in the image colored by axis"""
    points = []

    for z in range(z_start, len(depth_image)-z_end_offset):
        z_coord = (len(depth_image)-z) / len(depth_image)
        for x in range(x_start, len(depth_image[z])-x_end_offset):
            x_coord = (len(depth_image[z]) - x) / len(depth_image[z])
            y_coord = (255-depth_image[z][x])/255
            if math.isnan(y_coord):
                y_coord = 0

            r = int(x_coord * 255.0)
            g = int(y_coord * 255.0)
            b = int(z_coord * 255.0)
            a = 255
            rgb = pack_colors(r,g,b,a)

            pt = [x_coord, y_coord, z_coord, rgb]
            points.append(pt)

    return points

def get_pc2_message_from_depth_image_path(depth_image_path):
    """ given a path to a rep118 depth image return a pc2 message created from the depth image"""
    header = Header()
    header.frame_id = "map"
    points = get_points_from_depth_img_path(depth_image_path)
    fields = get_fields()

    pc2_message = point_cloud2.create_cloud(header, fields, points)


    return pc2_message


if __name__ == "__main__":
    import sys
    rospy.init_node("dynamic_pc2_node")
    pub = rospy.Publisher("dynamic_pc2_topic", PointCloud2, queue_size=2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        depth_image_path = sys.argv[1]
        message = get_pc2_message_from_depth_image_path(depth_image_path)
        pub.publish(message)
        rate.sleep()
