#!/usr/bin/env python
"""given a rep118 depth image and an rgb image
create points by the depth image and color them by rgb image"""
import rospy

from roslib import message

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from std_msgs.msg import Header

import cv2 as cv

from shared import get_fields, pack_colors
from pc2_from_image import get_points_from_depth_img_path, get_points_from_depth_img

def get_points_from_depth_and_rgb_paths(depth_image_path, rgb_image_path):
    """given a path to an rep118 depth image and rgb image
    color the points from the depth image by the rgb image"""
    points = get_points_from_depth_img_path(depth_image_path, z_start = 0,
                                                        z_end_offset = 0,
                                                        x_start = 0,
                                                        x_end_offset = 0)
    rgb_img = cv.imread(rgb_image_path)
    return color_points(points, rgb_img)

def color_points(points, img):
    "given a set of points and an rgb image, color the points by the rgb image"
    if(len(img) * len(img[0])) != len(points):
        raise ValueError(f"""image pixels and points don't match in length \n
            image shape = {img.shape}\nnumpoints = {len(points)}""")
    point_index = 0
    for z in range(len(img)):
        for x in range(len(img[z])):
            color_vec = img[z][x]
            if color_vec.size != 3:
                color_vec = [color_vec, color_vec, color_vec]
            r = color_vec[2]
            g = color_vec[1]
            b = color_vec[0]
            a = 255
            if points[point_index][1] == 1:
                r = 0
                b = 0
                g = 0
                a = 0
            rgb = pack_colors(r,g,b,a)
            points[point_index][3] = rgb
            point_index += 1
    return points

def get_pc2_message_from_depth_and_rgb_paths(depth_image_path, rgb_image_path):
    """given a path to a depth image and rgb image
    return the point cloud with points from depth and color from rgb"""
    header = Header()
    header.frame_id = "map"
    points = get_points_from_depth_and_rgb_paths(depth_image_path, rgb_image_path)
    fields = get_fields()
    pc2_message = point_cloud2.create_cloud(header, fields, points)
    return pc2_message
def get_pc2_message_from_depth_and_rgb_np(depth_img, rgb_img):
    """given a np_array to a depth image and rgb image
    return the point cloud with points from depth and color from rgb"""
    header = Header()
    header.frame_id = "map"
    points = get_points_from_depth_img(depth_img)
    points = color_points(points, rgb_img)
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
        rgb_image_path = sys.argv[2]
        message = get_pc2_message_from_depth_and_rgb_paths(depth_image_path,rgb_image_path)
        pub.publish(message)
        rate.sleep()
