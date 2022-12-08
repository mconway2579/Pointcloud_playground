#!/usr/bin/env python
"""inbetween node for topics and pc2_from_rgb_image.py"""
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

from pc2_from_rgb_image import get_pc2_message_from_depth_and_rgb_np

class RosBridge:
    """listen to two topics, one depth image, one rgb image
    get the points from the depth image and color the points from the rgb image"""

    def publish_pc(self):
        """when we have an rgb & depth image, overlay the two and publish the result into ros"""
        if self.has_rgb and self.has_depth:
            #rospy.loginfo(f"depth shape {self.depth_img.shape}\n rgb shape {self.rgb_img.shape}")
            cv.imshow("depth", self.depth_img)
            cv.imshow("rgb", self.rgb_img)
            cv.waitKey(1)
            message = get_pc2_message_from_depth_and_rgb_np(self.depth_img, self.rgb_img)
            self.pub.publish(message)

    def rgb_callback(self, img_msg):
        """get an rgb image, store it"""
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            self.has_rgb = True
        except CvBridgeError:
            rospy.logerr("CvBridge Error")




    def depth_callback(self, img_msg):
        """get a depth image, store it, publish the pointcloud"""
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            self.has_depth = True
        except CvBridgeError:
            rospy.logerr("CvBridge Error")

        self.publish_pc()


    def __init__(self):
        """create rospy subscribers, publishers,
        create instance variables for images"""
        self.rgb_img = None
        self.has_rgb = False
        self.depth_img = None
        self.has_depth = False
        self.bridge = CvBridge()

        self.sub_rgb = rospy.Subscriber("/rgb_img", Image, self.rgb_callback)
        self.sub_depth = rospy.Subscriber("/depth_img", Image, self.depth_callback)


        self.pub = rospy.Publisher("dynamic_pc2_topic", PointCloud2, queue_size=2)





if __name__ == "__main__":
    rospy.init_node("ros_bridge_node")
    l=RosBridge()
    rospy.spin()
