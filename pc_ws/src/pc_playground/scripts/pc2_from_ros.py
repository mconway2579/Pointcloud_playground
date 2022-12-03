#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from sensor_msgs.msg import PointCloud2
from pc2_from_rgb_image import get_pc2_message_from_depth_and_rgb_np
class ros_bridge:

    def publish_pc(self):
        if self.has_rgb and self.has_depth:
            rospy.loginfo(f"depth shape {self.depth_img.shape}\n rgb shape {self.rgb_img.shape}")
            cv.imshow("depth", self.depth_img)
            cv.imshow("rgb", self.rgb_img)
            cv.waitKey(1)
            message = get_pc2_message_from_depth_and_rgb_np(self.depth_img, self.rgb_img)
            self.pub.publish(message)

    def rgb_callback(self, img_msg):
          # Try to convert the ROS Image message to a CV2 Image
          try:
              self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
              self.has_rgb = True
          except CvBridgeError:
              rospy.logerr("CvBridge Error")




    def depth_callback(self, img_msg):
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            self.has_depth = True
        except CvBridgeError:
            rospy.logerr("CvBridge Error")

        self.publish_pc()


    def __init__(self):
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
    l=ros_bridge()
    rospy.spin()
