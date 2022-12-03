#!/usr/bin/env python
import rospy
import struct

from roslib import message

from sensor_msgs.msg import Image

from std_msgs.msg import Header
import sys

class reassigner:
    def image_call_back(self, message):
        message.header.frame_id = self.new_frame_id
        self.pub.publish(message)
    def __init__(self, sub_topic, pub_topic, new_frame_id):
        self.new_frame_id = new_frame_id
        self.pub = rospy.Publisher(pub_topic, Image, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, Image, self.image_call_back)


if __name__ == "__main__":
    rospy.init_node("reasign_frame_ids")
    image_topic = sys.argv[1]
    r = reassigner(image_topic, f"new_frame_id{image_topic}", "camera_rgb_optical_frame")
    rospy.spin()
