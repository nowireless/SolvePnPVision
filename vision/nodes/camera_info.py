#!/usr/bin/env python

import rospy
import roshelper
import cv2
import cv_bridge
import os
import sys
import camera_info_manager
from sensor_msgs.msg import CameraInfo

from sensor_msgs.msg import Image

node = roshelper.Node("camera_info")


@node.entry_point()
class CameraInfoNode(object):
    def __init__(self):
        self.camera_info_url = rospy.get_param("~camera_info_url")
        rospy.loginfo("Camera info url: %s", self.camera_info_url)

        self.camera_info = camera_info_manager.CameraInfoManager("Realsense", self.camera_info_url)
        self.camera_info.setURL(self.camera_info_url)
        self.camera_info.loadCameraInfo()

    @node.publisher("/camera/color/camera_info", CameraInfo)
    def publish_info(self):
        info = self.camera_info.getCameraInfo()
        info.header.stamp = rospy.get_rostime()
        info.header.frame_id = "camera"
        return info


    @node.main_loop(frequency=10)
    def run(self):
        self.publish_info()

if __name__ == "__main__":
    node.start(spin=True)