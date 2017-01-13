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

node = roshelper.Node("ImageLoop")


@node.entry_point()
class ImageLoopNode(object):
    def __init__(self):
        self.image_dir = rospy.get_param("~image_dir")
        rospy.loginfo("Image dir: %s", self.image_dir)
        self.image_time = rospy.get_param("~image_time")
        rospy.loginfo("Image time: %s", self.image_time)

        self.camera_info_url = rospy.get_param("~camera_info_url")
        rospy.loginfo("Camera info url: %s", self.camera_info_url)

        self.images = []
        self.current_image = 0

        try:
            files = os.listdir(self.image_dir)
        except Exception as e:
            rospy.logerr("Image dir is not valid. Reason: %s", e)
            sys.exit(1)
        files = filter(lambda x: os.path.isfile(os.path.join(self.image_dir, x)), files)

        for f in files:
            image = cv2.imread(os.path.join(self.image_dir, f))
            self.images.append(image)
        rospy.loginfo("Images count %s", len(self.images))
        self.last = rospy.get_time()

        self.bridge = cv_bridge.CvBridge()

        self.camera_info = camera_info_manager.CameraInfoManager("Lifecam", self.camera_info_url)
        self.camera_info.setURL(self.camera_info_url)
        self.camera_info.loadCameraInfo()

    @node.publisher("/camera/image_raw", Image)
    def publish_image(self, image):
        info = self.camera_info.getCameraInfo()
        return self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

    @node.publisher("/camera/camera_info", CameraInfo)
    def publish_info(self, image):
        info = self.camera_info.getCameraInfo()
        info.header.stamp = rospy.get_rostime()
        info.header.frame_id = "camera"
        return info


    @node.main_loop(frequency=10)
    def run(self):
        if (rospy.get_time() - self.last) > self.image_time:
            self.last = rospy.get_time()
            self.current_image = (self.current_image + 1) % len(self.images)
            rospy.loginfo("Updating image to %s", self.current_image)
        self.publish_image(self.images[self.current_image])
        self.publish_info(self.images[self.current_image])

if __name__ == "__main__":
    node.start(spin=True)