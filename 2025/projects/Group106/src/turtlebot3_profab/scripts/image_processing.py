#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np

NODE_NAME = "image_processing"
TOPIC_NAME = "/usb_cam/image_raw/compressed"


def image_callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imshow("image", image_np)
    cv2.waitKey(1)


def main():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(TOPIC_NAME, CompressedImage, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
