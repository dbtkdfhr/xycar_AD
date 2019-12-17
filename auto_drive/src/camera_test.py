import cv2
import numpy as np
import time

try:
	import rospy
	from sensor_msgs.msg import Image
	from cv_bridge import CvBridge
except:
	pass

capture = cv2.VideoCapture('/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0')
