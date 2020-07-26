#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ObjectCount
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class img_disp:
	def __init__(self):
		self.sub_detimg = rospy.Subscriber('/darknet_ros/detection_image', Image ,self.img_callback) # topic publishing images with bounding boxes
		self.sub_bb = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes ,self.boxes_callback) # topic publishing xmin ymin xmax ymax of bounding boxes 
		self.bridge = CvBridge()
	
	def img_callback(self,msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg , 'bgr8')
		except CvBridgeError as e:
			print(e)
		self.img = cv_image 

	def boxes_callback(self,msg):
		self.boxes = msg.bounding_boxes # tuple with elements like [probability xmin ymin xmax ymax id Class]
		height,width, _ = self.img.shape
		print (height)
		print (width)
		x = ((self.boxes[0].xmax + self.boxes[0].xmin)/(2))
		y = ((self.boxes[0].ymax + self.boxes[0].ymin)/(2))
		print(' x is :' , x , 'y is :' ,y)
		self.img = cv2.circle(self.img, (x,y), radius=3, color=(0, 0, 255), thickness=1)
		cv2.imshow('detected',self.img)
		cv2.waitKey(1)		

def main():
	i = img_disp()
	rospy.init_node('locate_object')
	 
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
	
