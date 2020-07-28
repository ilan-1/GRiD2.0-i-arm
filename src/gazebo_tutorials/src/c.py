#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np
import time
import pygame

pygame.init()
screen = pygame.display.set_mode((400, 400))

# class for state variables
class state_var:
	def __init__(self):
		# position of end effector in meters
		self.y = 0.65
		self.z = 0.75

		# angles between links in radiants
		self.theta1 = 0.0
		self.theta2 = np.pi/2
		self.theta3 = np.pi/2

# class for parameters
class Param:
	def __init__(self):
		# length of link lenghts in meters
		self.link1 = 0.75
		self.link2 = 0.65
		self.link3 = 0.25

		# increment 
		self.inc = 0.01

# function to calculate thetas 
def theta_calc(state_var,param):
	x = state_var.y
	y = state_var.z
	L1 = param.link1
	L2 = param.link2
	
	c = np.arctan2(x,y)
	b1 = (L1**2+L2**2-x**2-y**2)/(2*L1*L2)
	a1 = (x**2+y**2+L1**2-L2**2)/(2*L1*((x**2+y**2)**0.5))

	b1 = max(-1,b1)
	b1 = min(1,b1)

	a1 = max(-1,a1)
	a1 = min(1,a1)

	b = np.arccos(b1)
	a = np.arccos(a1)

	theta1 = c-a
	#theta1 = -(np.pi/2 - theta1) 
	theta2 = (-b + np.pi)
	theta3 = np.pi - theta1 - theta2

	return theta1,theta2,theta3

def main():
	state = state_var()
	param = Param()
	pubj1 = rospy.Publisher('/rrbot/joint1_position_controller/command' , Float64 ,queue_size=10)
	pubj2 = rospy.Publisher('/rrbot/joint2_position_controller/command' , Float64 ,queue_size=10)
	pubj3 = rospy.Publisher('/rrbot/joint3_position_controller/command' , Float64 ,queue_size=10)
	rospy.init_node('mynode',anonymous = True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		print (state.y , state.z ,state.theta1 ,state.theta2 ,state.theta3)
		val1 = state.theta1 - 0.075
		val2 = state.theta2 - 0.075
		val3 = state.theta3 #+ 0.3
		pubj1.publish(val1)
		pubj2.publish(val2)
		pubj3.publish(val3)
		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				quit()
		keys = pygame.key.get_pressed()
        	if keys[pygame.K_LEFT]:
            		state.y -= param.inc
        	if keys[pygame.K_RIGHT]:
            		state.y += param.inc
        	if keys[pygame.K_UP]:
            		state.z += param.inc
        	if keys[pygame.K_DOWN]:
            		state.z -= param.inc

		state.theta1,state.theta2,state.theta3 = theta_calc(state,param)
		rate.sleep()

def signal_handler(signal,frame):
    plt.close()
    sys.exit(0)

if __name__ == '__main__':
	main()
	signal.signal(signal.SIGINT,signal_handler)
