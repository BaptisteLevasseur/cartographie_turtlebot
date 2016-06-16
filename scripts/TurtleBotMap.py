#!/usr/bin/env python
# coding: utf-8



import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.srv import GetMap
import tf
from math import pi, cos, sin, isnan


import scipy.misc
import numpy as np


class TurtleBotMap():

	def __init__(self, target_frame, base_link, listener):
		self.target_frame = target_frame
		self.base_link = base_link
		self.listener = listener
		self.metadata = None
		self.mapData = None
		self.pose_origin = None
		self.pose_robot = None


	
	def get_image_pose(self):
		return self.pose_to_pix(self.pose_robot)
	

	def pose_to_pix(self, pose_robot):
		w = self.metadata[0]
		h = self.metadata[1]
		res = self.metadata[2]
		# We first convert pose_robot from the "map" frame to the image frame
		x_robot, y_robot,theta_robot = pose_robot
		x_origin, y_origin, theta_origin= self.pose_origin
		tempx=x_origin+x_robot
		tempy=y_origin+x_robot
		xr_in_im = (y_origin - y_robot) / res + h
		yr_in_im = (x_robot -  x_origin) / res  
		# And apply a rotation
		theta_in_im = theta_robot - theta_origin
		return (int(xr_in_im), int(yr_in_im), theta_in_im)
	
	def pix_to_pose(self, pose_robot_in_im):
		w = self.metadata[0]
		h = self.metadata[1]
		res = self.metadata[2]
		# We first convert pose_robot from the "map" frame to the image frame
		x_robot_in_im, y_robot_in_im,theta_robot_in_im = pose_robot_in_im
		x_origin, y_origin, theta_origin= self.pose_origin
		
		x_robot= y_robot_in_im * res  + x_origin
		y_robot= -((x_robot_in_im - h) * res - y_origin)
		theta_robot = theta_robot_in_im + theta_origin
		return (x_robot, y_robot, theta_robot)

	def get_map(self):
 		# The map is accessed by calling the service dynamic_map()
		get_map = rospy.ServiceProxy('dynamic_map', GetMap)
		# Request the map as well as the metadata
		m = get_map().map
		width=m.info.width
		height = m.info.height
		res = m.info.resolution
		origin = m.info.origin
		data = m.data
		self.metadata = (width, height, res) #metadata pour la map
		x_origin = origin.position.x
		y_origin = origin.position.y
		theta_origin =  (tf.transformations.euler_from_quaternion((origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w)))[2]
		self.pose_origin = (x_origin, y_origin, theta_origin) #origine de la map

		# we also need to know where we are
		t0 = rospy.Time(0)
		self.listener.waitForTransform('map', self.base_link, t0, rospy.Duration(1))
		((x_robot,y_robot,z), rot) = self.listener.lookupTransform('map', self.base_link, t0)
		euler = tf.transformations.euler_from_quaternion(rot)
		theta_robot = euler[2]
		self.pose_robot = (x_robot, y_robot, theta_robot) #position du robot


		# The robot is at (x_robot, y_robot, theta_robot) in the "map" frame
		# The origin is at (x_origin, y_origin, theta_origin) in the "map" frame
		
		

		self.mapData=list(data)
		for i in range(height):
		  for j in range(width):
		     self.mapData[i*width+j]=data[(height-1-i)*width+j]
		
		return (self.metadata, self.mapData)




