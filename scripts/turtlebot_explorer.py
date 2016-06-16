#!/usr/bin/env python
# coding: utf-8

# This node communicates with the actionlib server of move_base
# by sending it goals as a Pose in the /map frame

# http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20(Python)

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
from TurtleBotMap import *
from geometry_msgs.msg import Twist

import scipy.misc
import numpy as np

rospy.init_node('explore_client')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

listener = tf.TransformListener()

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)


###### Some global defintions


goal_number=0
target_frame='map'
base_link='base_link'


def reach_goal(x, y, theta):
  target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

  t0=rospy.Time.now()
  goal=MoveBaseGoal()
  goal.target_pose.header.seq=goal_number
  goal.target_pose.header.stamp=t0
  goal.target_pose.header.frame_id=target_frame

  goal.target_pose.pose.position = Point(x, y, 0)
  goal.target_pose.pose.orientation.x = target_quat[0]
  goal.target_pose.pose.orientation.y = target_quat[1]
  goal.target_pose.pose.orientation.z = target_quat[2]
  goal.target_pose.pose.orientation.w = target_quat[3]
  print(str(goal))

  # Sends the goal to the action server.
  client.send_goal(goal)

  # Waits for the server to finish performing the action.
  client.wait_for_result()
  print(client.get_result())

  # Prints out the result of executing the action
  return client.get_result()



# First goal : turn around to scan the surroundings
#t0 = rospy.Time(0)
#listener.waitForTransform('map', base_link, t0, rospy.Duration(1))
#((x,y,z), rot) = listener.lookupTransform('map', base_link, t0)
#euler = tf.transformations.euler_from_quaternion(rot)
#reach_goal(x, y, euler[2] + pi)
#print("G0 done")

#t0 = rospy.Time(0)
#listener.waitForTransform('map', base_link, t0, rospy.Duration(1))
#((x,y,z), rot) = listener.lookupTransform('map', base_link, t0)
#euler = tf.transformations.euler_from_quaternion(rot)
#reach_goal(x, y, euler[2] + pi)
#print("G1 done")
def rotate(vitesse = 1):
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = vitesse  #vitesse en radians par secondes
	rate=rospy.Rate(10)
	timer = 0
	print("Demarrage rotation")
	while timer <  2*pi/vitesse:
		cmd_vel.publish(twist)
		rate.sleep()
		timer = timer+0.1 #Fréquence de 10 Hz donc 0.1 secondes
	print("Fin de rotation")
###### We now look for places to go....


# Some useful functions for converting a position in the world in the
# map frame into its coordinates in the "map", i.e. in the image

# We now start the controller and should in principle run the node
# until no reachable locations remain unknown 
# (i.e. there will be no unknown places next to a known free place)
# Retourne "True" si le pixel (x,y) est adjacent à une bordure et est dans la zone accessible
def is_free(x,y):
    width = metadata[0]
    if(mapData[x*width+y]==-2):
        for k in range(-1,2):
            for l in range(-1,2): # On regarde les cellules adjacentes
                if(k != 0 or l != 0):
                    if(mapData[(x+k)*width+(y+l)] == -1):  # Si une de ces cellules adjacentes est inconnue
                        print("Bordure trouvée")
                        return True
    else:
        return False


# /!\ Il faudra étudier s'il est vraiment nécessaire de recréer un vecteur "copyData" => Apparemment on ne peut pas modifier data avec des valeurs interdites.
# Retourne "True" si le pixel (x,y) est accessible par le robot (situé à un rayon donné des murs)
def is_accessible(x,y,rayon_inflate=4): #Renvoie True si l'on n'est pas près d'un mur
    width = metadata[0]
    if(mapData[x*width+y]==0):
        for m in range(-rayon_inflate,rayon_inflate+1): #On regarde si l'on n'est pas près d'un mur (costmap)
            for n in range(-rayon_inflate,rayon_inflate+1):
                if(mapData[(x+m)*width+(y+n)]==100):
                    return False
        return True
    return False

def remplissage_diff(): #Diffuse la zone d'accessibilité en prenant en compte l'espacement des murs
    pile=[]
    x_robot=pose_in_im[0] #Position du robot
    y_robot=pose_in_im[1]
    for i in range(-3,4):
        for j in range(-3,4):
    		pile.append([x_robot+i,y_robot+j])
    print("Analyse des zones accessibles")
    while pile != []:
        [x,y]=pile.pop()
	width = metadata[0]
	height = metadata[1]
        mapData[x*width+y]=-2 #La valeur arbitraire "-2" correspond à une zone accessible pour le robot
        for k in range(-1,2):
            for l in range(-1,2): #Pour chaque pixels adjacents au pixel actuel, on regarde si la zone est accessible
                if(k !=0 or l !=0):
                    if is_accessible(x+k,y+l):
                        pile.append([x+k,y+l])
    print("Analyse terminée")
    return


def find_ppv(rayon = 10): #Cherche le plus proche voisin libre 
    x_robot=pose_in_im[0]
    y_robot=pose_in_im[1]
    width = metadata[0]
    height = metadata[1]
    print("Recherche du plus proche voisin")
    while (2*rayon < max(height,width)):
        for i in range(-rayon,rayon+1): #Parcourt la carte en partant de la position initiale du robot en faisant des carr
            if(i == -rayon or i==rayon): #Si on est sur un bord de gauche ou de droite
                for j in range(-rayon,rayon+1):
                    if(x_robot+i<width and y_robot+j<height):
                        if(is_free(x_robot+i,y_robot+j)):
                            return (x_robot+i,y_robot+j)
            else:
                
                if(x_robot+i<width and y_robot+j<height):
                    if(is_free(x_robot+i,y_robot+rayon)):
                        return (x_robot+i,y_robot+rayon)
                    if(is_free(x_robot+i,y_robot-rayon)):
                        return (x_robot+i,y_robot-rayon)
        rayon = rayon + 1
    return (float('nan'),float('nan'))




	
turtlebot_map=TurtleBotMap(target_frame,base_link, listener)
(metadata,mapData)=turtlebot_map.get_map()
pose_in_im=turtlebot_map.get_image_pose()
rotate()



#(metadata,pose_origin,pose_robot,pose_in_im,pose_in_map,copyData,image_array)=get_map_data()
remplissage_diff()
(x_im,y_im)=find_ppv()

def whenshutdown():
	rospy.loginfo("Stop node")
        width = metadata[0]
        height = metadata[1]
	image_array = np.zeros((height, width,3), dtype=int)
	# Plotting the map
	for i in range(height):
	  for j in range(width):
	    if(mapData[i*width+j] == -1): # Unknown
	      image_array[i,j,0] = 255
	      image_array[i,j,1] = 255
	      image_array[i,j,2] = 255
	    elif(mapData[i*width+j] == 0 or mapData[i*width+j] == -2): # Free
	      image_array[i,j,0] = 125
	      image_array[i,j,1] = 125
	      image_array[i,j,2] = 125
	    elif(mapData[i*width+j] == 100): # Walls
	      image_array[i,j,0] = 0
	      image_array[i,j,1] = 0
	      image_array[i,j,2] = 0
	# Plotting the location of the robot
	for i in range(-3,4):
	  for j in range(-3,4):
	    image_array[pose_in_im[0]+i, pose_in_im[1]+j] = (255, 0, 0)
	# Plotting its orientation
	for i in range(10):
	  image_array[int(pose_in_im[0]+i*sin(-pose_in_im[2])), int(pose_in_im[1]+i*cos(-pose_in_im[2]))] = (0, 0, 255)
	scipy.misc.imsave('map.png', image_array)
rospy.on_shutdown(whenshutdown)

while(not (isnan(x_im) and isnan(y_im)) and not rospy.is_shutdown() ):
	(x,y,theta)=turtlebot_map.pix_to_pose((x_im,y_im,0))
	print(x,y,theta)
	
	reach_goal(x,y,theta) 
	rotate()
	(metadata,mapData)=turtlebot_map.get_map()
	pose_in_im=turtlebot_map.get_image_pose()

	remplissage_diff()
	(x_im,y_im)=find_ppv()


