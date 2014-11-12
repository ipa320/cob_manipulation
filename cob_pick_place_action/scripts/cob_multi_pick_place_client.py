#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_pick_place_action')
import rospy
import actionlib
import random
import numpy
from math import pi
from tf.transformations import *

from geometry_msgs.msg import PoseStamped
import simple_moveit_interface as smi_
import cob_pick_place_action.msg 

### Helper function 
def gen_pose(frame_id="/base_link", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

def setup_environment():
	psi = smi_.get_planning_scene_interface()
	rospy.sleep(1.0)
	
	smi_.clear_objects()
	
	### Add a floor
	smi_.add_ground()
	
	### Add table
	pose = PoseStamped()
	pose.header.frame_id = "/base_footprint"
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x = -0.9
	pose.pose.position.y = 0
	pose.pose.position.z = 0.39
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 0
	pose.pose.orientation.z = 0
	pose.pose.orientation.w = 1
	psi.add_box("bookcase", pose, size=(0.5, 1.5, 0.78))
	rospy.sleep(1.0)
	


def cob_pick_action_client(object_class, object_name, object_pose):
	pick_action_client = actionlib.SimpleActionClient('cob_pick_action', cob_pick_place_action.msg.CobPickAction)
	
	pick_action_client.wait_for_server()
	
	# Creates a goal to send to the action server.
	goal = cob_pick_place_action.msg.CobPickGoal()
	goal.object_class = object_class
	goal.object_name = object_name
	goal.object_pose = object_pose
	
	#goal.grasp_id = 21
	#goal.grasp_database = "KIT"
	goal.grasp_database = "OpenRAVE"
	goal.support_surface = "bookcase"
	
	#print goal
	# Sends the goal to the action server.
	pick_action_client.send_goal(goal)
	
	# Waits for the server to finish performing the action.
	finished_before_timeout=pick_action_client.wait_for_result(rospy.Duration(300, 0))
	
	if finished_before_timeout:
		state=pick_action_client.get_state()
		print "Action finished: %s"%state
	# Prints out the result of executing the action
	return state  # State after waiting for PickupAction


def cob_place_action_client(object_class, object_name, destinations):
	# Creates the SimpleActionClient, passing the type of the action
	# (CobPlaceAction) to the constructor.
	place_action_client = actionlib.SimpleActionClient('cob_place_action', cob_pick_place_action.msg.CobPlaceAction)

	# Waits until the action server has started up and started
	# listening for goals.
	place_action_client.wait_for_server()
	
	# Creates a goal to send to the action server.
	goal = cob_pick_place_action.msg.CobPlaceGoal()
	goal.object_class = object_class
	goal.object_name = object_name
	goal.destinations = destinations
	goal.support_surface = "bookcase"
	
	#print goal
	# Sends the goal to the action server.
	place_action_client.send_goal(goal)

	# Waits for the server to finish performing the action.
	finished_before_timeout=place_action_client.wait_for_result(rospy.Duration(300, 0))

	if finished_before_timeout:
		state=place_action_client.get_state()
		print "Action finished: %s"%state
	# Prints out the result of executing the action
	return state  # State after waiting for CobPlaceAction


if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('CobPickAction_client_py')
		
		setup_environment()
		
		psi = smi_.get_planning_scene_interface()
		rospy.sleep(1.0)
		
		filename = roslib.packages.get_pkg_dir('cob_pick_place_action')+"/files/meshes/yellowsaltcube.stl"
		pose1 = gen_pose(pos=[-0.7, 0.0, 0.85], euler=[random.uniform(-pi, pi), random.uniform(-pi, pi), random.uniform(-pi, pi)])
		psi.add_mesh("cube1", pose1, filename)
		rospy.sleep(1.0)
	
		pose2 = gen_pose(pos=[-0.7, 0.2, 0.85], euler=[random.uniform(-pi, pi), random.uniform(-pi, pi), random.uniform(-pi, pi)])
		psi.add_mesh("cube2", pose2, filename)
		rospy.sleep(1.0)
		
		
		destinations = []
		for x in numpy.arange(-0.9, -0.6, 0.1):
			for y in numpy.arange(-0.3, -0.2, 0.1):
				for theta in numpy.arange(-pi, pi, pi/2):
					destination = gen_pose(pos=[x,y,0.78], euler=[0,0,theta])
					destinations.append(destination)
		
		
		
		result = cob_pick_action_client(18, "cube1", pose1)
		
		#destinations = []
		#destination1 = gen_pose(pos=[-0.7, -0.15, 0.78])
		#destinations.append(destination1)
		result = cob_place_action_client(18, "cube1", destinations)
		
		
		
		result = cob_pick_action_client(18, "cube2", pose2)
		
		#destination2 = gen_pose(pos=[-0.7, -0.30, 0.78])
		#destinations.append(destination2)
		#destinations.append(pose1)
		#destinations.append(pose2)
		result = cob_place_action_client(18, "cube2", destinations)
		
		
		
		
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
