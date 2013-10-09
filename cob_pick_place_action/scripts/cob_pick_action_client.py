#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_pick_place_action')
import rospy
import actionlib
import random
from math import pi
from tf.transformations import *

from geometry_msgs.msg import PoseStamped
import simple_moveit_interface as smi_
import cob_pick_place_action.msg 


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
	psi.add_box("support_surface", pose, size=(0.5, 1.5, 0.78))
	
	rospy.sleep(1.0)


def cob_pick_action_client():
	pick_action_client = actionlib.SimpleActionClient('cob_pick_action', cob_pick_place_action.msg.CobPickAction)
	
	pick_action_client.wait_for_server()
	
	setup_environment()
	
	# Creates a goal to send to the action server.
	goal = cob_pick_place_action.msg.CobPickGoal()
	goal.object_id = 18
	goal.object_name = "yellowsaltcube"
	#goal.object_id = 50
	#goal.object_name = "instantsoup"
	#goal.object_id = 103
	#goal.object_name = "instanttomatosoup"
	
	goal.object_pose.header.stamp = rospy.Time.now()
	goal.object_pose.header.frame_id = "base_footprint"
	#goal.object_pose.pose.position.x = random.uniform(-0.8, -0.6)
	#goal.object_pose.pose.position.y = random.uniform(-0.3,  0.3)
	#goal.object_pose.pose.position.z = random.uniform( 0.8,  1.1)
	#goal.object_pose.pose.orientation.x, goal.object_pose.pose.orientation.y, goal.object_pose.pose.orientation.z, goal.object_pose.pose.orientation.w = quaternion_from_euler(random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2)) 
	goal.object_pose.pose.position.x = -0.7
	goal.object_pose.pose.position.y = 0.0  
	goal.object_pose.pose.position.z = 0.78
	goal.object_pose.pose.orientation.x, goal.object_pose.pose.orientation.y, goal.object_pose.pose.orientation.z, goal.object_pose.pose.orientation.w = quaternion_from_euler(0,0,0) 
	
	#goal.grasp_id = 21
	#goal.grasp_database = "KIT"
	goal.grasp_database = "OpenRAVE"
	goal.support_surface = "support_surface"
	
	# Sends the goal to the action server.
	pick_action_client.send_goal(goal)
	
	# Waits for the server to finish performing the action.
	finished_before_timeout=pick_action_client.wait_for_result(rospy.Duration(300, 0))
	
	if finished_before_timeout:
		state=pick_action_client.get_state()
		print "Action finished: %s"%state
	# Prints out the result of executing the action
	return state  # State after waiting for PickupAction

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('CobPickAction_client_py')
		result = cob_pick_action_client()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
