#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_pick_place_action')
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import cob_pick_place_action.msg 

def cob_pick_action_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (CobPickAction) to the constructor.
	pick_action_client = actionlib.SimpleActionClient('cob_pick_action', cob_pick_place_action.msg.CobPickAction)

	# Waits until the action server has started up and started
	# listening for goals.
	pick_action_client.wait_for_server()
	
	# Creates a goal to send to the action server.
	goal = cob_pick_place_action.msg.CobPickGoal()
	#goal.object_id = 18
	#goal.object_name = "yellowsaltcube"
	goal.object_id = 50
	goal.object_name = "instantsoup"
	goal.object_pose.header.stamp = rospy.Time.now()
	goal.object_pose.header.frame_id = "base_footprint"
	goal.object_pose.pose.position.x = -0.7
	goal.object_pose.pose.position.y =  0.0  
	goal.object_pose.pose.position.z =  0.85
	goal.object_pose.pose.orientation.w = 1.0
	goal.object_pose.pose.orientation.x = 0.0
	goal.object_pose.pose.orientation.y = 0.0
	goal.object_pose.pose.orientation.z = 0.0

	
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
