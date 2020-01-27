#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import random
from math import pi

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
import cob_pick_place_action.msg

def cob_place_action_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (CobPlaceAction) to the constructor.
	place_action_client = actionlib.SimpleActionClient('cob_place_action', cob_pick_place_action.msg.CobPlaceAction)

	# Waits until the action server has started up and started
	# listening for goals.
	place_action_client.wait_for_server()

	# Creates a goal to send to the action server.
	goal = cob_pick_place_action.msg.CobPlaceGoal()
	#goal.object_class = 18
	#goal.object_name = "yellowsaltcube"
	goal.object_class = 50
	goal.object_name = "instantsoup"

	pose = PoseStamped()

	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "base_footprint"
	#pose.pose.position.x = random.uniform(-0.8, -0.6)
	#pose.pose.position.y = random.uniform(-0.3,  0.3)
	#pose.pose.position.z = random.uniform( 0.8,  1.1)
	#pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2))
	pose.pose.position.x = -0.7
	pose.pose.position.y = -0.5
	pose.pose.position.z = 0.9
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(0,0,0)
	goal.destinations.append(pose)


	# Sends the goal to the action server.
	place_action_client.send_goal(goal)

	# Waits for the server to finish performing the action.
	finished_before_timeout=place_action_client.wait_for_result(rospy.Duration(300, 0))

	if finished_before_timeout:
		state=place_action_client.get_state()
		print("Action finished: %s"%state)
	# Prints out the result of executing the action
	return state  # State after waiting for CobPlaceAction

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('CobPlaceAction_client_py')
		result = cob_place_action_client()
	except rospy.ROSInterruptException:
		print("program interrupted before completion")
