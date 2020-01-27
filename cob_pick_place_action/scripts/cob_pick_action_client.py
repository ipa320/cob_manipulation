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
import simple_moveit_interface as smi_
import cob_pick_place_action.msg


def setup_environment():
	psi = smi_.get_planning_scene_interface()
	rospy.sleep(1.0)

	#smi_.clear_objects("arm_7_link")
	smi_.clear_objects("arm_left_7_link")

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


def cob_pick_action_client():
	pick_action_client = actionlib.SimpleActionClient('cob_pick_action', cob_pick_place_action.msg.CobPickAction)

	pick_action_client.wait_for_server()

	setup_environment()

	# Creates a goal to send to the action server.
	goal = cob_pick_place_action.msg.CobPickGoal()
	#goal.object_class = 18
	#goal.object_name = "yellowsaltcube"
	#goal.object_class = 50
	#goal.object_name = "instantsoup"
	#goal.object_class = 103
	#goal.object_name = "instanttomatosoup"
	goal.object_class = 5001
	goal.object_name = "pringles"
	goal.object_pose.header.stamp = rospy.Time.now()
	goal.object_pose.header.frame_id = "base_footprint"

	### cob3
	#goal.object_pose.pose.position.x = random.uniform(-0.8, -0.6)
	#goal.object_pose.pose.position.y = random.uniform(-0.3,  0.3)
	#goal.object_pose.pose.position.z = random.uniform( 0.8,  1.1)
	#goal.object_pose.pose.orientation.x, goal.object_pose.pose.orientation.y, goal.object_pose.pose.orientation.z, goal.object_pose.pose.orientation.w = quaternion_from_euler(random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2))
	#goal.gripper_type = "sdh"
	#goal.gripper_side = ""

	### cob4
	goal.object_pose.pose.position.x = 0.617 + random.uniform(-0.1, 0.1)
	goal.object_pose.pose.position.y = 0.589 + random.uniform(-0.1, 0.1)
	goal.object_pose.pose.position.z = 0.979 + random.uniform(-0.1, 0.1)
	goal.object_pose.pose.orientation.x, goal.object_pose.pose.orientation.y, goal.object_pose.pose.orientation.z, goal.object_pose.pose.orientation.w = quaternion_from_euler(random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2),random.uniform(-pi/2, pi/2))
	#goal.object_pose.pose.orientation.x, goal.object_pose.pose.orientation.y, goal.object_pose.pose.orientation.z, goal.object_pose.pose.orientation.w = quaternion_from_euler(-1.571, -0.000, -1.400)
	goal.gripper_type = "sdhx"
	#goal.gripper_side = ""
	goal.gripper_side = "left"

	#goal.grasp_database = "KIT"
	goal.grasp_database = "OpenRAVE"
	#goal.grasp_database = "ALL"
	#goal.grasp_id = 2
	goal.support_surface = "bookcase"

	# Sends the goal to the action server.
	pick_action_client.send_goal(goal)

	# Waits for the server to finish performing the action.
	finished_before_timeout=pick_action_client.wait_for_result(rospy.Duration(300, 0))

	if finished_before_timeout:
		state=pick_action_client.get_state()
		print("Action finished: %s"%state)
	else:
		print("Action did not finish within timeout")
	return

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('CobPickAction_client_py')
		cob_pick_action_client()
	except rospy.ROSInterruptException:
		print("program interrupted before completion")
