#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_gripper_grasp_controller_sdh')
import rospy

from simple_script_server import *
from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.srv import *

fake_pr2_gripper_open = [1.57,-1.57,0.0,-0.50,0.7,-0.50,0.7]
fake_pr2_gripper_closed = [1.57,-1.57,0.0,-0.20,0.7,-0.20,0.7]


class cob_gripper_grasp_controller_sdh:

	def __init__(self):
		self.sss = simple_script_server()
		self.action_server = actionlib.SimpleActionServer("/sdh_gripper_grasp_posture_controller", GraspHandPostureExecutionAction, self.execute, False)
		self.query_service = rospy.Service('/sdh_gripper_grasp_status', GraspStatus, self.query_cb)
		self.action_server.start()
		time.sleep(1)

	def execute(self, server_goal):
		if server_goal.goal == 2: #GRASP
			if(len(server_goal.grasp.grasp_posture.position) == 0):
				ros.log_err("Empty grasp given")
			else:
				self.sss.move("sdh", [list(server_goal.grasp.grasp_posture.position)])
				rospy.loginfo("Moving to grasp pose " + str(server_goal.grasp.grasp_posture.position))
				#self.sss.move("sdh", [fake_pr2_gripper_closed])
				#self.sss.move("sdh", [[0,0,0,0,0,0,0,0]])
		if server_goal.goal == 1: #PRE_GRASP
			if(len(server_goal.grasp.pre_grasp_posture.position) == 0):
				ros.log_err("Empty pregrasp given")
			else:
				rospy.loginfo("Moving to pregrasp pose " + str(server_goal.grasp.pre_grasp_posture.position))
				self.sss.move("sdh", [list(server_goal.grasp.pre_grasp_posture.position)])
				#self.sss.move("sdh", [fake_pr2_gripper_open])
				print "fake_pr2"
		if server_goal.goal == 3: #RELEASE
			print "received release command"
			self.sss.move("sdh", "cylopen")
		self.action_server.set_succeeded()

	def query_cb(self, req):
		res = GraspStatusResponse()
		res.is_hand_occupied = True
		return res


if __name__ == "__main__":
	rospy.init_node('cob_gripper_grasp_controller_sdh')
	gripper = cob_gripper_grasp_controller_sdh()
	print "Running cob_gripper_grasp_controller_sdh"
	while not rospy.is_shutdown():
		rospy.sleep(1.0)
