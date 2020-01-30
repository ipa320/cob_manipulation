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


#from itertools import groupby
#import numpy, time, analyzegrasp3d, scipy
import csv, os

#ROS
import rospy, roslib
from moveit_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *


#check if a database with the object_id exists
def check_database(object_name, gripper_type):
	#Begins here to read the grasp .csv-Files
	path_in = roslib.packages.get_pkg_dir('cob_grasp_generation')+'/files/database/'+object_name+'/'+gripper_type+'_'+object_name+'.csv'

	#Check if path exists
	if os.path.exists(path_in):
		return True
	else:
		return False

def get_grasp_list(object_name, gripper_type, sort_by_quality=False):
	#Begins here to read the grasp .csv-Files
	path_in = roslib.packages.get_pkg_dir('cob_grasp_generation')+'/files/database/'+object_name+'/'+gripper_type+'_'+object_name+'.csv'

	#Check if path exists
	try:
		with open(path_in) as f: pass
	except IOError as e:
		rospy.logerr("The path or file does not exist: "+path_in)

	#If exists open with dictreader
	reader = csv.DictReader( open(path_in, "rb"), delimiter=',')

	if sort_by_quality:
		#sorting for threshold
		return sorted(reader, key=lambda d: float(d['eps_l1']), reverse=True)
	else:
		return sorted(reader, key=lambda d: float(d['id']), reverse=False)

#get the grasps
def get_grasps(object_name, gripper_type, gripper_side="", grasp_id=0, num_grasps=0, threshold=0):
	#open database
	grasp_list = get_grasp_list(object_name, gripper_type)

	#check for grasp_id and return
	if grasp_id > 0:
		if grasp_id < len(grasp_list):
			#print _fill_grasp_msg(gripper_type, gripper_side, grasp_list[grasp_id])
			return [_fill_grasp_msg(gripper_type, gripper_side, grasp_list[grasp_id])]
		else:
			print("Grasp not available")
			return []

	#sorting for threshold
	sorted_list = sorted(grasp_list, key=lambda d: float(d['eps_l1']), reverse=True)

	#calculate max_grasps
	if num_grasps > 0:
		max_grasps=min(len(sorted_list),num_grasps)
	else:
		max_grasps=len(sorted_list)

	#grasp output
	selected_grasp_list = []
	for i in range(0,max_grasps):
		if threshold > 0 and float(sorted_list[i]['eps_l1']) >= threshold:
			selected_grasp_list.append(_fill_grasp_msg(gripper_type, gripper_side, sorted_list[i]))
		elif threshold == 0:
			selected_grasp_list.append(_fill_grasp_msg(gripper_type, gripper_side, sorted_list[i]))
		else:
			pass

	#print len(selected_grasp_list)
	return selected_grasp_list

#fill the grasp message of ROS
def _fill_grasp_msg(gripper_type, gripper_side, grasp):

	#grasp posture
	joint_config = JointTrajectory()
	#joint_config.header.stamp = rospy.Time.now()
	#joint_config.header.frame_id = ""

	#entries in the grasp table are side-independend
	if gripper_type == "sdh":
		joint_config.joint_names = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint']
	elif gripper_type == "sdhx":
		joint_config.joint_names = ['gripper_finger_1_joint', 'gripper_finger_2_joint']
	else:
		rospy.logerr("Gripper not supported")
		return Grasp()

	#get grasp joint configuration
	point = JointTrajectoryPoint()
	for joint_name in joint_config.joint_names:
		point.positions.append(float(grasp[joint_name]))
		point.velocities.append(0.0)
		point.accelerations.append(0.0)
		point.effort.append(0.0)
		point.time_from_start = rospy.Duration(3.0)
	joint_config.points.append(point)

	#side-specific joint_name correction
	#ToDo: get rid of this hardcoded-joint names
	if gripper_type == "sdh":
		if gripper_side == "left":
			joint_config.joint_names = ['sdh_left_knuckle_joint', 'sdh_left_finger_12_joint', 'sdh_left_finger_13_joint', 'sdh_left_finger_22_joint', 'sdh_left_finger_23_joint', 'sdh_left_thumb_2_joint', 'sdh_left_thumb_3_joint']
		elif gripper_side == "right":
			joint_config.joint_names = ['sdh_right_knuckle_joint', 'sdh_right_finger_12_joint', 'sdh_right_finger_13_joint', 'sdh_right_finger_22_joint', 'sdh_right_finger_23_joint', 'sdh_right_thumb_2_joint', 'sdh_right_thumb_3_joint']
		else:
			joint_config.joint_names = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint']
	elif gripper_type == "sdhx":
		if gripper_side == "left":
			joint_config.joint_names = ['gripper_left_finger_1_joint', 'gripper_left_finger_2_joint']
		elif gripper_side == "right":
			joint_config.joint_names = ['gripper_right_finger_1_joint', 'gripper_right_finger_2_joint']
		else:
			joint_config.joint_names = ['gripper_finger_1_joint', 'gripper_finger_2_joint']
	else:
		rospy.logerr("Gripper not supported")
		return Grasp()

	#print joint_config

	#pregrasp posture
	pre_joint_config = JointTrajectory()
	#pre_joint_config.header.stamp = rospy.Time.now()
	#pre_joint_config.header.frame_id = ""

	#ToDo: get rid of this hardcoded-joint names and open_config
	if gripper_type == "sdh":
		if gripper_side == "left":
			pre_joint_config.joint_names = ['sdh_left_knuckle_joint', 'sdh_left_finger_12_joint', 'sdh_left_finger_13_joint', 'sdh_left_finger_22_joint', 'sdh_left_finger_23_joint', 'sdh_left_thumb_2_joint', 'sdh_left_thumb_3_joint']
		elif gripper_side == "right":
			pre_joint_config.joint_names = ['sdh_right_knuckle_joint', 'sdh_right_finger_12_joint', 'sdh_right_finger_13_joint', 'sdh_right_finger_22_joint', 'sdh_right_finger_23_joint', 'sdh_right_thumb_2_joint', 'sdh_right_thumb_3_joint']
		else:
			pre_joint_config.joint_names = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint']
		open_config = [0.0, -0.9854, 0.9472, -0.9854, 0.9472, -0.9854, 0.9472]
	elif gripper_type == "sdhx":
		if gripper_side == "left":
			pre_joint_config.joint_names = ['gripper_left_finger_1_joint', 'gripper_left_finger_2_joint']
		elif gripper_side == "right":
			pre_joint_config.joint_names = ['gripper_right_finger_1_joint', 'gripper_right_finger_2_joint']
		else:
			pre_joint_config.joint_names = ['gripper_finger_1_joint', 'gripper_finger_2_joint']
		open_config = [-0.85, -0.5]

	else:
		rospy.logerr("Gripper not supported")
		return Grasp()

	point = JointTrajectoryPoint()
	for i in range(len(pre_joint_config.joint_names)):
		point.positions.append(open_config[i])
		point.velocities.append(0.0)
		point.accelerations.append(0.0)
		point.effort.append(0.0)
		point.time_from_start = rospy.Duration(3.0)
	pre_joint_config.points.append(point)
	#print pre_joint_config

	#grasp pose
	grasp_pose = PoseStamped()
	grasp_pose.header.stamp = rospy.Time.now()
	#grasp_pose.header.frame_id = ""
	grasp_pose.pose.position.x = float(grasp['pos-x'])*0.001 #mm to m
	grasp_pose.pose.position.y = float(grasp['pos-y'])*0.001 #mm to m
	grasp_pose.pose.position.z = float(grasp['pos-z'])*0.001 #mm to m
	grasp_pose.pose.orientation.x = float(grasp['qx'])
	grasp_pose.pose.orientation.y = float(grasp['qy'])
	grasp_pose.pose.orientation.z = float(grasp['qz'])
	grasp_pose.pose.orientation.w =	float(grasp['qw'])

	#grasp
	grasp_out = Grasp()
	grasp_out.id = grasp['id']
	grasp_out.pre_grasp_posture = pre_joint_config
	grasp_out.grasp_posture = joint_config
	grasp_out.grasp_pose = grasp_pose
	grasp_out.grasp_quality = float(grasp['eps_l1'])
	grasp_out.max_contact_force = 0

	return grasp_out
