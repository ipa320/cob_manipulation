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


"""Analyze Grasp 3D - own module: a collection of several function for analyzing the grasps
"""

from openravepy import poseFromMatrix  # pylint: disable=import-error
import numpy, time, scipy, csv, os
import roslib.packages

#converts all openrave grasps to csv format for scripting
#input-first-line (meta-info): object_name, gripper_type, time (@todo: add planner infos)
#input-struct: number, jointconf, trafo, epsilon, volume, forceclosure, validindicees, direction
def or_to_csv(validgrasps):
	meta_info = validgrasps[0]
	object_name = meta_info[0]
	gripper_type = meta_info[1]

	#create directories
	directory = roslib.packages.get_pkg_dir('cob_grasp_generation')+'/files/database/'+object_name
	if not os.path.exists(directory):
    		os.makedirs(directory)
	pathname_out = directory+'/'+gripper_type+'_'+object_name+'.csv'
	f_out = open(pathname_out, 'w+')

	wr = csv.writer(f_out, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
	wr.writerow(['id', 'object', 'sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'direction', 'qw', 'qx', 'qy', 'qz','pos-x', 'pos-y', 'pos-z', 'eps_l1', 'vol_l1'])
	# ToDo: This will not work for grippers other than SDH

	for i in range (1,len(validgrasps)): #because line 0 is meta-info
		row = [] #create/empty row for new grasp
		actual_grasp = validgrasps[i]
		row.append(str(actual_grasp[0])) #ID
		row.append(object_name)

		##SOLL
		#SDH Joint Values - Beginnend mit Daumen(1) und die Zwei Finger GUZS nummeriert(2)(3)
		#[Fingerwinkel(2)(3), Fingerwinkel(2), Fingerknick(2), Fingerwinkel(3), Fingerknick(3), Fingerwinkel(1), Fingerknick(1)]
		joint_values = actual_grasp[1]

		#Check Knuckle_joint of SDH for values < 0
		#Reason: Openrave generates some zero values that are slighlty below zero
		if joint_values[0] < 0:
			joint_values[0] = 0

		#assign joint values in the right order
		joint_values_sorted = [joint_values[0], joint_values[3], joint_values[4], joint_values[1], joint_values[2], joint_values[5], joint_values[6]]
		for i in range(0,len(joint_values)):
			row.append(joint_values_sorted[i])

		#grasp-direction
		globApproach = actual_grasp[7] #globalApproachDir
		if globApproach[2] < -0.7: #if hand approaches > 63 degrees
			row.append('TOP')
		else:
			row.append('SIDE')

		#convert 4x4 matrix to pose
		trafo4x4 = actual_grasp[2]
		m_in_mm = 1000
		pose = poseFromMatrix(trafo4x4) #returns [w, q0, q1, q2, x, y, z]
		pose[4] = m_in_mm * pose[4] #x-koord
		pose[5] = m_in_mm * pose[5] #y-koord
		pose[6] = m_in_mm * pose[6] #z-koord
		for i in range(0,len(pose)):
			row.append(pose[i])

		#quality
		row.append("%.9f" % actual_grasp[3]) #epsilon with precision
		row.append("%.9f" % actual_grasp[4]) #volume with precision

		#misc
		row.append(actual_grasp[6]) #validindicees

		#write this grasp as row to file
		if (not actual_grasp[3] == 0.0) and (not actual_grasp[4] == 0.0):
			wr.writerow(row)

	f_out.close()
	print("Finished.")
