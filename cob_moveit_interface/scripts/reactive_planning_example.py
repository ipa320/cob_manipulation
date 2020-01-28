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


import rospy
import simple_moveit_interface as smi
import six

#Requirements:
#- latest cob_bringup layer with lookat_component
#- sensor input for moveit enabled?
#- robot facing a wall in the background
#- lookat_action server and monitor started?
#- lookat works for both arm positions?
#
#- These positions currently are cob3-3 specific


if __name__ == '__main__':
	rospy.init_node('move_example')

	smi.add_ground()
	rospy.sleep(1.0)

	#while rospy.get_time() == 0.0: pass
	rospy.logerr('Are you using cob3-3? Did you make sure all requirements are met? Read the code file!')
	ans = six.moves.input('[y/n] ')
	if ans == 'y':
		while not rospy.is_shutdown():
			print("Going Left")
			success = smi.moveit_joint_goal("arm", [0.6132456177400849, -1.2667904686872227, -1.3351177346743512, 0.623757247043325, 2.6443916288424734, 0.2923237627622837, -2.121120037112681], True)
			#success = smi.moveit_joint_goal("arm", [0.4474927123777702, -1.4042363503909014, -1.3427710033793065, 0.8556133802524375, 2.8670612914249532, 0.2447635934094972, -1.8559613290891581], True)
			print("Going Right")
			success = smi.moveit_joint_goal("arm", [-0.4429381361070571, -2.027464518329273, 0.5376498408783335, -0.07512923290336566, -0.5249117648804152, 0.4386535797226253, -0.7085319372561121], True)
			#success = smi.moveit_joint_goal("arm", [-0.2875421191578158, -1.8672004116184473, 0.5368099709961187, -0.07435202034076506, -0.523647727633693, 0.43781158053874125, -0.709280142446791], True)
	elif ans == 'n':
		rospy.logerr('Please activate it!')


