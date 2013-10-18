#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_moveit_interface')
import rospy

import simple_moveit_interface as smi

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
	while rospy.get_time() == 0.0: pass
	rospy.logerr('Are you using cob3-3? Did you make sure all requirements are met? Read the code file!') 
	ans = raw_input('[y/n] ')
	if ans == 'y':
		smi.add_ground()
		rospy.sleep(1.0)
		while not rospy.is_shutdown():
			print "Going Left"
			success = smi.moveit_joint_goal("arm", [0.4474927123777702, -1.4042363503909014, -1.3427710033793065, 0.8556133802524375, 2.8670612914249532, 0.2447635934094972, -1.8559613290891581], True)
			print "Going Right"
			success = smi.moveit_joint_goal("arm", [-0.2875421191578158, -1.8672004116184473, 0.5368099709961187, -0.07435202034076506, -0.523647727633693, 0.43781158053874125, -0.709280142446791], True)
	elif ans == 'n':
		rospy.logerr('Please activate it!')
	

