#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_moveit_interface')
import rospy

import simple_moveit_interface as smi

if __name__ == '__main__':
	rospy.init_node('move_example')
	while rospy.get_time() == 0.0: pass
	
	config = smi.get_goal_from_server("arm", "folded")
	print config
	success = smi.moveit_joint_goal("arm", config)
	print success
	

