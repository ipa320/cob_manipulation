#!/usr/bin/env python

import rospy
from tf.transformations import *
from simple_moveit_interface import *


def gen_pose(frame_id="/odom_combined", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

if __name__ == '__main__':
	rospy.init_node('spawn_grasp_object')
	while rospy.get_time() == 0.0: pass
	
	psi = get_planning_scene_interface()
	rospy.sleep(1.0)
	
	### ADDING OBJECT
	object_name = "pringles"
	pose = gen_pose(frame_id="base_footprint", pos=[-0.7, 0.5, 1.05], euler=[0, 0, 0])
	filename = roslib.packages.get_pkg_dir('cob_pick_place_action')+'/files/meshes/'+object_name+'.stl')
	psi.add_mesh(object_name, pose, filename)
	
	rospy.sleep(1.0)
	
