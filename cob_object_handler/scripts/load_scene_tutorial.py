#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_object_handler')
import rospy
import os

from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *
from cob_object_handler.srv import *


def handle_object(operation, name, type=-1, frame_id="/odom_combined", pos=[0,0,0], euler=[0,0,0], dimensions=[], filename="", scale=[1,1,1], padding=0.0, attach_link="", touch_links=[]):
	client = rospy.ServiceProxy('/object_handler/handle_object', HandleObject)
	
	req = HandleObjectRequest()
	req.operation = operation
	req.id = name
	req.type = type
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	req.pose = pose
	req.dimensions = dimensions
	req.filename = filename
	req.scale.x, req.scale.y, req.scale.z = scale
	req.padding = padding
	req.attach_link = attach_link
	req.touch_links = touch_links
	
	res = client(req)
	print res
	
	return True

if __name__ == '__main__':
	rospy.init_node('load_scene')
	while rospy.get_time() == 0.0: pass
	
	print "waiting for service..."
	rospy.wait_for_service('/object_handler/handle_object')
	print "...done"
	
	
	### ADDING SIMPLE SHAPES
	handle_object("add", "floor", HandleObjectRequest.BOX, frame_id="/base_link", pos=[0,0,0], euler=[0,0,0], dimensions=[3.0, 3.0, 0.1])
	
	handle_object("add", "box", HandleObjectRequest.BOX, pos=[-0.9, 0.8, 0.5], dimensions=[0.3, 0.5, 0.7])
	handle_object("add", "sphere", HandleObjectRequest.SPHERE, pos=[0.0, 0.0, 2.5], dimensions=[0.3])
	handle_object("add", "cylinder", HandleObjectRequest.CYLINDER, pos=[-0.5, -0.7, 0.75], dimensions=[0.1, 1.5])
	
	
	### ADDING A MESH
	handle_object("add", "mug", HandleObjectRequest.MESH, pos=[0.6, 0.0, 0.9], euler=[-1.57, 0, 3.14], filename="package://gazebo_worlds/Media/models/mug-test.stl", scale=[0.1, 0.1, 0.1])
	
	### ADDING DETECTED OBJECT
	handle_object("add", "milk_model", HandleObjectRequest.DETECT)
	
	### ADDING URDF OBJECT
	handle_object("add", "table_ikea", HandleObjectRequest.URDF, pos=[1.1,0, 0], filename="/home/fxm/git/care-o-bot/cob_environments/cob_gazebo_objects/objects/table_ikea.urdf")
	
	### ADDING URDF.XACRO OBJECT
	file_path="/home/fxm/git/care-o-bot/cob_environments/cob_gazebo_worlds/urdf/ipa-apartment/"
	xacro_file=file_path+"ipa-apartment.urdf.xacro"
	urdf_file=file_path+"ipa-apartment.urdf"
	os.system("rosrun xacro xacro.py "+xacro_file+" > "+urdf_file)
	handle_object("add", "world", HandleObjectRequest.URDF, filename=urdf_file)
	#os.system("rm "+urdf_file)
	
	###support for .model
	###support for .world
	
	### ATTACHING AN OBJECT --- YOU CAN ALWAYS ONLY HAVE ONE OBJECT ATTACHED TO THE SAME LINK
	touch_links = ["sdh_grasp_link", "sdh_palm_link", "sdh_finger_11_link", "sdh_finger_12_link", "sdh_finger_13_link", "sdh_finger_21_link", "sdh_finger_22_link", "sdh_finger_23_link", "sdh_thumb_1_link", "sdh_thumb_2_link", "sdh_thumb_3_link"]
	handle_object("attach", "box", attach_link="arm_7_link", touch_links=touch_links)
	handle_object("attach", "sphere", attach_link="tray_right_link")
	
	### DETACHING AN OBJECT
	handle_object("detach", "sphere")
	
	### REMOVING AN OBJECT
	handle_object("remove", "world[elevator")
	handle_object("remove", "world[wall_ext")
	handle_object("remove", "cylinder")
	
	
