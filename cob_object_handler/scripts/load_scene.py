#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_object_handler')
import rospy

from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *
from cob_object_handler.srv import *

def createObject(shapetype, name, pos, euler, extent):
	co = CollisionObject()
	s = Shape()
	co.id = name
	co.operation.operation = CollisionObjectOperation.ADD
	co.header.frame_id = "/odom_combined";
	#co.header.frame_id = "/base_link";
	co.header.stamp = rospy.Time.now();
	s.type = shapetype
	s.dimensions = list(extent)
	co.shapes = [s]
	p = Pose()
	p.position.x, p.position.y, p.position.z = pos
	p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_euler(*euler)
	co.poses = [p]
	print co
	return co


def load_object(type, name, operation, pose=PoseStamped(), dimensions=[], filename=""):
	client = rospy.ServiceProxy('/object_handler/handle_object', HandleObject)
	
	req = HandleObjectRequest()
	req.type = type
	req.id = name
	req.operation = operation
	req.pose = pose
	req.dimensions = dimensions
	req.filename = filename
	
	res = client(req)
	print res
	
	return True

if __name__ == '__main__':
	rospy.init_node('load_scene')
	while rospy.get_time() == 0.0: pass
	
	print "waiting for service..."
	rospy.wait_for_service('/object_handler/handle_object')
	print "...done"
	
	#floor_pose = PoseStamped()
	#floor_pose.header.frame_id = "/odom_combined"
	#floor_pose.header.stamp = rospy.Time.now()
	#floor_pose.pose.position.x, floor_pose.pose.position.y, floor_pose.pose.position.z = [0,0,0]
	#floor_pose.pose.orientation.x, floor_pose.pose.orientation.y, floor_pose.pose.orientation.z, floor_pose.pose.orientation.w = quaternion_from_euler(0,0,0)
	#load_object(HandleObjectRequest.BOX, "floor", "add", floor_pose, dimensions=[3.0,3.0.0.1] )
	
	#box_pose = PoseStamped()
	#box_pose.header.frame_id = "/odom_combined"
	#box_pose.header.stamp = rospy.Time.now()
	#box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z = [-0.9, 0.8, 0.5]
	#box_pose.pose.orientation.x, box_pose.pose.orientation.y, box_pose.pose.orientation.z, box_pose.pose.orientation.w = quaternion_from_euler(0,0,0)
	#load_object(HandleObjectRequest.BOX, "box", "add", box_pose, dimensions=[0.3, 0.5, 0.7] )
	
	#sphere_pose = PoseStamped()
	#sphere_pose.header.frame_id = "/odom_combined"
	#sphere_pose.header.stamp = rospy.Time.now()
	#sphere_pose.pose.position.x, sphere_pose.pose.position.y, sphere_pose.pose.position.z = [0.0, 0.0, 2.5]
	#sphere_pose.pose.orientation.x, sphere_pose.pose.orientation.y, sphere_pose.pose.orientation.z, sphere_pose.pose.orientation.w = quaternion_from_euler(0,0,0)
	#load_object(HandleObjectRequest.SPHERE, "sphere", "add", sphere_pose, dimensions=[0.3] )
	
	#cyl_pose = PoseStamped()
	#cyl_pose.header.frame_id = "/odom_combined"
	#cyl_pose.header.stamp = rospy.Time.now()
	#cyl_pose.pose.position.x, cyl_pose.pose.position.y, cyl_pose.pose.position.z = [-0.5, -0.7, 0.75]
	#cyl_pose.pose.orientation.x, cyl_pose.pose.orientation.y, cyl_pose.pose.orientation.z, cyl_pose.pose.orientation.w = quaternion_from_euler(0,0,0)
	#load_object(HandleObjectRequest.CYLINDER, "cyl", "add", cyl_pose, dimensions=[0.1, 1.5] )
	
	#mesh_pose = PoseStamped()
	#mesh_pose.header.frame_id = "/odom_combined"
	#mesh_pose.header.stamp = rospy.Time.now()
	#mesh_pose.pose.position.x, mesh_pose.pose.position.y, mesh_pose.pose.position.z = [1.2, 0.0, 0.75]
	#mesh_pose.pose.orientation.x, mesh_pose.pose.orientation.y, mesh_pose.pose.orientation.z, mesh_pose.pose.orientation.w = quaternion_from_euler(-1.57,0,3.14)
	#load_object(HandleObjectRequest.MESH, "mesh", "add", mesh_pose, filename="package://gazebo_worlds/Media/models/mug-test.stl" )
	
	#load_object(HandleObjectRequest.DETECT, "milk_model", "add")
