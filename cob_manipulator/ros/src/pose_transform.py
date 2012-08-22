#!/usr/bin/env python  
import roslib
roslib.load_manifest('cob_manipulator')
import rospy
import tf
from cob_srvs.srv import *
from geometry_msgs.msg import  *

def addTransform(transformer, target, origin, translation, rotation,time):
	m = geometry_msgs.msg.TransformStamped()
	m.header.frame_id = target
	m.header.stamp =  time
	m.child_frame_id = origin
	m.transform.translation = translation
	m.transform.rotation = rotation
	transformer.setTransform(m)
	
class PoseTransformer:
	def __init__(self):
		self.listener = tf.TransformListener()
		self.srv = rospy.Service('~get_pose_stamped_transformed', GetPoseStampedTransformed, self.handle_transform)
		rospy.loginfo("Started Pose Transformer Service.")
	def handle_transform(self, request):
		print request
		# TODO: handle properly
		tpo = request.target.pose.orientation
		if tpo.x == 0 and tpo.y == 0 and tpo.z == 0 and tpo.w == 0:
			tpo.w = 1.0
		opo = request.origin.pose.orientation
		if opo.x == 0 and opo.y == 0 and opo.z == 0 and opo.w == 0:
			opo.w = 1.0
		res = GetPoseStampedTransformedResponse()

		try:
			res.result = self.listener.transformPose(request.root_name,request.target)
			res.result.header.stamp = rospy.Time.now()
			res.result.pose.orientation = request.orientation_override
			res.success = True
		except:
			res.success = False
		print res
		return res

if __name__ == '__main__':
	rospy.init_node('pose_transform')
	pt = PoseTransformer()
	rospy.spin()
