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
	# TODO: handle properly
	tpo = request.target.pose.orientation
	if tpo.x == 0 and tpo.y == 0 and tpo.z == 0 and tpo.w == 0:
	    tpo.w = 1.0
	opo = request.origin.pose.orientation
	if opo.x == 0 and opo.y == 0 and opo.z == 0 and opo.w == 0:
	    opo.w = 1.0
	
        t = tf.TransformerROS(True, rospy.Duration(100.0))
        stamp = rospy.Time.now()
        self.listener.waitForTransform(request.tip_name,request.target.header.frame_id,rospy.Time(),rospy.Duration(1.0))
        lookUp = self.listener.transformPose(request.tip_name,request.target)
        addTransform(t,"old_tip","new_pose_target",lookUp.pose.position,lookUp.pose.orientation,stamp)
        
        self.listener.waitForTransform(request.tip_name,request.origin.header.frame_id,rospy.Time(),rospy.Duration(1.0))
        lookUp = self.listener.transformPose(request.tip_name,request.origin)
        addTransform(t,"old_tip","old_origin",lookUp.pose.position,lookUp.pose.orientation,stamp)

        transform = t.lookupTransform("old_origin","new_pose_target", stamp)
        #TODO: transform directly, omit additional transformer
        
        res = GetPoseStampedTransformedResponse()
        
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = request.tip_name
        ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = transform[0]
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = transform[1]
        
        self.listener.waitForTransform(request.root_name,request.tip_name,rospy.Time.now(),rospy.Duration(1.0))
        try:
            res.result = self.listener.transformPose(request.root_name,ps)
            res.success = True
        except:
            res.success = False
        return res

if __name__ == '__main__':
    rospy.init_node('pose_transform')
    pt = PoseTransformer()
    rospy.spin()
