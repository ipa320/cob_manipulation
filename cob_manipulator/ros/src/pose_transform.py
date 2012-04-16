#!/usr/bin/env python  
import roslib
roslib.load_manifest('cob_manipulator')
import rospy
import tf
from cob_manipulator.srv import *
from geometry_msgs.msg import  *

def addTransform(transformer, target, origin, translation, rotation):
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = target
    m.header.stamp = rospy.Time.now()
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
        t = tf.TransformerROS(True, rospy.Duration(10.0))
        
        self.listener.waitForTransform(request.tip_name,request.target.header.frame_id,rospy.Time(),rospy.Duration(1.0))
        lookUp = self.listener.transformPose(request.tip_name,request.target)
        addTransform(t,"old_tip","new_pose_target",lookUp.pose.position,lookUp.pose.orientation)
        
        self.listener.waitForTransform(request.tip_name,request.origin.header.frame_id,rospy.Time(),rospy.Duration(1.0))
        lookUp = self.listener.transformPose(request.tip_name,request.origin)
        addTransform(t,"old_tip","old_origin",lookUp.pose.position,lookUp.pose.orientation)

        transform = t.lookupTransform("old_origin","new_pose_target",rospy.Time())
        #TODO: transform directly, omit additional transformer
        
        res = GetPoseStampedTransformedResponse()
        
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
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
