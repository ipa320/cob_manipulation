#!/usr/bin/env python

import roslib
roslib.load_manifest('cob_kinematics')
import rospy

from kinematics_msgs.srv import *
from arm_navigation_msgs.srv import *

def init():
    SetPlanningSceneDiffService = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
    
    #sending empty request for triggering planning scene 
    planning_scene_request = SetPlanningSceneDiffRequest()
    planning_scene_response = SetPlanningSceneDiffService(planning_scene_request)
    #print planning_scene_response
    if not planning_scene_response:
	    print "Can't get planning scene!"
    
def callFK(joint_values, links):
	req = GetPositionFKRequest()
	req.header.stamp = rospy.Time.now()
	req.header.frame_id="tray_link"
	req.fk_link_names = links
	req.robot_state.joint_state.name = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint'] + ['arm_%d_joint'%(d+1) for d in range (7)]
	req.robot_state.joint_state.position = [0]*7 + joint_values
	fks = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_fk', GetPositionFK)
	res = fks(req)
	#print req
	#print res
	return res.pose_stamped if res.error_code.val == res.error_code.SUCCESS else None
	
def callIK(pose_stamped, link):
	req = GetPositionIKRequest()
	req.timeout = rospy.Duration(5)
	req.ik_request.ik_link_name = link
	req.ik_request.ik_seed_state.joint_state.name = ['arm_%d_joint'%(d+1) for d in range (7)]
	req.ik_request.ik_seed_state.joint_state.position = [-1.1572567240035734, -1.9104664691761568, -2.5334780195730255, -1.7853311980377056, -0.072798739390243047, 0.91767934923272776, -1.8876618005378798]
	req.ik_request.pose_stamped = pose_stamped
	iks = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_ik', GetPositionIK)
	res = iks(req)
	return res.solution.joint_state.position if res.error_code.val == res.error_code.SUCCESS else None, res.error_code


rospy.init_node("test_ik_wrapper")
init()
pose1 = callFK([-1.3813422735995828, -1.9312548399287899, -1.7251153715474463, -1.4565057632094409, 0.7169946808744756, 1.0560171841958719, -2.1230019199424044],['sdh_tip_link'])[0]
print pose1
sol,error_code = callIK(pose1,'sdh_tip_link');
print sol, error_code.val
pose2 = callFK(list(sol),['sdh_tip_link'])[0]
print pose2

