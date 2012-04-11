#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script
from kinematics_msgs.srv import *
import random
import math

joint_limit = [2.9670,2.0943,2.9670,2.0943,2.9670,2.0943,2.9670]

def calcDist(l1,l2):
	return math.sqrt(sum(map(lambda a,b: (a-b)*(a-b), l1,l2) )) 

class TestScript(script):
		
	def Initialize(self):
		self.sss.init("arm")
		self.fks = rospy.ServiceProxy('/cob_arm_kinematics/get_fk', GetPositionFK)
		self.iks = rospy.ServiceProxy('/cob_arm_kinematics/get_ik', GetPositionIK)

		
	def callFK(self, joint_values):
		req = GetPositionFKRequest()
		req.header.stamp = rospy.Time.now()
		req.header.frame_id="base_footprint"
		req.fk_link_names = ['arm_7_link']
		req.robot_state.joint_state.name = ['arm_%d_joint'%(d+1) for d in range (7)]
		req.robot_state.joint_state.position = joint_values
		
		res = self.fks(req)
		#print res
		return res.pose_stamped if res.error_code.val == res.error_code.SUCCESS else None
		
	def callIK(self, pose_stamped):
		req = GetPositionIKRequest()
		req.timeout = rospy.Duration(5)
		req.ik_request.ik_link_name = "arm_7_link"
		req.ik_request.ik_seed_state.joint_state.name = ['arm_%d_joint'%(d+1) for d in range (7)]
		req.ik_request.ik_seed_state.joint_state.position = [0]*7
		req.ik_request.pose_stamped = pose_stamped
	    
		res = self.iks(req)
		return res.solution.joint_state.position if res.error_code.val == res.error_code.SUCCESS else None, res.error_code
		
	def Run(self): 
	    for i in range(10000):
		print
		print 'joint', #1
		joint_states = [0]*7
		for j in range(7):
		    joint_states[j] = random.uniform(-joint_limit[j],+joint_limit[j])
		for j in joint_states:
		    print j, 
		print "FK", #9
		fk = self.callFK(joint_states)
		if fk is None:
		    continue
		p1 = fk[0].pose.position
		q1 = fk[0].pose.orientation
		print p1.x,p1.y,p1.z, q1.x,q1.y,q1.z,q1.w,
		
		ik, error_code = self.callIK(fk[0])
		#print ik
		print "IK", error_code.val, # 17, 18
		if ik is None:
		    print "none","none","none","none","none","none","none",
		    continue
		for j in ik:
		    print j, 
		print calcDist(joint_states,ik), # 26
		print "RES", #27
		fk_new = self.callFK(joint_states)
		if fk_new is None:
		    continue
		p2 = fk_new[0].pose.position
		q2 = fk_new[0].pose.orientation
		print p2.x,p2.y,p2.z, q2.x,q2.y,q2.z,q2.w, calcDist( [p1.x,p1.y,p1.z] , [p2.x,p2.y,p2.z]), calcDist( [q1.x,q1.y,q1.z,q1.w] , [q2.x,q2.y,q2.z,q2.w]), # 28--35, #35,#36
	    print
	    exit()
		
if __name__ == "__main__":
	SCRIPT = TestScript()
	SCRIPT.Start()

