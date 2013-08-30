#!/usr/bin/env python
import roslib; roslib.load_manifest("cob_kinematics_special")
import rospy
from moveit_msgs.srv import *

def empty_service_client():
    rospy.wait_for_service('/compute_ik')
    try:
        client = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        req = GetPositionIKRequest()
        res = client(req)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print "Calling Service..."
    print "...received response: %s"%(empty_service_client())
