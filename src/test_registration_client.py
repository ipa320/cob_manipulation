#!/usr/bin/env python

import rospy
from cob_srvs.srv import SetString, SetStringRequest, SetStringResponse


if __name__ == "__main__":
    rospy.wait_for_service('/register_links')
    try:
        client = rospy.ServiceProxy('/register_links', SetString)
        req = SetStringRequest()
        req.data = "arm_left_7_link"
        res = client(req)
        print res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
