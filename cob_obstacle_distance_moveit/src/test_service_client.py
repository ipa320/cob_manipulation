#!/usr/bin/env python

import rospy
from cob_control_msgs.srv import GetObstacleDistance, GetObstacleDistanceRequest, GetObstacleDistanceResponse

if __name__ == "__main__":
    rospy.wait_for_service('/calculate_distance')
    try:
        client = rospy.ServiceProxy('/calculate_distance', GetObstacleDistance)
        req = GetObstacleDistanceRequest()
        req.links.append("arm_left_7_link")
        res = client(req)
        print res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
