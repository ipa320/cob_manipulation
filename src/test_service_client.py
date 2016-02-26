#!/usr/bin/env python

import rospy
from obstacle_distance.msg import Chain
from obstacle_distance.srv import GetObstacleDistance, GetObstacleDistanceRequest, GetObstacleDistanceResponse


if __name__ == "__main__":
    rospy.wait_for_service('/calculate_distance')
    try:
        client = rospy.ServiceProxy('/calculate_distance', GetObstacleDistance)
        req = GetObstacleDistanceRequest()
        req.links.append("arm_left_7_link")
        chain = Chain()
        chain.chain_base = "arm_right_base_link"
        chain.chain_tip = "arm_right_7_link"
        req.chains.append(chain)
        res = client(req)
        print res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
