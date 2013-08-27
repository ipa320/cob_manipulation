#! /usr/bin/env python
import roslib; roslib.load_manifest('cob_grasp_generation')
import rospy

import actionlib
import manipulation_msgs.msg
import cob_grasp_generation.msg 

def query_grasps_client():
    client = actionlib.SimpleActionClient('query_grasps', cob_grasp_generation.msg.QueryGraspsAction)
    client.wait_for_server()

    goal = cob_grasp_generation.msg.QueryGraspsGoal()
    goal.object_name="peanuts"
    goal.grasp_id = 0
    goal.num_grasps = 0
    goal.threshold = 0
    
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()
    
if __name__ == '__main__':
    try:
        rospy.init_node('query_grasps_client')
        result = query_grasps_client()
        print "Result:"
        #print result
        print len(result.grasp_list)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
