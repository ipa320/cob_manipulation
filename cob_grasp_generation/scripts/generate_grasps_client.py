#! /usr/bin/env python
import roslib; roslib.load_manifest('cob_grasp_generation')
import rospy

import actionlib
import manipulation_msgs.msg
import cob_grasp_generation.msg 

def generate_grasps_client():
    client = actionlib.SimpleActionClient('generate_grasps', cob_grasp_generation.msg.GenerateGraspsAction)
    client.wait_for_server()

    goal = cob_grasp_generation.msg.GenerateGraspsGoal()
    #goal.object_name="yellowsaltcube"
    #goal.object_name="hotpot"
    #goal.object_name="hotpot2"
    goal.object_name="instanttomatosoup"
    goal.viewer = True
    goal.replan = True
    #ToDo: set the other OpenRAVE parameters for grasp_generation
    
    
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()
    
if __name__ == '__main__':
    try:
        rospy.init_node('generate_grasps_client')
        result = generate_grasps_client()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
