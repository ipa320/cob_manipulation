#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_grasp_generation')
import rospy

import actionlib
import manipulation_msgs.msg
import cob_grasp_generation.msg 

def generate_grasps_client():
    client = actionlib.SimpleActionClient('generate_grasps', cob_grasp_generation.msg.GenerateGraspsAction)
    client.wait_for_server()

    goal = cob_grasp_generation.msg.GenerateGraspsGoal(object_name="knaeckebrot", grasp_id=-1, num_grasps=-1, threshold=0.012)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('generate_grasps_client')
        result = generate_grasps_client()
        print "Result:"
        #print result
        print len(result.grasp_list)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
