#! /usr/bin/env python
import roslib; roslib.load_manifest('cob_grasp_generation')
import rospy

import actionlib
import moveit_msgs.msg
import cob_grasp_generation.msg 

def show_grasps_client():
    client = actionlib.SimpleActionClient('show_grasps', cob_grasp_generation.msg.ShowGraspsAction)
    client.wait_for_server()

    #object_name = raw_input("Insert object name: ")
    #grasp_id = int(raw_input("Insert grasp_id: "))
    object_name = "yellowsaltcube"
    grasp_id = 0
    
    gripper_type = "sdh"
    
    while not rospy.is_shutdown():
        print grasp_id
        
        # Set the goal here: object_name, grasp_id, sort-by-quality
        goal = cob_grasp_generation.msg.ShowGraspsGoal(object_name, gripper_type, grasp_id, True)

        client.send_goal(goal)
        client.wait_for_result()
        print client.get_result()
        
        raw_input("Enter for next grasp...")
        grasp_id = grasp_id + 1
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('show_grasp_client')
        result = show_grasps_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
