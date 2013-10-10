#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_lookat_action')
import rospy
import random

# Brings in the SimpleActionClient
import actionlib

import cob_lookat_action.msg
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import *
from moveit_msgs.msg import *

def cob_lookat_action_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('lookat_action', cob_lookat_action.msg.LookAtAction)
    
    focus_pub = rospy.Publisher('/lookat_target', PoseStamped)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    
    rospy.wait_for_service('/compute_fk')
    get_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
    
    fk_req = moveit_msgs.srv.GetPositionFKRequest()

    fk_req.header.stamp = rospy.Time.now()
    fk_req.header.frame_id = "base_link"
    fk_req.fk_link_names = ["lookat_focus_frame"]
    fk_req.robot_state.joint_state.name = ["torso_lower_neck_tilt_joint","torso_pan_joint","torso_upper_neck_tilt_joint","lookat_lin_joint","lookat_x_joint","lookat_y_joint","lookat_z_joint"]
    fk_req.robot_state.joint_state.position = [random.uniform(-0.25,0.25), random.uniform(-0.2,0.2), random.uniform(-0.4,0.4), random.uniform(-5.0,5.0), random.uniform(-3.141,3.141), random.uniform(-3.141,3.141), random.uniform(-3.141,3.141)]
    print "REQUEST: ", fk_req
    
    fk_res = moveit_msgs.srv.GetPositionFKResponse()
    try:
        fk_res = get_fk(fk_req)
        print "RESPONSE:", fk_res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    # Creates a goal to send to the action server.
    goal = cob_lookat_action.msg.LookAtGoal()
    goal.target = fk_res.pose_stamped[0]
    #focus = PoseStamped()
    #focus.header.stamp = rospy.Time.now()
    #focus.header.frame_id = "base_link"
    #focus.pose.position.x = random.uniform(-5.0, -0.3)
    #focus.pose.position.y = random.uniform(-1.0, 1.0)
    #focus.pose.position.z = random.uniform(0.0, 3.0)
    #focus.pose.orientation.x = 0.0
    #focus.pose.orientation.y = 0.0
    #focus.pose.orientation.z = 0.0
    #focus.pose.orientation.w = 1.0
    #goal.target = focus
    print "GOAL: ", goal
    
    #Debug
    focus_pub.publish(goal.target)
    rospy.sleep(2.0)
   
 
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    
    result = client.get_result()
    print result

    return result.success

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('cob_lookat_action_client')
        result = cob_lookat_action_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
