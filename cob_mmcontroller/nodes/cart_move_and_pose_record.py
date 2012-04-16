#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

import PyKDL

from geometry_msgs.msg import *


class cart_move_and_pose_record:
    def __init__(self):

        # publisher
        self.cart_command_pub = rospy.Publisher('/arm_controller/cart_command', Twist)

        # subscriber
        self.cart_state_sub = rospy.Subscriber('/arm_controller/cart_state', PoseStamped, self.cartStateCB)

        # action server
        self.moveAndRecord_as = actionlib.SimpleActionServer('moveAndRecord', MoveAndRecordAction, self.moveAndRecordActionCB, False)
        self.moveAndRecord_as.start()

    def cartStateCB(self, pose):
        self.current_pose = pose.pose
    

    def moveAndRecordActionCB(self, goal):
        # set up and initialize action feedback and result
        result_ = MoveModelPriorResult()
        feedback_ = MoveModelPriorFeedback()
        result_.success = False
        feedback_.message = 
        self.moveAndRecord_as.publish_feedback(feedback_)

        # set up 
        start_time = rospy.get_time()
        duration = rospy.get_time() - start_time

        # command cartesian twist
        self.cart_command_pub.publish(goal.twist)

        # record poses
        current_pose_stamped = PoseStamped()
        while duration <= goal.target_duration:
            current_pose_stamped.stamp = rospy.Time(duration.to_sec())
            current_pose_stamped.pose = self.current_pose
            result_.poses.append(current_pose_stamped)
            rospy.sleep(0.01)
            duration = rospy.get_time() - start_time

        # command zero twist
        self.cart_command_pub.publish(Twist())

        result_.success = True
        seld.moveAndRecord_as.set_succeeded(result_)





if __name__ == "__main__":
    try:
        rospy.init_node('cart_move_and_pose_record')
        cart_move_and_pose_record()
        rospy.spin()
    except rospy.ROSInterrupedException as e: rospy.signal_shutdown(str(e))
                                                                             
