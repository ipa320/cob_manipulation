#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from geometry_msgs.msg import *
from cob_mmcontroller.msg import *


class cart_move_and_pose_record:
    def __init__(self):
        self.Bstarted = False

        # publisher
        self.cart_command_pub = rospy.Publisher('/arm_controller/cart_command', Twist)

        # subscriber
        self.cart_state_sub = rospy.Subscriber('/arm_controller/cart_state', PoseStamped, self.cartStateCB)

        # action server
        self.moveAndRecord_as = actionlib.SimpleActionServer('move_and_record', MoveAndRecordAction, self.moveAndRecordActionCB, False)
        self.moveAndRecord_as.start()


    def cartStateCB(self, pose):
        self.current_pose = pose.pose
        #if not self.Bstarted:
        #    self.poses_cb = []
        #if self.Bstarted:
        #    self.poses_cb.append(pose.pose)
    

    def moveAndRecordActionCB(self, goal):
        # set up and initialize action feedback and result
        self.result_ = MoveAndRecordResult()
        self.feedback_ = MoveAndRecordFeedback()
        self.result_.success = False

        self.Bstarted = True

        self.feedback_.message = "started"
        self.moveAndRecord_as.publish_feedback(self.feedback_)

        # set up 
        start_time = rospy.get_rostime()
        duration = rospy.get_rostime() - start_time

        # record 1 second before command twist
        while duration <= rospy.Duration.from_sec(1.0):
            self.result_.time.append(rospy.Time(duration.to_sec()))
            self.result_.poses.append(self.current_pose)
            self.result_.twists.append(Twist())
            rospy.sleep(0.005)
            duration = rospy.get_rostime() - start_time

        # command cartesian twist
        self.cart_command_pub.publish(goal.twist)

        while duration <= (goal.target_duration + rospy.Duration.from_sec(1.0)):
            self.result_.time.append(rospy.Time(duration.to_sec()))
            self.result_.poses.append(self.current_pose)
            self.result_.twists.append(goal.twist)
            rospy.sleep(0.005)
            duration = rospy.get_rostime() - start_time

        # command zero twist
        self.cart_command_pub.publish(Twist())

        # record 1 second after stopped movement
        #while duration <= (goal.target_duration + rospy.Duration.from_sec(2.0)):
        #    result_.time.append(rospy.Time(duration.to_sec()))
        #    result_.poses.append(self.current_pose)
        #    result_.twists.append(Twist())
        #    rospy.sleep(0.01)
        #    duration = rospy.get_rostime() - start_time
        

        #self.result_.poses_cb = self.poses_cb

        rospy.sleep(0.8)

        self.result_.success = True
        rospy.sleep(0.5)
        self.moveAndRecord_as.set_succeeded(self.result_)





if __name__ == "__main__":
    try:
        rospy.init_node('cart_move_and_pose_record')
        cart_move_and_pose_record()
        rospy.spin()
    except rospy.ROSInterruptException as e: rospy.signal_shutdown(str(e))
                                                                             
