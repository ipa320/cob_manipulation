#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib

from cob_mmcontroller.msg import *

import os
import sys
import math
import datetime
import PyKDL
import matplotlib.pyplot as plt

twist_dur_list = [#[0., 0., 0., 0., 0., 0., 0.],
              [.1, 0., 0., 0., 0., 0., 5.],
              [-.1, 0., 0., 0., 0., 0., 5.],
              """[0., .1, 0., 0., 0., 0., 5.],
              [0., -.1, 0., 0., 0., 0., 5.],
              [0., 0., .1, 0., 0., 0., 5.],
              [0., 0., -.1, 0., 0., 0., 5.],
              [.05, .05, 0., 0., 0., 0., 5.],
              [-.05, -.05, 0., 0., 0., 0., 5.],
              [.05, 0., .05, 0., 0., 0., 5.],
              [-.05, 0., -.05, 0., 0., 0., 5.],
              [.0, .05, .05, 0., 0., 0., 5.],
              [.0, -.05, -.05, 0., 0., 0., 5.],
              [0., 0., 0., .1, 0., 0., 5.], #~5°
              [0., 0., 0., -.1, 0., 0., 5.], #~5°
              [0., 0., 0., 0., .1, 0., 5.],
              [0., 0., 0., 0., -.1, 0., 5.],
              [0., 0., 0., 0., 0., .1, 5.],
              [0., 0., 0., 0., 0., -.1, 5.],"""
              ]

class move_and_record_client:
    def __init__(self):
        client = actionlib.SimpleActionClient('move_and_record', MoveAndRecordAction)
        client.wait_for_server()
        print "Server OK"

        self.rootdir = os.path.join("..", data, sys.argv[1])
        os.mkdir(self.rootdir)

    def record_and_save(self)
        for twist in twist_dur_list:
            goal = self.parse_twist_list

            client.send_goal(goal, feedback_cb=print_feedback)
            while client.get_state() == 0 or client.get_state() == 1:
                client.wait_for_result(rospy.Duration.from_sec(1.0))

            result = client.get_result()
            data_set = self.parse_measurement(result, goal.twist)
            self.plot_and_save_data_set(goal.twist, data_set)


    def parse_twist_list(self, twist):
        goal = MoveAndRecordGoal()
        goal.twist.postition.x = twist[0]
        goal.twist.postition.y = twist[1]
        goal.twist.postition.z = twist[2]
        goal.twist.orientation.x = twist[3]
        goal.twist.orientation.y = twist[4]
        goal.twist.orientation.z = twist[5]
        goal.target_duration = twist[6]
        
        return goal


    def parse_measurement(self, record, twist)
        data_set = {"time": [], "pos_x": [[], [], []], "pos_y": [[], [], []], "pos_z": [[], [], []], "rot_x": [[], [], []], "rot_y": [[], [], []], "rot_z": [[], [], []]}

        time_last = 0.0
        pos_x_last = 0.0
        pos_y_last = 0.0
        pos_z_last = 0.0
        rot_x_last = 0.0
        rot_y_last = 0.0
        rot_z_last = 0.0


        for pose in record.poses:
            # parse time
            data_set["time"].append(pose.header.stamp)
            
            # parse poses
            data_set["pos_x"][0].append(pose.pose.position.x)
            data_set["pos_y"][0].append(pose.pose.position.y) 
            data_set["pos_z"][0].append(pose.pose.position.z) 

            #KDL quaternion in r-p-y
            temp_rot = PyKDL.Roration.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                 pose.pose.orientation.z, pose.pose.orientation.w)
            (roll, pitch, yaw) = temp_rot.GetRPY()
            data_set["rot_x"][0].append(roll)
            data_set["rot_y"][0].append(pitch) 
            data_set["rot_z"][0].append(yaw)

            # calc velocity
            dt = header.stamp - time_last
            time_last = header.stamp
            data_set["pos_x"][1].append((pose.pose.position.x - pos_x_last) / dt)
            data_set["pos_y"][1].append((pose.pose.position.y - pos_x_last) / dt) 
            data_set["pos_z"][1].append((pose.pose.position.z - pos_x_last) / dt) 

            data_set["rot_x"][1].append((pose.pose.orientation.x - rot_x_last) / dt)
            data_set["rot_y"][1].append((pose.pose.orientation.y - rot_x_last) / dt) 
            data_set["rot_z"][1].append((pose.pose.orientation.z - rot_x_last) / dt)

            # parse twist
            data_set["pos_x"][2].append(twist.linear.x)
            data_set["pos_y"][2].append(twist.linear.y) 
            data_set["pos_z"][2].append(twist.linear.z) 

            data_set["rot_x"][2].append(twist.angular.x)
            data_set["rot_y"][2].append(twist.angular.y) 
            data_set["rot_z"][2].append(twist.angular.z)

        return data_set


    def plot_and_save_data_set(self, twist, data_set)
        # set up date
        today = datetime.date.today()

        # make directory
        twist_string = "_".join([str(twist.linear.x),
                                 str(twist.linear.y),
                                 str(twist.linear.z),
                                 str(twist.angular.x),
                                 str(twist.angular.y),
                                 str(twist.angular.z)])
        path = os.path.join(self_.rootdir, "twist_"+twist_string+str(today))
        os.mkdir(path)

        time = data_set["time"]

        # save data set
        with open(os.path.join(path, "data_set"), 'w') as data_file:
            data_file.write(str(data_set))

        # plot data
        for name, data in data_set.iteritems():
            if name != "time":
                plt.plot(time, data[0], 'r', time, date[1], 'b--', time, data[2], 'g')
                plt.savefig(os.path.join(path, str(today)+"_"+name), format="pdf")




    def print_feedback(self, feedback):
        print "Feedback: " + feedback


if __name__ == '__main__':
    try:
        rospy.init_node('moveAndRecord_client')
        move_and_record_client()
        rospy.spin()
    except rospy.ROSInterruptException: pass
