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
                 [-.05, 0., 0., 0., 0., 0., 5.],
                 [.05, 0., 0., 0., 0., 0., 5.],
                 [-.1, 0., 0., 0., 0., 0., 5.],
                 [.1, 0., 0., 0., 0., 0., 5.],
                 [-.2, 0., 0., 0., 0., 0., 5.],
                 [.2, 0., 0., 0., 0., 0., 5.],
                 [-.05, 0., 0., 0., 0., 0., 5.],
                 [.05, 0., 0., 0., 0., 0., 5.],
                 #[0., .1, 0., 0., 0., 0., 5.],
                 #[0., -.1, 0., 0., 0., 0., 5.],
                 #[0., 0., .1, 0., 0., 0., 5.],
                 #[0., 0., -.1, 0., 0., 0., 5.],
                 #[.05, .05, 0., 0., 0., 0., 5.],
                 #[-.05, -.05, 0., 0., 0., 0., 5.],
                 #[.05, 0., .05, 0., 0., 0., 5.],
                 #[-.05, 0., -.05, 0., 0., 0., 5.],
                 #[.0, .05, .05, 0., 0., 0., 5.],
                 #[.0, -.05, -.05, 0., 0., 0., 5.],
                 #[0., 0., 0., .1, 0., 0., 5.], #~5 degrees
                 #[0., 0., 0., -.1, 0., 0., 5.], #~5 degrees
                 #[0., 0., 0., 0., .1, 0., 5.],
                 #[0., 0., 0., 0., -.1, 0., 5.],
                 #[0., 0., 0., 0., 0., .1, 5.],
                 #[0., 0., 0., 0., 0., -.1, 5.]
                 ]
twist_gain = 0.5
rootdir = ''

def main():
    global rootdir
    try:
        rospy.init_node('moveAndRecord_client')
    except rospy.ROSInterruptException as e: rospy.signal_shutdown(str(e)); sys.exit()
    
    client = actionlib.SimpleActionClient('move_and_record', MoveAndRecordAction)
    client.wait_for_server()
    print "Server OK"
    print os.path.abspath(".") 
    print sys.path[0]

    rootdir = os.path.join(sys.path[0], "..", "data", sys.argv[1])
    os.mkdir(rootdir)
    print "created dir"

    for twist in twist_dur_list:
        goal = parse_twist_list(twist)
        print "move", goal

        client.send_goal(goal, feedback_cb=print_feedback)
        while client.get_state() == 0 or client.get_state() == 1:
            client.wait_for_result(rospy.Duration.from_sec(1.0))

        result = client.get_result()
        data_set = parse_measurement(result, goal.twist)
        plot_and_save_data_set(goal.twist, data_set)


def parse_twist_list(twist):
    goal = MoveAndRecordGoal()
    goal.twist.linear.x = twist_gain*twist[0]
    goal.twist.linear.y = twist_gain*twist[1]
    goal.twist.linear.z = twist_gain*twist[2]
    goal.twist.angular.x = twist_gain*twist[3]
    goal.twist.angular.y = twist_gain*twist[4]
    goal.twist.angular.z = twist_gain*twist[5]
    goal.target_duration = rospy.Duration.from_sec(twist[6])
    
    return goal


def parse_measurement(record, twist):
    data_set = {"time": [], "pos_x": [[], [], []], "pos_y": [[], [], []], "pos_z": [[], [], []], "rot_x": [[], [], []], "rot_y": [[], [], []], "rot_z": [[], [], []]}

    time_last = rospy.Time.from_sec(0.0)
    pos_x_last = 0.0
    pos_y_last = 0.0
    pos_z_last = 0.0
    rot_x_last = 0.0
    rot_y_last = 0.0
    rot_z_last = 0.0

    # parse time
    for time in record.time:
        data_set["time"].append(time.to_sec())
        
    pos_x_init = record.poses[0].position.x
    pos_y_init = record.poses[0].position.y
    pos_z_init = record.poses[0].position.z
    temp_rot = PyKDL.Rotation.Quaternion(record.poses[0].orientation.x, record.poses[0].orientation.y,
                                         record.poses[0].orientation.z, record.poses[0].orientation.w)
    (rot_x_init, rot_y_init, rot_z_init) = temp_rot.GetRPY()
    
    for pose in record.poses:
        # parse poses
        data_set["pos_x"][0].append(pose.position.x - pos_x_init)
        data_set["pos_y"][0].append(pose.position.y - pos_y_init) 
        data_set["pos_z"][0].append(pose.position.z - pos_z_init) 

        #KDL quaternion in r-p-y
        temp_rot = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                             pose.orientation.z, pose.orientation.w)
        (roll, pitch, yaw) = temp_rot.GetRPY()
        data_set["rot_x"][0].append(roll - rot_x_init)
        data_set["rot_y"][0].append(pitch - rot_y_init) 
        data_set["rot_z"][0].append(yaw - rot_z_init)

        # calc velocity
        #dt = pose.header.stamp - time_last
        #dt = dt.to_sec()
        #time_last = pose.header.stamp
        #data_set["pos_x"][1].append((pose.position.x - pos_x_last) / dt)
        #data_set["pos_y"][1].append((pose.position.y - pos_x_last) / dt) 
        #data_set["pos_z"][1].append((pose.position.z - pos_x_last) / dt) 

        #data_set["rot_x"][1].append((pose.orientation.x - rot_x_last) / dt)
        #data_set["rot_y"][1].append((pose.orientation.y - rot_x_last) / dt) 
        #data_set["rot_z"][1].append((pose.orientation.z - rot_x_last) / dt)

        # parse twist
        data_set["pos_x"][2].append(twist.linear.x)
        data_set["pos_y"][2].append(twist.linear.y) 
        data_set["pos_z"][2].append(twist.linear.z) 

        data_set["rot_x"][2].append(twist.angular.x)
        data_set["rot_y"][2].append(twist.angular.y) 
        data_set["rot_z"][2].append(twist.angular.z)

    return data_set


def plot_and_save_data_set(twist, data_set):
    # set up date
    today = datetime.date.today()

    # make directory
    twist_string = "_".join([str(twist.linear.x),
                             str(twist.linear.y),
                             str(twist.linear.z),
                             str(twist.angular.x),
                             str(twist.angular.y),
                             str(twist.angular.z)])
    path = os.path.join(rootdir, "twist_"+twist_string+"_"+str(today))
    os.mkdir(path)

    time = data_set["time"]

    # save data set
    with open(os.path.join(path, "data_set"), 'w') as data_file:
        data_file.write(str(data_set))

    # plot data
    for name, data in data_set.iteritems():
        if name != "time":
            plt.plot(time, data[0])#, 'r-', time, data[2], 'g-')#, time, data[1], 'b--') 
            plt.savefig(os.path.join(path, str(today)+"_"+name+".pdf"), format="pdf")
            print "Plot %s saved"%name
            plt.clf()




def print_feedback(feedback):
    print "Feedback: ", feedback


if __name__ == '__main__':
    main()
