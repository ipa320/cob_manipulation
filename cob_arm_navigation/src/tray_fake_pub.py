#!/usr/bin/env python

import roslib; roslib.load_manifest("cob_arm_navigation")
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("tray_fake_pub")
p = rospy.Publisher('joint_states', JointState)

msg = JointState()
msg.name = ['tray_1_joint', 'tray_2_joint', 'tray_3_joint']
msg.position = [0.0 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.1)

