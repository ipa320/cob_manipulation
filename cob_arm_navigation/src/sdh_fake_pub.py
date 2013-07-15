#!/usr/bin/env python

import roslib; roslib.load_manifest("cob_arm_navigation")
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("sdh_fake_pub")
p = rospy.Publisher('joint_states', JointState)

msg = JointState()
msg.name = ['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_21_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
msg.position = [0,-1.57,-1.57,0,-1.57,-1.57,-1.57,-1.57]#[-1.57 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.1)

