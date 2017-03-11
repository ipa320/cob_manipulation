#!/usr/bin/python

import random
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("lookat_target_publisher")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    pose = (random.uniform(0.2, 1.5), random.uniform(-1.0, 1.0), 1.4+random.uniform(-0.5, 0.5))
    while not rospy.is_shutdown():
        br.sendTransform( pose,
                         (0.0, 0.0, 0.0, 1.0),
                          rospy.Time.now(),
                          "lookat_target",
                          "base_link")
        
        try:
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException, e:
            rospy.logwarn("ROSTimeMovedBackwardsException during sleep(). Continue anyway...")
        except rospy.exceptions.ROSInterruptException as e:
            pass
