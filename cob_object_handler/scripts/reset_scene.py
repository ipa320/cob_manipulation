#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_manipulation_test')
import rospy

from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *

def removeAll():
    co = CollisionObject()
    co.id = "all"
    co.operation.operation = CollisionObjectOperation.REMOVE
    co.header.frame_id = "/odom_combined";
    co.header.stamp = rospy.Time.now();
    return co
   
if __name__ == '__main__':
    rospy.init_node('load_scene')
    while rospy.get_time() == 0.0: pass
    pub = rospy.Publisher('/collision_object', CollisionObject)
    pub2 = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
    rospy.sleep(1.0)
    pub.publish( removeAll() )
    pub2.publish( AttachedCollisionObject(object=removeAll()) )
    rospy.sleep(1.0)
    print rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)(SetPlanningSceneDiffRequest())
