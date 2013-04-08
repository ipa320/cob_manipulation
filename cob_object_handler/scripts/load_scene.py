#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_object_handler')
import rospy

from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from geometry_msgs.msg import *
from tf.transformations import *
from cob_object_handler.srv import *

def createObject(shapetype, name, pos, euler, extent):
    co = CollisionObject()
    s = Shape()
    co.id = name
    co.operation.operation = CollisionObjectOperation.ADD
    #co.header.frame_id = "/odom_combined";
    co.header.frame_id = "/base_link";
    co.header.stamp = rospy.Time.now();
    s.type = shapetype
    s.dimensions = list(extent)
    co.shapes = [s]
    p = Pose()
    p.position.x, p.position.y, p.position.z = pos
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quaternion_from_euler(*euler)
    co.poses = [p]
    print co
    return co
   
if __name__ == '__main__':
    rospy.init_node('load_scene')
    while rospy.get_time() == 0.0: pass
    
    #print "Waiting for Service 'addWorld'"
    #rospy.wait_for_service('addWorld')
    #print "...done"
    #try:
    #    addWorld = rospy.ServiceProxy('addWorld', HandleObject)
    #    req = HandleObjectRequest()
    #    req.operation = "add"
    #    res = addWorld(req)
    #except rospy.ServiceException, e:
    #    print "Service call failed: %s"%e
    #print "world added"
        
    #print "aiting for Service 'handle_object'"
    #rospy.wait_for_service('/object_handler/handle_object')
    #print "...done"
    #try:
    #    addObject = rospy.ServiceProxy('/object_handler/handle_object', HandleObject)
    #    req = HandleObjectRequest()
    #    req.operation = "add"
    #    req.object = "table_ikea"
    #    res = addObject(req)
    #except rospy.ServiceException, e:
    #    print "Service call failed: %s"%e
    #print "table_ikea added"
    
    pub = rospy.Publisher('/collision_object', CollisionObject)
    rospy.sleep(1.0)
    pub.publish( createObject(Shape.BOX, "table_ikea", [-0.301, -0.982, 0.375],[0,0,0],[0.9, 0.9, 0.75]) )
    rospy.sleep(1.0)
    #pub.publish( createObject(Shape.CYLINDER, "milk", [-0.8,-0.2,0.9],[0,0,0],[0.05, 0.2]) )
    #rospy.sleep(1.0)
    print rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)(SetPlanningSceneDiffRequest())
