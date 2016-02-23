#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh


if __name__ == "__main__":
    rospy.init_node("simple_obstacle_pub")
    root_frame = "base_link"

    pub = rospy.Publisher("/collision_object", CollisionObject, queue_size = 1)
    rospy.sleep(1.0)

    # Publish a simple sphere
    x = CollisionObject()
    x.id = "sphere"
    x.header.frame_id = root_frame
    x.operation = CollisionObject.ADD
    #x.operation = CollisionObject.REMOVE
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions.append(0.1)  # radius
    x.primitives.append(sphere)

    pose = Pose()
    pose.position.x = 0.35
    pose.position.y = -0.45
    pose.position.z = 0.8
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    x.primitive_poses.append(pose)
    pub.publish(x)
    rospy.sleep(1.0)
    
    rospy.loginfo("Done")
