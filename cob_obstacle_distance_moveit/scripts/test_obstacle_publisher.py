#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh


if __name__ == "__main__":
    rospy.init_node("simple_obstacle_pub")
    root_frame = "torso_center_link"

    pub = rospy.Publisher("/collision_object", CollisionObject, queue_size = 1)
    rospy.sleep(1.0)

    # Publish a simple sphere
    x = CollisionObject()
    x.id = "sphere"
    x.header.frame_id = root_frame
    x.operation = CollisionObject.ADD
    #x.operation = CollisionObject.REMOVE
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.BOX
    sphere.dimensions.append(0.001)  # radius
    sphere.dimensions.append(0.001)  # radius
    sphere.dimensions.append(0.001)  # radius
    x.primitives.append(sphere)

    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    x.primitive_poses.append(pose)
    pub.publish(x)
    rospy.sleep(1.0)
    
    rospy.loginfo("Done")
