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


import threading
from copy import deepcopy

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
import tf





_transform_listener = None
_transform_listener_creation_lock = threading.Lock()

_mgc_dict = dict()
_mgc_dict_creation_lock = threading.Lock()

_psi = None
_psi_creation_lock = threading.Lock()

def get_transform_listener():
    '''
    Gets the transform listener for this process.
    This is needed because tf only allows one transform listener per process. Threadsafe, so
    that two threads could call this at the same time, at it will do the right thing.
    '''
    global _transform_listener
    with _transform_listener_creation_lock:
        if _transform_listener == None:
            _transform_listener = tf.TransformListener()
        return _transform_listener



def get_move_group_commander(group):
    '''
    Gets the move_group_commander for this process.
    '''
    global _mgc_dict
    with _mgc_dict_creation_lock:
        if not group in _mgc_dict:
            _mgc_group = MoveGroupCommander(group)
            _mgc_group.set_planner_id('RRTConnectkConfigDefault')
            _mgc_dict[group] = _mgc_group

    add_ground()
    return _mgc_dict[group]



def get_planning_scene_interface():
    '''
    Gets the planning_scene_interface for this process.
    '''
    global _psi
    with _psi_creation_lock:
        if _psi == None:
            _psi = PlanningSceneInterface()
        return _psi


def add_ground():
    psi = get_planning_scene_interface()
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    # offset such that the box is 0.1 mm below ground (to prevent collision with the robot itself)
    pose.pose.position.z = -0.0501
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = "base_link"
    psi.attach_box("base_link", "ground", pose, (3, 3, 0.1))

#ToDo: attached objects are not removed
def clear_objects():
    psi = get_planning_scene_interface()
    psi.remove_world_object("")

def clear_attached_object(attach_link, object_name=None):
    psi = get_planning_scene_interface()
    psi.remove_attached_object(link = attach_link, name = object_name)
    psi.remove_world_object(object_name)


def attach_mesh_to_link(link, name, path):
    psi = get_planning_scene_interface()
    pose = PoseStamped()
    pose.pose.orientation.w = 1
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = link
    psi.attach_mesh(link, name, pose, path)

def moveit_joint_goal(group, goal, replanning=False):
    mgc = get_move_group_commander(group)
    mgc.allow_replanning(replanning)
    if mgc.go(goal):
        print("Done moving")
        return 'succeeded'
    else:
        print("Failed to plan path")
        return 'failed'

def moveit_pose_goal(group, ref_frame, goal, replan=False):
    mgc = get_move_group_commander(group)
    mgc.allow_replanning(replan)
    mgc.set_pose_reference_frame(ref_frame)
    ret = mgc.go(goal)
    mgc.allow_replanning(False)
    if ret:
        print("Done moving")
        return 'succeeded'
    else:
        print("Failed to plan path")
        return 'failed'



#this is linear movement
def moveit_cart_goals(group, ref_frame, goal_list, avoid_collisions=True):
    mgc = get_move_group_commander(group)

    mgc.set_pose_reference_frame(ref_frame)
    (traj,frac)  = mgc.compute_cartesian_path(goal_list, 0.01, 4, avoid_collisions)
    print(traj,frac)

    #mgc.execute(traj)
    #print "Done moving"
    #return 'succeeded'

    if frac == 1.0:
        if mgc.execute(traj):
            print("Done moving")
            return 'succeeded'
        else:
            print("Something happened during execution")
            return 'failed'
    else:
        print("Failed to plan full path!")
        return 'failed'


def moveit_get_current_pose(group):
    mgc = get_move_group_commander(group)

    cp = mgc.get_current_pose()
    cp.header.stamp = rospy.Time()
    return cp







####################################
#  helpers
####################################

def get_goal_from_server(group, parameter_name):
        ns_global_prefix = "/script_server"

        # get joint_names from parameter server
        param_string = ns_global_prefix + "/" + group + "/joint_names"
        if not rospy.has_param(param_string):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
                return None
        joint_names = rospy.get_param(param_string)

        # check joint_names parameter
        if not type(joint_names) is list: # check list
                rospy.logerr("no valid joint_names for %s: not a list, aborting...",group)
                print("joint_names are:",joint_names)
                return None
        else:
            for i in joint_names:
                if not type(i) is str: # check string
                    rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",group)
                    print("joint_names are:",joint_names)
                    return None
                else:
                    rospy.logdebug("accepted joint_names for group %s",group)

        # get joint values from parameter server
        if type(parameter_name) is str:
            if not rospy.has_param(ns_global_prefix + "/" + group + "/" + parameter_name):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",ns_global_prefix + "/" + group + "/" + parameter_name)
                return None
            param = rospy.get_param(ns_global_prefix + "/" + group + "/" + parameter_name)
        else:
            param = parameter_name

        # check trajectory parameters
        if not type(param) is list: # check outer list
                rospy.logerr("no valid parameter for %s: not a list, aborting...",group)
                print("parameter is:",param)
                return None

        #no need for trajectories anymore, since planning (will) guarantee collision-free motion!
        point = param[len(param)-1]

        #print point,"type1 = ", type(point)
        if type(point) is str:
            if not rospy.has_param(ns_global_prefix + "/" + group + "/" + point):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",ns_global_prefix + "/" + group + "/" + point)
                return None
            point = rospy.get_param(ns_global_prefix + "/" + group + "/" + point)
            point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
            #print point
        elif type(point) is list:
            rospy.logdebug("point is a list")
        else:
            rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",group)
            print("parameter is:",param)
            return None

        # here: point should be list of floats/ints
        #print point
        if not len(point) == len(joint_names): # check dimension
            rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",group,len(joint_names),len(point))
            print("parameter is:",param)
            return None

        for value in point:
            #print value,"type2 = ", type(value)
            if not ((type(value) is float) or (type(value) is int)): # check type
                #print type(value)
                rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",group)
                print("parameter is:",param)
                return None
            rospy.logdebug("accepted value %f for %s",value,group)

        return point











