#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_grasp_generation')
import rospy

import actionlib
import manipulation_msgs.msg
import cob_grasp_generation.msg
from cob_grasp_generation import or_grasp_generation 

class ShowGraspServer(object):
  # create messages that are used to publish feedback/result
  _feedback = cob_grasp_generation.msg.ShowGraspFeedback()
  _result   = cob_grasp_generation.msg.ShowGraspResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, cob_grasp_generation.msg.ShowGraspAction, execute_cb=self.execute_cb)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = False
    grasp_list = []
    
    # publish info to the console for the user
    rospy.loginfo('%s: Opening qtcoin-Viewer to show grasp for object %s with ID %i.' % (self._action_name, goal.object_name, goal.grasp_id))
    
    # Show the Grasps
    rospy.sleep(2.0)
    
    #check if database of object is available
    if or_grasp_generation.check_database(goal.object_name):
	rospy.loginfo('Display Grasp. Object: %s | ID: %i' % (goal.object_name, goal.grasp_id))
    	or_grasp_generation.show_grasp(goal.object_name, goal.grasp_id)
	success = True
    else:
	rospy.logerr('Database for Object %s does not exist!' % (goal.object_name))	

    #Fill result
    self._result.success = success
    
    #Set action state
    if success:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    else:
      rospy.logwarn('%s: Failed' % self._action_name)
      self._as.set_aborted(self._result)
      
if __name__ == '__main__':
  rospy.init_node('show_grasp')
  ShowGraspServer('show_grasp')
  rospy.spin()
