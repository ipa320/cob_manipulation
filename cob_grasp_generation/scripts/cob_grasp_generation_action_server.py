#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_grasp_generation')
import rospy

import actionlib
import manipulation_msgs.msg
import cob_grasp_generation.msg
from cob_grasp_generation import or_grasp_generation 

class CobGraspGenerationActionServer(object):
  # create messages that are used to publish feedback/result
  _feedback = cob_grasp_generation.msg.GenerateGraspsFeedback()
  _result   = cob_grasp_generation.msg.GenerateGraspsResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, cob_grasp_generation.msg.GenerateGraspsAction, execute_cb=self.execute_cb)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = False
    grasp_list = []
    
    # publish info to the console for the user
    rospy.loginfo('%s: Trying to get some grasps for object: >> %s <<' % (self._action_name, goal.object_name))
      
    # Do Witalij's fancy grasp_generation
    rospy.sleep(2.0)
    
    #check if database of object is available
    if or_grasp_generation.check_database(goal.object_name):
		rospy.loginfo('Grasps for object %s exist in the database.', goal.object_name)
		rospy.loginfo('Returning grasp list for selected object.')
		#return grasp_list
		grasp_list = or_grasp_generation.get_grasps(goal.object_name)
    else:
	#plan first, then return grasp list
    	rospy.loginfo('Database for object %s does not exist. Now planning Grasps for the object',goal.object_name)
    	or_grasp_generation.generate_grasps(goal.object_name)
    	grasp_list = or_grasp_generation.get_grasps(goal.object_name)
    	
    #Fill result
    self._result.success = success
    self._result.grasp_list = grasp_list
    
    #check if grasp_list is filled and return success
    if len(grasp_list) > 1:
		success = True
    
    #Set action state
    if success:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    else:
      rospy.logwarn('%s: Failed' % self._action_name)
      self._as.set_aborted(self._result)
      
if __name__ == '__main__':
  rospy.init_node('generate_grasps')
  CobGraspGenerationActionServer('generate_grasps')
  rospy.spin()
