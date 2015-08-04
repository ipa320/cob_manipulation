#! /usr/bin/env python

import rospy

import actionlib
import moveit_msgs.msg
import cob_grasp_generation.msg
from cob_grasp_generation import or_grasp_generation 

class CobGraspGenerationActionServer(object):

  def __init__(self):
    self.orgg = or_grasp_generation.ORGraspGeneration()
    
    self._as_generate = actionlib.SimpleActionServer('generate_grasps', cob_grasp_generation.msg.GenerateGraspsAction, execute_cb=self.generate_cb, auto_start = False)
    self._as_generate.start()
    self._as_query = actionlib.SimpleActionServer('query_grasps', cob_grasp_generation.msg.QueryGraspsAction, execute_cb=self.query_cb, auto_start = False)
    self._as_query.start()
    self._as_show = actionlib.SimpleActionServer('show_grasps', cob_grasp_generation.msg.ShowGraspsAction, execute_cb=self.show_cb, auto_start = False)
    self._as_show.start()
    
    print("CobGraspGenerationActionServer: actions started...")

  def generate_cb(self, goal):
    success = False
    num_grasps = 0

    rospy.loginfo('Generating grasps for object %s using gripper_type %s' % (goal.object_name, goal.gripper_type))

    if self.orgg.check_database(goal.object_name, goal.gripper_type):
        rospy.logwarn('Grasps for object %s exist in the database.', goal.object_name)
        success = False 
    else:
        self.orgg.setup_environment(goal.object_name, goal.gripper_type, goal.viewer) 
        rospy.loginfo('GraspTable for object %s does not exist. Now planning Grasps for the object',goal.object_name)
        num_grasps = self.orgg.generate_grasps(goal.object_name, goal.gripper_type, goal.replan) 
        if (num_grasps > 0):
            success = True
    
    result   = cob_grasp_generation.msg.GenerateGraspsResult()
    result.success = success
    result.num_grasps = num_grasps

    if success:
      rospy.loginfo('Generate: Succeeded')
      self._as_generate.set_succeeded(result)
    else:
      rospy.logwarn('Generate: Failed')
      self._as_generate.set_aborted(result)



  def query_cb(self, goal):
    success = False
    grasp_list = []

    rospy.loginfo('Querying grasps for object %s using gripper_type %s' % (goal.object_name, goal.gripper_type))

    if self.orgg.check_database(goal.object_name, goal.gripper_type):
        rospy.loginfo('GraspTable for object %s exist in the database.', goal.object_name)
        rospy.loginfo('Returning grasp list for selected object.')
        grasp_list = self.orgg.get_grasps(goal.object_name, goal.gripper_type, goal.grasp_id, goal.num_grasps, goal.threshold)
    else:
        rospy.logwarn('GraspTable for object %s does not exist!',goal.object_name)

    if not (grasp_list == []):
        success = True
    
    result   = cob_grasp_generation.msg.QueryGraspsResult()
    result.success = success
    result.grasp_list = grasp_list

    if success:
      rospy.loginfo('Query: Succeeded')
      self._as_query.set_succeeded(result)
    else:
      rospy.logwarn('Query: Failed')
      self._as_query.set_aborted(result)


  def show_cb(self, goal):
    success = False
    
    rospy.loginfo('Show grasp %i for object %s using gripper_type %s' % (goal.grasp_id, goal.object_name, goal.gripper_type))

    if self.orgg.check_database(goal.object_name, goal.gripper_type):
      self.orgg.setup_environment(goal.object_name, goal.gripper_type, viewer=True)
      rospy.loginfo('Display Grasp. Object: %s | ID: %i' % (goal.object_name, goal.grasp_id))
      self.orgg.show_grasp(goal.object_name, goal.gripper_type, goal.grasp_id, goal.sort_by_quality)
      success = True
    else:
      rospy.logerr('GraspTable for Object %s does not exist!' % (goal.object_name))

    result   = cob_grasp_generation.msg.ShowGraspsResult()
    result.success = success

    if success:
      rospy.loginfo('Show: Succeeded')
      self._as_show.set_succeeded(result)
    else:
      rospy.logwarn('Show: Failed')
      self._as_show.set_aborted(result)

      
if __name__ == '__main__':
  rospy.init_node('generate_grasps')
  CobGraspGenerationActionServer()
  rospy.spin()
