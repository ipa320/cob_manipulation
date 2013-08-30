/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta, David Lu!!, Ugo Cupcic
*********************************************************************/

//#include <pluginlib/class_list_macros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <cob_kinematics_special/special_kdl_kinematics_plugin.h>
#include <class_loader/class_loader.h>

#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

//register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(cob_kinematics_special::SpecialKDLKinematicsPlugin, kinematics::KinematicsBase)

//PLUGINLIB_DECLARE_CLASS(cob_kinematics_special, special_kdl_kinematics_plugin, cob_kinematics_special::SpecialKDLKinematicsPlugin, kdl_kinematics_plugin::KDLKinematicsPlugin)




namespace cob_kinematics_special
{

  SpecialKDLKinematicsPlugin::SpecialKDLKinematicsPlugin():active_(false) {}

bool SpecialKDLKinematicsPlugin::initialize(const std::string &robot_description,
                                     const std::string& group_name,
                                     const std::string& base_frame,
                                     const std::string& tip_frame,
                                     double search_discretization)
{
  //const std::string special_robot_description = "special_robot_description";
  //setValues(special_robot_description, group_name, base_frame, tip_frame, search_discretization);
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

  ros::NodeHandle private_handle("~");
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  //rdf_loader::RDFLoader special_rdf_loader(special_robot_description);
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();
  //const boost::shared_ptr<urdf::ModelInterface>& urdf_model = special_rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("kdl","URDF and SRDF must be loaded for KDL kinematics solver to work.");
    return false;
  }

  kinematic_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  if(!kinematic_model_->hasJointModelGroup(group_name))
  {
    ROS_ERROR_NAMED("kdl","Kinematic model does not contain group %s", group_name.c_str());
    return false;
  }
  robot_model::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
  if(!joint_model_group->isChain())
  {
    ROS_ERROR_NAMED("kdl","Group '%s' is not a chain", group_name.c_str());
    return false;
  }

  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR_NAMED("kdl","Could not initialize tree object");
    return false;
  }
  if (!kdl_tree.getChain(base_frame_, tip_frame_, kdl_chain_))
  {
    ROS_ERROR_NAMED("kdl","Could not initialize chain object");
    return false;
  }

  dimension_ = joint_model_group->getVariableCount();
  ik_chain_info_.joint_names = joint_model_group->getJointModelNames();
  ik_chain_info_.limits = joint_model_group->getVariableLimits();
  fk_chain_info_.joint_names = ik_chain_info_.joint_names;
  fk_chain_info_.limits = ik_chain_info_.limits;

  if(!joint_model_group->hasLinkModel(tip_frame_))
  {
    ROS_ERROR_NAMED("kdl","Could not find tip name in joint group '%s'", group_name.c_str());
    return false;
  }
  ik_chain_info_.link_names.push_back(tip_frame_);
  fk_chain_info_.link_names = joint_model_group->getLinkModelNames();

  joint_min_.resize(ik_chain_info_.limits.size());
  joint_max_.resize(ik_chain_info_.limits.size());

  for(unsigned int i=0; i < ik_chain_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_chain_info_.limits[i].min_position;
    joint_max_(i) = ik_chain_info_.limits[i].max_position;
  }

  // Get Solver Parameters
  int max_solver_iterations;
  double epsilon;
  bool position_ik;

  private_handle.param("max_solver_iterations", max_solver_iterations, 500);
  private_handle.param("epsilon", epsilon, 1e-5);
  private_handle.param(group_name+"/position_only_ik", position_ik, false);
  ROS_DEBUG_NAMED("kdl","Looking in private handle: %s for param name: %s",
            private_handle.getNamespace().c_str(),
            (group_name+"/position_only_ik").c_str());

  if(position_ik)
    ROS_INFO_NAMED("kdl","Using position only ik");

  num_possible_redundant_joints_ = kdl_chain_.getNrOfJoints() - joint_model_group->getMimicJointModels().size() - (position_ik? 3:6);

  // Check for mimic joints
  bool has_mimic_joints = joint_model_group->getMimicJointModels().size() > 0;
  std::vector<unsigned int> redundant_joints_map_index;

  std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints;
  unsigned int joint_counter = 0;
  for(std::size_t i=0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    //first check whether it belongs to the set of active joints in the group
    if(joint_model_group->isActiveDOF(kdl_chain_.segments[i].getJoint().getName()))
    {
      kdl_kinematics_plugin::JointMimic mimic_joint;
      mimic_joint.reset(joint_counter);
      mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
      mimic_joint.active = true;
      mimic_joints.push_back(mimic_joint);
      ++joint_counter;
      continue;
    }
    if(joint_model_group->hasJointModel(kdl_chain_.segments[i].getJoint().getName()))
    {
      if(joint_model_group->getJointModel(kdl_chain_.segments[i].getJoint().getName())->getMimic())
      {
        kdl_kinematics_plugin::JointMimic mimic_joint;
        mimic_joint.reset(joint_counter);
        mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
        mimic_joint.offset = joint_model_group->getJointModel(kdl_chain_.segments[i].getJoint().getName())->getMimicOffset();
        mimic_joint.multiplier = joint_model_group->getJointModel(kdl_chain_.segments[i].getJoint().getName())->getMimicFactor();
        mimic_joints.push_back(mimic_joint);
        continue;
      }
    }
  }
  for(std::size_t i=0; i < mimic_joints.size(); ++i)
  {
    if(!mimic_joints[i].active)
    {
      const robot_model::JointModel* joint_model = joint_model_group->getJointModel(mimic_joints[i].joint_name)->getMimic();
      for(std::size_t j=0; j < mimic_joints.size(); ++j)
      {
        if(mimic_joints[j].joint_name == joint_model->getName())
        {
          mimic_joints[i].map_index = mimic_joints[j].map_index;
        }
      }
    }
  }
  mimic_joints_ = mimic_joints;

  // Setup the joint state groups that we need
  kinematic_state_.reset(new robot_state::RobotState((const robot_model::RobotModelConstPtr) kinematic_model_));
  kinematic_state_2_.reset(new robot_state::RobotState((const robot_model::RobotModelConstPtr) kinematic_model_));

  // Store things for when the set of redundant joints may change
  position_ik_ = position_ik;
  joint_model_group_ = joint_model_group;
  max_solver_iterations_ = max_solver_iterations;
  epsilon_ = epsilon;

  active_ = true;
  ROS_DEBUG_NAMED("kdl","KDL solver initialized");
  return true;
}


///private functions copied from KDLKinematicsPlugin

bool SpecialKDLKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);
}

bool SpecialKDLKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                           const std::vector<double> &consistency_limits,
                                           const KDL::JntArray& solution) const
{
  std::vector<double> seed_state_vector(dimension_), solution_vector(dimension_);
  for(std::size_t i = 0; i < dimension_; ++i)
  {
    seed_state_vector[i] = seed_state(i);
    solution_vector[i] = solution(i);
  }
  robot_state::JointStateGroup* joint_state_group = kinematic_state_->getJointStateGroup(getGroupName());
  robot_state::JointStateGroup* joint_state_group_2 = kinematic_state_2_->getJointStateGroup(getGroupName());
  joint_state_group->setVariableValues(seed_state_vector);
  joint_state_group_2->setVariableValues(solution_vector);

  const std::vector<robot_state::JointState*>& joint_state_vector = joint_state_group->getJointStateVector();
  const std::vector<robot_state::JointState*>& joint_state_vector_2 = joint_state_group_2->getJointStateVector();

  for(std::size_t i = 0; i < joint_state_vector.size(); ++i)
  {
    if(joint_state_vector[i]->distance(joint_state_vector_2[i]) > consistency_limits[i])
      return false;
  }

  return true;
}

int SpecialKDLKinematicsPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_chain_info_.joint_names.size(); i++) {
    if (ik_chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int SpecialKDLKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  int i=0;
  while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
      return i+1;
    }
    i++;
  }
  return -1;
}

void SpecialKDLKinematicsPlugin::getRandomConfiguration(KDL::JntArray &jnt_array,
                                                 bool lock_redundancy) const
{
  std::vector<double> jnt_array_vector(dimension_,0.0);
  robot_state::JointStateGroup*  joint_state_group = kinematic_state_->getJointStateGroup(getGroupName());
  joint_state_group->setToRandomValues();
  joint_state_group->getVariableValues(jnt_array_vector);
  for(std::size_t i=0; i < dimension_; ++i)
  {
    if(lock_redundancy)
      if(isRedundantJoint(i))
        continue;
    jnt_array(i) = jnt_array_vector[i];
  }
}

void SpecialKDLKinematicsPlugin::getRandomConfiguration(const KDL::JntArray &seed_state,
                                                 const std::vector<double> &consistency_limits,
                                                 KDL::JntArray &jnt_array,
                                                 bool lock_redundancy) const
{
  std::vector<double> values, near;
  for(std::size_t i=0; i < dimension_; ++i)
  {
    near.push_back(seed_state(i));
  }
  robot_state::JointStateGroup*  joint_state_group = kinematic_state_->getJointStateGroup(getGroupName());
  joint_state_group->setToRandomValuesNearBy(near, consistency_limits);
  joint_state_group->getVariableValues(values);
  for(std::size_t i=0; i < dimension_; ++i)
  {
    if(lock_redundancy)
      for(std::size_t j=0; j < redundant_joint_indices_.size(); ++j)
        if(redundant_joint_indices_[j] == i)
          continue;
    jnt_array(i) = values[i];
  }
}

bool SpecialKDLKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for(std::size_t j=0; j < redundant_joint_indices_.size(); ++j)
    if(redundant_joint_indices_[j] == index)
      return true;
  return false;
}



} // namespace
