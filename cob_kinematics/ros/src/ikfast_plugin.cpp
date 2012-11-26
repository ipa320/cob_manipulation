/*!
 *****************************************************************
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   ROS stack name: cob_manipulation
 * \note
 *   ROS package name: cob_kinematics
 *
 * \author
 *   Author: Mathias Lüdtke
 *
 * \brief
 *   IKfast ROS kinematics plug-in that integrates auto-generated analytical IK solvers
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ros/ros.h>

#include <cob_kinematics/kinematics_base_wrapper.h>

namespace IKFAST_NAMESPACE {
 
class IKFastPlugin: public kinematics::KinematicsBase{
    public:
      /**
       * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
       * @param ik_link_name - the name of the link for which IK is being computed
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
      virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                                 const std::vector<double> &ik_seed_state,
                                 std::vector<double> &solution,
                                 int &error_code) {}      

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    const double &timeout,
                                    std::vector<double> &solution,
                                    int &error_code) {}      

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @param the distance that the redundancy can be from the current position 
       * @return True if a valid solution was found, false otherwise
       */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    const double &timeout,
                                    const unsigned int& redundancy,
                                    const double &consistency_limit,
                                    std::vector<double> &solution,
                                    int &error_code) {}      

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    const double &timeout,
                                    std::vector<double> &solution,
                                    const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                    const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                    int &error_code) {}      

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
       * around those specified in the seed state are admissible and need to be searched.
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @param consistency_limit the distance that the redundancy can be from the current position 
       * @return True if a valid solution was found, false otherwise
       */
      virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    const double &timeout,
                                    const unsigned int& redundancy,
                                    const double &consistency_limit,
                                    std::vector<double> &solution,
                                    const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                    const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                    int &error_code) {}

      /**
       * @brief Given a set of joint angles and a set of links, compute their pose
       * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
       * @param response - the response contains stamped pose information for all the requested links
       * @return True if a valid solution was found, false otherwise
       */
       virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                  const std::vector<double> &joint_angles, 
                                  std::vector<geometry_msgs::Pose> &poses){
                                  }

       /**
       * @brief  Initialization function for the kinematics
       * @return True if initialization was successful, false otherwise
       */
       virtual bool initialize(const std::string& group_name,
                              const std::string& base_name,
                              const std::string& tip_name,
                              const double& search_discretization){
            setValues(group_name, base_name, tip_name, search_discretization);
            // TODO parse stuff ;)
       }
                              
      /**
       * @brief  Return all the joint names in the order they are used internally
       */
      virtual WRAPPER_DECLARE_GET_VECTOR(getJointNames) {}

      /**
       * @brief  Return all the link names in the order they are represented internally
       */
      virtual WRAPPER_DECLARE_GET_VECTOR(getLinkNames) {}


      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~IKFastPlugin(){}
  };

}
#include <pluginlib/class_list_macros.h>

#define ADD_TYPED_PLUGIN(type,name) PLUGINLIB_DECLARE_CLASS(cob_kinematics, type##name, IKFAST_NAMESPACE::IKFastPlugin, kinematics::KinematicsBase)
#define ADD_IKFAST_PLUGIN(name) ADD_TYPED_PLUGIN(IKFast_, name)
ADD_IKFAST_PLUGIN(IKFAST_NAMESPACE); 

