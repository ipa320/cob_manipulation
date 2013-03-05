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
 *   Wrapper for kinematics::KinematicsBase that backports interface from fuerte to electric
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

#ifndef KINEMATICS_BASE_WRAPPER_H_ 
#define KINEMATICS_BASE_WRAPPER_H_

#include <ros/ros.h>

#ifdef ROS_VERSION_ELECTRIC

#define WRAPPER_DECLARE_GET_VECTOR(name) std::vector<std::string> name() 

#ifndef WRAPPER_DEFAULT_SD
#define WRAPPER_DEFAULT_SD (0.05)
#endif

#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/RobotState.h>

#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <boost/function.hpp>

namespace kinematics{
    static const int SUCCESS = 1;
    static const int TIMED_OUT = -1;
    static const int NO_IK_SOLUTION = -2;
    static const int FRAME_TRANSFORM_FAILURE = -3;
    static const int IK_LINK_INVALID = -4;
    static const int IK_LINK_IN_COLLISION = -5;
    static const int STATE_IN_COLLISION = -6;
    static const int INVALID_LINK_NAME = -7;
    static const int GOAL_CONSTRAINTS_VIOLATED = -7;
    static const int INACTIVE = -8;

    namespace wrapper{
#       include <kinematics_base/kinematics_base.h>
    }
    class KinematicsBase : public wrapper::kinematics::KinematicsBase{
    public:
      /**
       * @brief  Initialization function for the kinematics
       * @return True if initialization was successful, false otherwise
       */
      virtual void setValues(const std::string& group_name,
                             const std::string& base_name,
                             const std::string& tip_name,
                             const double& search_discretization) {
        group_name_ = group_name;
        base_name_ = base_name;
        tip_name_ = tip_name;
        search_discretization_ = search_discretization;
      }

      virtual bool initialize(const std::string& group_name,
                              const std::string& base_name,
                              const std::string& tip_name,
                              const double& search_discretization) = 0;

      /**
       * @brief  Return the frame in which the kinematics is operating
       * @return the string name of the frame in which the kinematics is operating
       */
      virtual const std::string& getGroupName() const {
        return group_name_;    
      }

      virtual const std::string& getBaseName() const {
        return base_name_;
      }

      /**
       * @brief  Return the links for which kinematics can be computed
       */
      virtual const std::string& getTipName() const {
        return tip_name_;
      }
      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~KinematicsBase(){}

      void setSearchDiscretization(double sd) {
        search_discretization_ = sd;
      }

      double getSearchDiscretization() const {
        return search_discretization_;
      }

      // ELECTRIC
      virtual bool initialize(std::string name){
        ros::NodeHandle private_handle("~"+name);
        std::string root_name, tip_name; 
        if (!private_handle.getParam("root_name", root_name)) {
            ROS_FATAL("GenericIK: No root name found on parameter server");
            return false;
        }
        if (!private_handle.getParam("tip_name", tip_name)) {
            ROS_FATAL("GenericIK: No tip name found on parameter server");
            return false;
        }
        return initialize(name, root_name, tip_name,  WRAPPER_DEFAULT_SD);
        
      }

      /**
       * @brief  Return the frame in which the kinematics is operating
       * @return the string name of the frame in which the kinematics is operating
       */
      virtual std::string getBaseFrame() { return getBaseName(); }

      /**
       * @brief  Return the links for which kinematics can be computed
       */
      virtual std::string getToolFrame() { return getTipName(); }

    protected:
      std::string group_name_;
      std::string base_name_;
      std::string tip_name_;
      double search_discretization_;
      KinematicsBase(){}
    };
}
#else
#define WRAPPER_DECLARE_GET_VECTOR(name) const std::vector<std::string>& name() const 
#include <kinematics_base/kinematics_base.h>
#endif

#endif
