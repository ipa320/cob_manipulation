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

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
#include "ikfast.h"

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <urdf/model.h>

namespace IKFAST_NAMESPACE {

class IKFastPlugin: public kinematics::KinematicsBase {
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
            std::vector<double> &solution, int &error_code) {
    }

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
            const std::vector<double> &ik_seed_state, const double &timeout,
            std::vector<double> &solution, int &error_code) {
    }

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
            const std::vector<double> &ik_seed_state, const double &timeout,
            const unsigned int& redundancy, const double &consistency_limit,
            std::vector<double> &solution, int &error_code) {
    }

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(
            const geometry_msgs::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            const double &timeout,
            std::vector<double> &solution,
            const boost::function<void(const geometry_msgs::Pose &ik_pose,
                    const std::vector<double> &ik_solution, int &error_code)> &desired_pose_callback,
            const boost::function<void(const geometry_msgs::Pose &ik_pose,
                    const std::vector<double> &ik_solution, int &error_code)> &solution_callback,
            int &error_code) {
    }

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
    virtual bool searchPositionIK(
            const geometry_msgs::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            const double &timeout,
            const unsigned int& redundancy,
            const double &consistency_limit,
            std::vector<double> &solution,
            const boost::function<void(const geometry_msgs::Pose &ik_pose,
                    const std::vector<double> &ik_solution, int &error_code)> &desired_pose_callback,
            const boost::function<void(const geometry_msgs::Pose &ik_pose,
                    const std::vector<double> &ik_solution, int &error_code)> &solution_callback,
            int &error_code) {
    }

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose
     * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
     * @param response - the response contains stamped pose information for all the requested links
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool getPositionFK(const std::vector<std::string> &link_names,
            const std::vector<double> &joint_angles, std::vector<
                    geometry_msgs::Pose> &poses) {
    }

    /**
     * @brief  Initialization function for the kinematics
     * @return True if initialization was successful, false otherwise
     */
    virtual bool initialize(const std::string& group_name,
            const std::string& base_name, const std::string& tip_name,
            const double& search_discretization) {
        setValues(group_name, base_name, tip_name, search_discretization);

        links_.resize(1);
        links_[0] = tip_name_;

        std::string urdf_xml, full_urdf_xml;
        ros::NodeHandle node_handle;
        node_handle.param("urdf_xml", urdf_xml,
                std::string("robot_description"));
        node_handle.searchParam(urdf_xml, full_urdf_xml);
        ROS_DEBUG("Reading xml file from parameter server");
        std::string result;
        if (!node_handle.getParam(full_urdf_xml, result)) {
            ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
            return false;
        }
        // Load and Read Models
        if (!loadModel(result)) {
            ROS_FATAL("Could not load models!");
            return false;
        }
        indices_.clear();
        for (int i = 0; i < GetNumFreeParameters(); ++i)
            indices_.push_back(GetFreeParameters()[i]);

    }

    /**
     * @brief  Return all the joint names in the order they are used internally
     */
    virtual WRAPPER_DECLARE_GET_VECTOR(getJointNames) {
        return joints_;
    }

    /**
     * @brief  Return all the link names in the order they are represented internally
     */
    virtual WRAPPER_DECLARE_GET_VECTOR(getLinkNames) {
        return links_;
    }

    /**
     * @brief  Virtual destructor for the interface
     */
    virtual ~IKFastPlugin() {
    }
    IKFastPlugin() :
        bounds_epsilon_(0.0) {
    }
protected:
    std::vector<std::pair<double, double> > min_max_;
    double bounds_epsilon_;
    std::vector<double> indices_;
    std::vector<std::string> links_;
    std::vector<std::string> joints_;
    bool loadModel(const std::string xml) {
        urdf::Model robot_model;

        if (!robot_model.initString(xml)) {
            ROS_FATAL("Could not initialize robot model");
            return -1;
        }
        if (!readJoints(robot_model)) {
            ROS_FATAL("Could not read information about the joints");
            return false;
        }
        return true;
    }

    bool readJoints(urdf::Model &robot_model) {
        int dimension_ = 0;
        // get joint maxs and mins
        boost::shared_ptr<const urdf::Link> link = robot_model.getLink(
                tip_name_);
        boost::shared_ptr<const urdf::Joint> joint;
        while (link && link->name != base_name_) {
            joint = robot_model.getJoint(link->parent_joint->name);
            if (!joint) {
                ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
                return false;
            }
            if (joint->type != urdf::Joint::UNKNOWN && joint->type
                    != urdf::Joint::FIXED) {
                ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
                dimension_++;
            }
            link = robot_model.getLink(link->getParent()->name);
        }

        if (dimension_ != GetNumJoints()) {
            ROS_ERROR("Solver needs %d joints, but %d were found.",GetNumJoints(), dimension_);
            return false;
        }

        min_max_.resize(dimension_);
        joints_.resize(dimension_);
        link = robot_model.getLink(tip_name_);

        unsigned int i = 0;
        while (link && i < dimension_) {
            joint = robot_model.getJoint(link->parent_joint->name);
            if (joint->type != urdf::Joint::UNKNOWN && joint->type
                    != urdf::Joint::FIXED) {
                ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

                float lower, upper;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    if (joint->safety) {
                        lower = joint->safety->soft_lower_limit
                                + bounds_epsilon_;
                        upper = joint->safety->soft_upper_limit
                                - bounds_epsilon_;
                    } else {
                        lower = joint->limits->lower + bounds_epsilon_;
                        upper = joint->limits->upper - bounds_epsilon_;
                    }
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                }
                min_max_[dimension_ - i - 1] = std::make_pair(lower, upper);
                joints_[dimension_ - i - 1] = joint->name;
                i++;
            }
            link = robot_model.getLink(link->getParent()->name);
        }

        return true;
    }
};

}
#include <pluginlib/class_list_macros.h>

#define ADD_TYPED_PLUGIN(type,name) PLUGINLIB_DECLARE_CLASS(cob_kinematics, type##name, IKFAST_NAMESPACE::IKFastPlugin, kinematics::KinematicsBase)
#define ADD_IKFAST_PLUGIN(name) ADD_TYPED_PLUGIN(IKFast_, name)
ADD_IKFAST_PLUGIN(IKFAST_NAMESPACE)
;

