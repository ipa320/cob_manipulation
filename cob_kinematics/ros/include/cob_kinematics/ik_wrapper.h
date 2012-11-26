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
 *   IK/FK wrapper with support for multiple IK links at the end-effector
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

#ifndef I_IK_WRAPPER_H
#define I_IK_WRAPPER_H

#include <ros/ros.h>

#include <urdf/model.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/RobotState.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/tree.hpp>

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

class FkLookup{
public:    
    class ChainFK{
	const std::string root_name;
	const std::string tip_name;
	std::map<std::string, unsigned int> joints;
	KDL::ChainFkSolverPos *solver;
    public:
        ChainFK():solver(0){}
        ChainFK(const KDL::Tree &tree, const std::string &root, const std::string &tip);
        virtual ~ChainFK(){ if (solver) { delete solver; solver = 0; } }
	bool parseJointState(const sensor_msgs::JointState &state_in, KDL::JntArray &array_out, std::map<std::string, unsigned int> &missing) const;
	bool getFK(const KDL::JntArray &joints, KDL::Frame & frame_out) const { return solver != 0 && solver->JntToCart(joints,frame_out) == 0; }
	const std::string& getTip() const { return tip_name; }
	const std::string& getRoot() const { return root_name; }
	void getJoints(std::vector<std::string> &v) const;
    };
    bool addRoot(const urdf::Model &model, const std::string &root);
    const ChainFK * getChain(const std::string &tip);
private:    
    typedef std::map<std::string, boost::shared_ptr<ChainFK> > chain_map;
    typedef std::map<std::string, boost::shared_ptr<KDL::Tree> > tree_map;
    chain_map chains;
    tree_map roots;
    std::map<std::string, std::string> tips;
};

class IKWrapper{   
    FkLookup fk_lookup;
    arm_navigation_msgs::RobotState robot_state;
    
public:
    IKWrapper(const urdf::Model &model, const std::vector<std::string> root_names,const std::vector<std::string> tip_names);

    void updateRobotState(const arm_navigation_msgs::RobotState &rs){
        robot_state = rs; // no lock -> lock it externally!
    }

    int transformPositionIKRequest(kinematics_msgs::PositionIKRequest &request, const geometry_msgs::Pose *pose = 0);
    void getRoots(const std::vector<std::string> &links, std::vector<std::string> &roots_links);
    bool getJoints(const std::vector<std::string> &links, std::vector<std::string> &joints);
		     
    bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                     kinematics_msgs::GetPositionFK::Response &response, ros::ServiceClient &get_fk_client);
};
#endif /* !I_IK_WRAPPER_H */
