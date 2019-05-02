/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#include <ros/ros.h>
#include <cob_lookat_action/cob_lookat_action_server.h>


bool CobLookAtAction::init()
{
    /// get parameters from parameter server
    if (!nh_.getParam("joint_names", joint_names_))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }

    if (!nh_.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }

    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    /// parse robot_description and generate KDL chains
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("/robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    tree.getChain(chain_base_link_, chain_tip_link_, chain_main_);
    if (chain_main_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }


    ROS_WARN_STREAM("Waiting for ActionServer: " << fjt_name_);
    fjt_ac_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_, fjt_name_, true);
    fjt_ac_->waitForServer(ros::Duration(10.0));

    lookat_as_ = new actionlib::SimpleActionServer<cob_lookat_action::LookAtAction>(nh_, lookat_name_, boost::bind(&CobLookAtAction::goalCB, this, _1), false);
    lookat_as_->start();

    return true;
}


void CobLookAtAction::goalCB(const cob_lookat_action::LookAtGoalConstPtr &goal)
{
    bool success = true;
    std::string message;

    /// set up lookat chain
    KDL::Chain chain_lookat, chain_full;
    KDL::Vector lookat_lin_axis(0.0, 0.0, 0.0);
    switch (goal->pointing_axis_type)
    {
        case cob_lookat_action::LookAtGoal::X_POSITIVE:
            lookat_lin_axis.x(1.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_POSITIVE:
            lookat_lin_axis.y(1.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_POSITIVE:
            lookat_lin_axis.z(1.0);
            break;
        case cob_lookat_action::LookAtGoal::X_NEGATIVE:
            lookat_lin_axis.x(-1.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_NEGATIVE:
            lookat_lin_axis.y(-1.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_NEGATIVE:
            lookat_lin_axis.z(-1.0);
            break;
        default:
            ROS_ERROR("PointingAxisType %d not defined! Using default: 'X_POSITIVE'!", goal->pointing_axis_type);
            lookat_lin_axis.x(1.0);
            break;
    }
    KDL::Joint lookat_lin_joint("lookat_lin_joint", KDL::Vector(), lookat_lin_axis, KDL::Joint::TransAxis);

    /// transform pointing_frame to offset
    KDL::Frame offset;
    tf::StampedTransform offset_transform;
    bool transformed = false;

    do
    {
        try
        {
            tf_listener_.lookupTransform(chain_tip_link_, goal->pointing_frame, ros::Time(0), offset_transform);
            transformed = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("LookatAction: %s", ex.what());
            ros::Duration(0.1).sleep();
        }
    } while (!transformed && ros::ok() && !lookat_as_->isPreemptRequested());

    if (lookat_as_->isPreemptRequested() )
    {
        success = false;
        message = "Preempted after lookup 'pointing_frame'";
        ROS_WARN_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setPreempted(lookat_res_);
        return;
    }

    tf::transformTFToKDL(offset_transform, offset);
    //tf::transformMsgToKDL(goal->pointing_offset, offset);

    KDL::Segment lookat_rotx_link("lookat_rotx_link", lookat_lin_joint, offset);
    chain_lookat.addSegment(lookat_rotx_link);

    KDL::Vector lookat_rotx_axis(1.0, 0.0, 0.0);
    KDL::Joint lookat_rotx_joint("lookat_rotx_joint", KDL::Vector(), lookat_rotx_axis, KDL::Joint::RotAxis);
    KDL::Segment lookat_roty_link("lookat_roty_link", lookat_rotx_joint);
    chain_lookat.addSegment(lookat_roty_link);

    KDL::Vector lookat_roty_axis(0.0, 1.0, 0.0);
    KDL::Joint lookat_roty_joint("lookat_roty_joint", KDL::Vector(), lookat_roty_axis, KDL::Joint::RotAxis);
    KDL::Segment lookat_rotz_link("lookat_rotz_link", lookat_roty_joint);
    chain_lookat.addSegment(lookat_rotz_link);

    KDL::Vector lookat_rotz_axis(0.0, 0.0, 1.0);
    KDL::Joint lookat_rotz_joint("lookat_rotz_joint", KDL::Vector(), lookat_rotz_axis, KDL::Joint::RotAxis);
    KDL::Segment lookat_focus_frame("lookat_focus_frame", lookat_rotz_joint);
    chain_lookat.addSegment(lookat_focus_frame);

    chain_full = chain_main_;
    chain_full.addChain(chain_lookat);

    /// set up solver
    fk_solver_pos_main_.reset(new KDL::ChainFkSolverPos_recursive(chain_main_));
    fk_solver_pos_.reset(new KDL::ChainFkSolverPos_recursive(chain_full));
    ik_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(chain_full));
    ik_solver_pos_.reset(new KDL::ChainIkSolverPos_NR(chain_full, *fk_solver_pos_, *ik_solver_vel_));

    /// transform target_frame to p_in
    KDL::Frame p_in;
    tf::StampedTransform transform_in;
    transformed = false;

    do
    {
        try
        {
            tf_listener_.lookupTransform(chain_base_link_, goal->target_frame, ros::Time(0), transform_in);
            transformed = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("LookatAction: %s", ex.what());
            ros::Duration(0.1).sleep();
        }
    } while (!transformed && ros::ok() && !lookat_as_->isPreemptRequested());

    if (lookat_as_->isPreemptRequested() )
    {
        success = false;
        message = "Preempted after lookup 'target_frame'";
        ROS_WARN_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setPreempted(lookat_res_);
        return;
    }

    //ToDo: check age of transformation
    ros::Time now = ros::Time::now();
    ROS_INFO_STREAM("NOW: " << now << ", STAMP: " << transform_in.stamp_ << ", DIFF: " << (now - transform_in.stamp_).toSec());

    tf::transformTFToKDL(transform_in, p_in);
    KDL::JntArray q_init(chain_full.getNrOfJoints());
    KDL::JntArray q_out(chain_full.getNrOfJoints());
    int result_ik = ik_solver_pos_->CartToJnt(q_init, p_in, q_out);

    ROS_WARN_STREAM("IK-Error: "<< ik_solver_pos_->getError() << " - " << ik_solver_pos_->strError(ik_solver_pos_->getError()));
    
    ROS_WARN_STREAM("q_init: " << q_init.data);
    ROS_WARN_STREAM("p_in: " << p_in.p.x() << ", " << p_in.p.y() << ", " << p_in.p.z());
    ROS_WARN_STREAM("q_out: " << q_out.data);

    /// solution valid?
    if (ik_solver_pos_->getError() != KDL::SolverI::E_NOERROR)
    {
        success = false;
        message = "Failed to find IK solution";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    //ToDo: check FK diff (target vs. q_out)
    KDL::Frame p_out;
    int result_fk = fk_solver_pos_->JntToCart(q_out, p_out);

    ROS_WARN_STREAM("FK-Error: "<< fk_solver_pos_->getError() << " - " << fk_solver_pos_->strError(fk_solver_pos_->getError()));

    ROS_WARN_STREAM("p_out: " << p_out.p.x() << ", " << p_out.p.y() << ", " << p_out.p.z());

    /// solution valid?
    if (fk_solver_pos_->getError() != KDL::SolverI::E_NOERROR)
    {
        success = false;
        message = "Failed to find FK solution";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    KDL::Vector v_diff = KDL::diff(p_in.p, p_out.p);
    ROS_WARN_STREAM("p_diff: " << v_diff.x() << ", " << v_diff.y() << ", " << v_diff.z());
    ROS_WARN_STREAM("NORM v_diff: " << v_diff.Norm());

    if (!KDL::Equal(p_in, p_out, 1.0) )
    {
        ROS_WARN_STREAM("P_IN !Equal P_OUT");
    }

    //ToDo: check FK based on main + offset
    KDL::Frame p_out_main;
    KDL::JntArray q_out_main(chain_main_.getNrOfJoints());
    for(unsigned int i = 0; i < chain_main_.getNrOfJoints(); i++)
    {
        q_out_main(i)=q_out(i);
    }
    int result_fk_main = fk_solver_pos_main_->JntToCart(q_out_main, p_out_main);
    KDL::Frame p_out_main_offset = p_out_main*offset;
    KDL::Frame tip2target = p_out_main_offset.Inverse()*p_in;

    ROS_WARN_STREAM("FK-Error MAIN: "<< fk_solver_pos_main_->getError() << " - " << fk_solver_pos_main_->strError(fk_solver_pos_main_->getError()));

    ROS_WARN_STREAM("q_out_main: " << q_out_main.data);
    ROS_WARN_STREAM("p_out_main: " << p_out_main.p.x() << ", " << p_out_main.p.y() << ", " << p_out_main.p.z());
    ROS_WARN_STREAM("offset: " << offset.p.x() << ", " << offset.p.y() << ", " << offset.p.z());
    ROS_WARN_STREAM("p_out_main_offset: " << p_out_main_offset.p.x() << ", " << p_out_main_offset.p.y() << ", " << p_out_main_offset.p.z());
    ROS_WARN_STREAM("tip2target: " << tip2target.p.x() << ", " << tip2target.p.y() << ", " << tip2target.p.z());

    /// solution valid?
    if (fk_solver_pos_main_->getError() != KDL::SolverI::E_NOERROR)
    {
        success = false;
        message = "Failed to find FK solution MAIN";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    KDL::Vector v_diff_main = KDL::diff(p_in.p, p_out_main_offset.p);
    ROS_WARN_STREAM("p_diff_main: " << v_diff_main.x() << ", " << v_diff_main.y() << ", " << v_diff_main.z());
    ROS_WARN_STREAM("NORM v_diff_main: " << v_diff_main.Norm());

    switch (goal->pointing_axis_type)
    {
        case cob_lookat_action::LookAtGoal::X_POSITIVE:
            tip2target.p.x(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_POSITIVE:
            tip2target.p.y(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_POSITIVE:
            tip2target.p.z(0.0);
            break;
        case cob_lookat_action::LookAtGoal::X_NEGATIVE:
            tip2target.p.x(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Y_NEGATIVE:
            tip2target.p.y(0.0);
            break;
        case cob_lookat_action::LookAtGoal::Z_NEGATIVE:
            tip2target.p.z(0.0);
            break;
        default:
            ROS_ERROR("PointingAxisType %d not defined! Using default: 'X_POSITIVE'!", goal->pointing_axis_type);
            tip2target.p.x(0.0);
            break;
    }

    if ( tip2target.p.Norm() > 0.1 )
    {
        success = false;
        message = "Tip2Target is not lookat-conform";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        ROS_WARN_STREAM("tip2target: " << tip2target.p.x() << ", " << tip2target.p.y() << ", " << tip2target.p.z());
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }

    /// execute solution as FJT
    control_msgs::FollowJointTrajectoryGoal lookat_traj;
    lookat_traj.trajectory.header.stamp = ros::Time::now();
    lookat_traj.trajectory.header.frame_id = chain_base_link_;
    lookat_traj.trajectory.joint_names = joint_names_;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    for(unsigned int i = 0; i < chain_main_.getNrOfJoints(); i++)
    {
        traj_point.positions.push_back(q_out(i));
    }
    traj_point.time_from_start = ros::Duration(3.0);
    lookat_traj.trajectory.points.push_back(traj_point);

    ROS_WARN_STREAM("LookatTraj: " << lookat_traj);

    fjt_ac_->sendGoal(lookat_traj);

    bool finished_before_timeout = fjt_ac_->waitForResult(ros::Duration(5.0));
    
    /// fjt action successful?
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = fjt_ac_->getState();
        if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            success = false;
            message = "FJT finished: " + state.toString();
            ROS_ERROR_STREAM(lookat_name_ << ": " << message);
            lookat_res_.success = success;
            lookat_res_.message = message;
            lookat_as_->setAborted(lookat_res_);
            return;
        }
    }
    else
    {
        success = false;
        message = "FJT did not finish before the time out.";
        ROS_ERROR_STREAM(lookat_name_ << ": " << message);
        lookat_res_.success = success;
        lookat_res_.message = message;
        lookat_as_->setAborted(lookat_res_);
        return;
    }


    /// lookat action successful?
    success = true;
    message = "Lookat finished successful.";
    ROS_ERROR_STREAM(lookat_name_ << ": " << message);
    lookat_res_.success = success;
    lookat_res_.message = message;
    lookat_as_->setSucceeded(lookat_res_);
    return;
}
