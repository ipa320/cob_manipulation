#include "ros/ros.h"
#include <urdf/model.h>

#include "kinematics_msgs/GetPositionIK.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <cob_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

#include <articulation_msgs/ParamMsg.h>
#include <articulation_msgs/TrackMsg.h>
#include <articulation_msgs/ModelMsg.h>
#include <cob_mmcontroller/MoveModel.h>
#include <cob_mmcontroller/MovePrismatic.h>
#include <cob_mmcontroller/MovePrismaticSimple.h>
#include <cob_mmcontroller/MoveRotational.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <cob_mmcontroller/augmented_solver.h>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <cob_mmcontroller/OpenFridgeAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace KDL;
using namespace std;


class cob_cartesian_trajectories
{
public:
    cob_cartesian_trajectories();

private:
    ros::NodeHandle n;
    actionlib::SimpleActionServer<cob_mmcontroller::OpenFridgeAction> as_;
    actionlib::SimpleActionServer<cob_mmcontroller::OpenFridgeAction> as2_;
    KDL::Twist getTwist(double dt, Frame F_current);
    void getSollLinear(double dt, double &sollx, double &solly, double &sollangle);
    void getSollCircular(double dt, double &sollx, double &solly, double &sollangle);
    void getTargetPosition(double dt, KDL::Frame &F_target);                                        //new
    void getPriTarget(double dt, KDL::Frame &F_target);                                             //new
    void getRotTarget(double dt, KDL::Frame &F_target);                                             //new
    double getParamValue(std::string param_name);                                                   //new
    KDL::Twist PIDController(const double dt, const KDL::Frame &F_target, const KDL::Frame &F_Current);              //new
    //void pubTargetTrack(const KDL::Frame &F_target)                     //new
    void cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void moveCircActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal);
    void moveLinActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal);
    //bool movePriSimpleCB(cob_mmcontroller::MovePrismaticSimple::Request& request, cob_mmcontroller::MovePrismaticSimple::Response& response);     //new
    bool movePriCB(cob_mmcontroller::MovePrismatic::Request& request, cob_mmcontroller::MovePrismatic::Response& response);     //new
    bool moveRotCB(cob_mmcontroller::MoveRotational::Request& request, cob_mmcontroller::MoveRotational::Response& response);   //new
    bool moveModelCB(cob_mmcontroller::MoveModel::Request& request, cob_mmcontroller::MoveModel::Response& response);           //new
    void sendMarkers();
    bool start();
    ros::Subscriber cart_state_sub_;
    ros::Publisher cart_command_pub;
    ros::Publisher debug_cart_pub_;
    ros::Publisher map_pub_;
    ros::ServiceServer serv_prismatic_simple;      //new
    ros::ServiceServer serv_prismatic;      //new
    ros::ServiceServer serv_rotational;     //new
    ros::ServiceServer serv_model;          //new
    std::vector<geometry_msgs::Point> trajectory_points;

    bool bRun;
    bool bStarted;
    double currentDuration;
    double targetDuration;
    
    //controller configuration
    double p_gain_;
    double i_gain_;
    double d_gain_;

    ros::Time timer;
    ros::Time tstart;
    KDL::Frame F_start;
    KDL::Twist Error;
    KDL::Twist Error_sum;
    KDL::Twist Error_dot;
    KDL::Twist Error_last;
    
    string mode;                       // prismatic, rotational, trajectory, model
    std::vector<articulation_msgs::ParamMsg> params;    // articulation parameters
    std::vector<geometry_msgs::Pose> track;     // trajectory
        
    geometry_msgs::PoseStamped current_hinge;

};


cob_cartesian_trajectories::cob_cartesian_trajectories() : as_(n, "moveCirc", boost::bind(&cob_cartesian_trajectories::moveCircActionCB, this, _1), false), as2_(n, "moveLin", boost::bind(&cob_cartesian_trajectories::moveLinActionCB, this, _1), false)
{
    ros::NodeHandle node;
    node.param("arm_controller/arm_mmcontroller_node/p_gain", p_gain_, 1.0);
    node.param("arm_controller/arm_mmcontroller_node/i_gain", i_gain_, 1.0);
    node.param("arm_controller/arm_mmcontroller_node/d_gain", d_gain_, 1.0);
    ROS_INFO("Starting PID controller with P: %e, I: %e, D: %e", p_gain_, i_gain_, d_gain_);
    
    cart_state_sub_ = n.subscribe("/arm_controller/cart_state", 1, &cob_cartesian_trajectories::cartStateCallback, this);
    cart_command_pub = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
    debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/mm/debug",1);
    //serv_prismatic_simple = n.advertiseService("/mm/move_pri_simple", &cob_cartesian_trajectories::movePriSimpleCB, this);      // new service for simply input
    serv_prismatic = n.advertiseService("/mm/move_pri", &cob_cartesian_trajectories::movePriCB, this);      // new service
    serv_rotational = n.advertiseService("/mm/move_rot", &cob_cartesian_trajectories::moveRotCB, this);     // new service
    serv_model = n.advertiseService("/mm/move_model", &cob_cartesian_trajectories::moveModelCB, this);       // new service to work with models
    map_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    //track_pub = n.advertise<articulation_msgs::TrackMsg>("/track", 1);                // publish generated trajectory for debugging
    bRun = false;
    as_.start();
    as2_.start();
    targetDuration = 0;
    currentDuration = 0;
    mode = "prismatic";     // prismatic, rotational, trajectory, model
}

void cob_cartesian_trajectories::sendMarkers()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_values";
    marker.id = 10;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.04;
    marker.scale.y = 0.2;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(10.0);

    for(unsigned int i=0; i<trajectory_points.size(); i++)
    {
        //ROS_INFO("line %f %f %f %f\n", iX1, iY1, iX2, iY2);
        marker.points.push_back(trajectory_points[i]);
    }
    map_pub_.publish(marker);
}

void cob_cartesian_trajectories::moveCircActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal)
{
    mode = "circular";
    current_hinge = goal->hinge;
    if(start())
    {
        while(bRun)
        {
            //wait until finished
            sleep(1);
        }
        as_.setSucceeded();
    }
    return;

}
void cob_cartesian_trajectories::moveLinActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal)
{
    mode = "linear";
    if(start())
    {
        while(bRun)
        {
            //wait until finished
            sleep(1);
        }
        as2_.setSucceeded();
    }
    return;

}

/*
bool cob_cartesian_trajectories::movePriSimpleCB(cob_mmcontroller::MovePrismaticSimple::Request& request, cob_mmcontroller::MovePrismaticSimple::Response& response)    //TODO // simple prismatic callback
{
    mode = "prismatic";
    targetDuration = request.target_duration;
    std::cout << "check1 " << "\n";
    //params[0].name = "length";
    std::cout << "check2 " << params[0].name << "\n";
    params[0].value = request.length;
    params[1].name = "prismatic_dir.x";
    params[1].value = request.pris_dir_x;
    params[2].name = "prismatic_dir.y";
    params[2].value = request.pris_dir_y;
    params[3].name = "prismatic_dir.z";
    params[3].value = request.pris_dir_z;
    std::cout << targetDuration << "\n";
    return start();
}*/

bool cob_cartesian_trajectories::movePriCB(cob_mmcontroller::MovePrismatic::Request& request, cob_mmcontroller::MovePrismatic::Response& response)    //TODO // prismatic callback
{
    mode = "prismatic";
    targetDuration = request.target_duration.data.toSec();
    params = request.params;
    std::cout << targetDuration << "\n";
    return start();
}

bool cob_cartesian_trajectories::moveRotCB(cob_mmcontroller::MoveRotational::Request& request, cob_mmcontroller::MoveRotational::Response& response)    //TODO // rotational callback
{
    mode = "rotational";
    targetDuration = request.target_duration.data.toSec();
    params = request.params;
    std::cout << targetDuration << "\n";
    return start();
}

/*bool cob_cartesian_trajectories::moveTrajCB(cob_mmcontroller::MoveTrajectory::Request& request, cob_mmcontroller::MoveTrajectory::Response& response)   // TODO // trajectory callback
{
    mode = "trajectory";
    targetDuration = request.target_duration.data.toSec();
    track = request.pose
    return start();
}*/

bool cob_cartesian_trajectories::moveModelCB(cob_mmcontroller::MoveModel::Request& request, cob_mmcontroller::MoveModel::Response& response)  //TODO // model callback
{
    mode = "model";
    targetDuration = request.target_duration.data.toSec();
    //model_params = request.model.params            // model parameters
    track = request.model.track.pose_projected;         // trajectory
    return start();
}

bool cob_cartesian_trajectories::start() //TODO request->model.params // start 
{
    if(bRun)
    {
        ROS_ERROR("Already running trajectory");
        return false;
    }
    else
    {
        bRun = true;
        bStarted = false;
        timer = ros::Time::now();
        tstart = ros::Time::now();
        currentDuration = 0;
        trajectory_points.clear();
        return true;
    }    
}

//Pose is global pose with odometry
void cob_cartesian_trajectories::cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(bRun)
    {
        ros::Duration dt = ros::Time::now() - timer;
        cout << "dt: " << dt << "\n";
        timer = ros::Time::now();
        if((targetDuration-currentDuration) <= 0)
        {
            geometry_msgs::Twist twist;
            cart_command_pub.publish(twist);
            ROS_INFO("finished trajectory in %f", ros::Time::now().toSec() - tstart.toSec());
            bRun = false;
            bStarted = false;
            sendMarkers();
            Error = Twist::Zero();
            Error_sum = Twist::Zero();
            Error_dot = Twist::Zero();
            return;
        }        
        KDL::Frame current;
        KDL::Frame myhinge;
        tf::PoseMsgToKDL(msg->pose, current);
        tf::PoseMsgToKDL(current_hinge.pose, myhinge);
        KDL::Vector unitz = myhinge.M.UnitZ();
        std::cout << "Radius because of Hinge: " << (myhinge.p - current.p) << "UnitZ of hinge: " << unitz.z() << "\n";
        geometry_msgs::Twist twist;
        KDL::Twist ktwist = getTwist(currentDuration, current);
        twist.linear.x =  ktwist.vel.x();
        twist.linear.y =  ktwist.vel.y();
        twist.linear.z =  ktwist.vel.z();

        twist.angular.x =  ktwist.rot.x();
        twist.angular.y =  ktwist.rot.y();
        twist.angular.z =  ktwist.rot.z();


        //twist.linear.z = -0.02;
        cart_command_pub.publish(twist);
        currentDuration+=dt.toSec();

        geometry_msgs::Point p;
        p.x = msg->pose.position.x;
        p.y = msg->pose.position.y;
        p.z = msg->pose.position.z;
        trajectory_points.push_back(p);
    }
    else
    {
        //publish zero    
        geometry_msgs::Twist twist;
        cart_command_pub.publish(twist);
    }
}

/*
void cob_cartesian_trajectories::getSollLinear(double dt, double &sollx, double &solly, double &sollangle)
{
    double look_ahead = 0.1;
    sollx = ((dt+look_ahead)/targetDuration) * -0.1;
    solly = 0.0; //((dt+look_ahead)/targetDuration) * 0.6;
    sollangle = 0.0;
}
*/

/*
void cob_cartesian_trajectories::getSollCircular(double dt, double &sollx, double &solly, double &sollangle)
{
    double look_ahead = 0.0;
    double max_ang = 3.14*0.505;
    double radius = 0.31; //TODO: Radius einstellen

    sollx = sin(max_ang*((dt+look_ahead)/targetDuration)) * radius ;
    solly = radius-(cos(max_ang*((dt+look_ahead)/targetDuration)) * radius);
    solly *= -1;
    std::cout << "Soll: " << sollx << ", " << solly << "\n";
    sollangle = -max_ang*((dt+look_ahead)/targetDuration);    
}
*/

void cob_cartesian_trajectories::getTargetPosition(double dt, KDL::Frame &F_target)    //TODO //
{
    if (mode == "prismatic")
        getPriTarget(dt, F_target);
    else if (mode == "rotational")
        getRotTarget(dt, F_target);
    /*else if (mode == "trajectory");
        getTrajTarget(dt, F_target);
    else if (mode == "model");
        getModelTarget(dt, F_target);*/
    else
        ROS_ERROR("Invalid mode");
}



void cob_cartesian_trajectories::getPriTarget(double dt, KDL::Frame &F_target)
{
    double length;
    double partial_length;
    double pris_dir_x;
    double pris_dir_y;
    double pris_dir_z;
    
    length = getParamValue("length");
    pris_dir_x = getParamValue("prismatic_dir.x");
    pris_dir_y = getParamValue("prismatic_dir.y");
    pris_dir_z = getParamValue("prismatic_dir.z");
    
    partial_length = length * (dt/targetDuration);
    
    F_target.p.x(F_start.p.x() + partial_length*pris_dir_x);
    F_target.p.y(F_start.p.y() + partial_length*pris_dir_y);
    F_target.p.z(F_start.p.z());
    //F_target.p.z(F_start.p.z() + partial_length*pris_dir_z);
    F_target.M = F_start.M; 
    
    std::cout << "F_X: " << F_target.p.x() << " F_Y: " << F_target.p.y() << " F_Z: " << F_target.p.z() << "\n";
}

//rotational trajectory from rot_axis, rot_radius and angle
void cob_cartesian_trajectories::getRotTarget(double dt, KDL::Frame &F_target)
{
    double angle;
    double partial_angle;
    double rot_center_x;
    double rot_center_y;
    double rot_center_z;
    double radius;
    KDL::Rotation temp_rot;
    
    double start_roll, start_pitch, start_yaw = 0.0;
    double target_roll, target_pitch, target_yaw = 0.0;

    angle = getParamValue("angle");
    rot_center_x = getParamValue("rot_center.x");
    rot_center_y = getParamValue("rot_center.y");
    rot_center_z = getParamValue("rot_center.z");
    
    partial_angle = angle * (dt/targetDuration);
    
    cout << "Partial Angle: " << partial_angle << "\n";
    
    radius = sqrt(rot_center_x*rot_center_x + rot_center_y*rot_center_y); // + rot_center_z*rot_center_z);
    
    
    F_target.p.x(F_start.p.x() + radius*sin(partial_angle));
    F_target.p.y(F_start.p.y() + radius*(1-cos(partial_angle)));
    F_target.p.z(F_start.p.z());
    //F_target.p.z(F_start.p.z + radius*sin(partial_angle));
    
    std::cout << "F_X: " << F_target.p.x() << " F_Y: " << F_target.p.y() << " F_Z: " << F_target.p.z() << "\n";
    

    temp_rot = Rotation::Identity();    // TODO Rotation::Rot(vector,angle);
    temp_rot.DoRotZ(partial_angle);        // rotation axis of articulation
    temp_rot = temp_rot*F_start.M;
    
    cout << "temp_rot: " << "\n" << temp_rot << "\n";
    
    F_target.M = temp_rot;
    
    F_start.M.GetRPY(start_roll, start_pitch, start_yaw);
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);

    cout << "Start R-P-Y: " << start_roll << " " << start_pitch << " " << start_yaw << "\n";
    cout << "Target R-P-Y: " << target_roll << " " << target_pitch << " " << target_yaw << "\n";
}


/* //rotational trajectory from rot_axis, rot_radius and angle
void cob_cartesian_trajectories::getRotTarget(double dt, KDL::Frame &F_target)
{
    double angle;
    double partial_angle;
    double radius;
    double rot_axis_x;
    double rot_axis_y;
    double rot_axis_z;
    double rot_axis_w;
    KDL::Rotation rot;
    KDL::Vector rot_vec;
    KDL::Rotation temp_rot;
    
    double alpha, d1, d2, d3;
    double norm;
    
    angle = getParamValue("angle");
    radius = getParamValue("rot_radius");
    rot_axis_x = getParamValue("rot_axis.x");
    rot_axis_y = getParamValue("rot_axis.y");
    rot_axis_z = getParamValue("rot_axis.z");
    rot_axis_w = getParamValue("rot_axis.w");
    //rot.Quaternion(rot_axis_x, rot_axis_y, rot_axis_z, rot_axis_w);
    //rot_vec = rot.GetRot();
    
    //Calculation of vector to rotate around 
    alpha = 2 * asin(rot_axis_w);
    d1 = rot_axis_x / sin(alpha/2);
    d2 = rot_axis_y / sin(alpha/2);
    d3 = rot_axis_z / sin(alpha/2);
    cout << "d1: " << d1 << " d2: " << d2 << " d3: " << d3 << "\n";
    rot_vec.x(d1);
    rot_vec.y(d2);
    rot_vec.z(d3);
    norm = rot_vec.Normalize(0.1);
    
    cout << "Norm: " << norm << "\n";
    
    //cout << "Rotation: " << rot << "\n";
    cout << "Rotation vector: " << rot_vec << "\n";
    
    partial_angle = angle * (dt/targetDuration);
    
    cout << "Partial Angle: " << partial_angle << "\n";
    
    
    
    
    temp_rot = Rotation::Rot(rot_vec, partial_angle);
    temp_rot = temp_rot*F_start.M;
    
    cout << "temp_rot: " << "\n" << temp_rot << "\n";
    
    F_target.M = temp_rot;
}*/


double cob_cartesian_trajectories::getParamValue(std::string param_name)
{
    for (unsigned int i = 0; i < params.size(); i++)
    {
        if (strncmp(params[i].name.c_str(), param_name.c_str(), param_name.size()) == 0)
        {
            std::cout << " -- " << params[i].name << " -- " << params[i].value << "\n";
            return params[i].value;
        }
    }
    ROS_ERROR("Missing parameter");
    std::cout << "No value found for parameter " << param_name << "\n";
    return 0.0; //?????
}


KDL::Twist cob_cartesian_trajectories::getTwist(double dt, Frame F_current)
{
    KDL::Frame F_target;    // = F_start; ???
    KDL::Frame F_diff; // = F_start;
    KDL::Twist lin;
    double start_roll, start_pitch, start_yaw = 0.0;
    double current_roll, current_pitch, current_yaw = 0.0;
    double target_roll, target_pitch, target_yaw = 0.0;

    if(!bStarted)
    {
        F_start = F_current;
        bStarted = true;
    }
    
    getTargetPosition(dt, F_target);
   
    F_start.M.GetRPY(start_roll, start_pitch, start_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);

    //std::cout << "AngleDiff: " << (current_yaw-start_yaw) << " vs " << (soll_angle) << " error: " << (soll_angle-current_yaw-start_yaw) << "\n";

    
    std::cout << "Target (x,y):  " << F_target.p.x() << ", " << F_target.p.y() << "\n";
    std::cout << "Error (x,y):   " << F_target.p.x()-F_current.p.x() << ", " << F_target.p.y()-F_current.p.y() << "\n";
    std::cout << "Start (x,y):   " << F_start.p.x() << ", " << F_start.p.y() << "\n";
    std::cout << "Current (x,y): " << F_current.p.x() << ", " << F_current.p.y() << "\n";

    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);
    
    //pubTargetTrack(F_target);
    
    lin = PIDController(dt, F_target, F_current);
    
    //DEBUG
    F_diff.p.x(F_target.p.x()-F_current.p.x());
    F_diff.p.y(F_target.p.y()-F_current.p.y());
    F_diff.p.z(F_target.p.z()-F_current.p.z());
    geometry_msgs::PoseArray poses;
    poses.poses.resize(3);
    tf::PoseKDLToMsg(F_current, poses.poses[0]);
    tf::PoseKDLToMsg(F_target, poses.poses[1]);
    tf::PoseKDLToMsg(F_diff, poses.poses[2]);
    debug_cart_pub_.publish(poses);
    //std::cout << "Twist x: " << 0.1 * F_diff.p.x() << " y: " << 0.1 * F_diff.p.y() << "\n";
    //

    return lin;
}

KDL::Twist cob_cartesian_trajectories::PIDController(const double dt, const KDL::Frame &F_target, const KDL::Frame &F_current)
{
    //double p = 1.0;           //gain for proportional part
    //double i = 1.0;           //gain for integrating part
    //double d = 1.0;           //gain for differential part
    KDL::Twist twist;
    double current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;
    double target_roll = 0.0, target_pitch = 0.0, target_yaw = 0.0;
    
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);
        
    Error.vel.x(F_target.p.x() - F_current.p.x());
    Error.vel.y(F_target.p.y() - F_current.p.y());
    Error.vel.z(F_target.p.z() - F_current.p.z());
    Error.rot.x(target_roll - current_roll);
    Error.rot.y(target_pitch - current_pitch);
    Error.rot.z(target_yaw - current_yaw);
    
    cout << "Error twist: " << "\n" << Error << "\n";
    
    Error_sum.vel.x(Error_sum.vel.x() + (F_target.p.x() - F_current.p.x()) * dt);
    Error_sum.vel.y(Error_sum.vel.y() + (F_target.p.y() - F_current.p.y()) * dt);
    Error_sum.vel.z(Error_sum.vel.z() + (F_target.p.z() - F_current.p.z()) * dt);
    Error_sum.rot.x(Error_sum.rot.x() + (target_roll - current_roll) * dt);
    Error_sum.rot.y(Error_sum.rot.y() + (target_pitch - current_pitch) * dt);
    Error_sum.rot.z(Error_sum.rot.z() + (target_yaw - current_yaw) * dt); 
    
    cout << "Error_sum twist: " << "\n" << Error_sum << "\n";
    
    Error_dot.vel.x((Error_last.vel.x() - F_target.p.x() - F_current.p.x()) / dt);
    Error_dot.vel.y((Error_last.vel.y() - F_target.p.y() - F_current.p.y()) / dt);
    Error_dot.vel.z((Error_last.vel.z() - F_target.p.z() - F_current.p.z()) / dt);
    Error_dot.rot.x((Error_last.rot.x() - target_roll - current_roll) / dt);
    Error_dot.rot.y((Error_last.rot.y() - target_pitch - current_pitch) / dt);
    Error_dot.rot.z((Error_last.rot.z() - target_yaw - current_yaw) / dt);
    
    cout << "Error_dot twist: " << "\n" << Error_dot << "\n";
    
    // create twist
    twist.vel.x(p_gain_*Error.vel.x() + i_gain_*Error_sum.vel.x());
    twist.vel.y(p_gain_*Error.vel.y() + i_gain_*Error_sum.vel.y());
    twist.vel.z(0.0);//p_gain_*Error.vel.z() + i_gain_*Error_sum.vel.z());//p_gain_*Error.vel.z());
    twist.rot.x(0.0);//p_gain_*Error.rot.x());//(p_gain_*Error.rot.x() + i_gain_*Error_sum.rot.x());
    twist.rot.y(0.0);//p_gain_*Error.rot.y());//(p_gain_*Error.rot.y() + i_gain_*Error_sum.rot.y());
    twist.rot.z(p_gain_*Error.rot.z() + i_gain_*Error_sum.rot.z());
    
    return twist;
}
/* publish generated trajectory
void cob_cartesian:trajectories::pubTargetTrack(const KDL::Frame &F_target)
{
    articulation::TrackMsg target_track;
    
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_cartesian_trajectories");
    cob_cartesian_trajectories controller ;
    ros::spin();

    return 0;
}
