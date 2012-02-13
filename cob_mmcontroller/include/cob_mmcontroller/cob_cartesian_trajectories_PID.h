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
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <map>

#include <articulation_msgs/ParamMsg.h>
#include <articulation_msgs/TrackMsg.h>
#include <articulation_msgs/ModelMsg.h>
#include <cob_mmcontroller/MoveModel.h>
#include <cob_mmcontroller/MovePrismatic.h>
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
    geometry_msgs::Twist getTwist(double dt, Frame F_current);
    void getSollLinear(double dt, double &sollx, double &solly, double &sollangle);
    void getSollCircular(double dt, double &sollx, double &solly, double &sollangle);
    void getTargetPosition(double dt, KDL::Frame &F_target);                                        //new
    void getPriTarget(double dt, KDL::Frame &F_target);                                             //new
    void getRotTarget(double dt, KDL::Frame &F_target);                                             //new
    double getParamValue(std::string param_name);                                                   //new
    geometry_msgs::Twist PIDController(const double dt, const KDL::Frame &F_target, const KDL::Frame &F_Current);              //new
    void pubTrack(const int track_id, const ros::Duration pub_duration, const KDL::Frame &F_pub);                     //new
    void pubTwistMarkers(const ros::Duration pub_duration, const geometry_msgs::Twist &Twist, const KDL::Frame &F_current);
    void cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void moveCircActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal);
    void moveLinActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal);
    //bool moveCircCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);  //old
    //bool moveLinCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);   //old
    bool movePriCB(cob_mmcontroller::MovePrismatic::Request& request, cob_mmcontroller::MovePrismatic::Response& response);     //new
    bool moveRotCB(cob_mmcontroller::MoveRotational::Request& request, cob_mmcontroller::MoveRotational::Response& response);   //new
    bool moveModelCB(cob_mmcontroller::MoveModel::Request& request, cob_mmcontroller::MoveModel::Response& response);           //new
    void sendMarkers();
    bool start();
    ros::Subscriber cart_state_sub_;
    ros::Publisher cart_command_pub;
    ros::Publisher debug_cart_pub_;
    ros::Publisher map_pub_;
    ros::Publisher twist_pub_;
    ros::Publisher track_pub_;
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

    //articulation_msgs::ModelMsg target_model;   // model covering/carry track msg
    map<int, articulation_msgs::TrackMsg> track_map;     //stores track_ids and tracks for publishing
    ros::Time pub_timer;
    int pub_counter;
};

