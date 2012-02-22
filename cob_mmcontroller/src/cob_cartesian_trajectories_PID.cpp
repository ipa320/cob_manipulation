#include <cob_mmcontroller/cob_cartesian_trajectories_PID.h>

cob_cartesian_trajectories::cob_cartesian_trajectories() : as_(n, "moveCirc", boost::bind(&cob_cartesian_trajectories::moveCircActionCB, this, _1), false), as2_(n, "moveLin", boost::bind(&cob_cartesian_trajectories::moveLinActionCB, this, _1), false), as_model_(n, "moveModel", boost::bind(&cob_cartesian_trajectories::moveModelActionCB, this, _1), false)
{
    ros::NodeHandle node;
    node.param("cob_cartesian_trajectories_PID/p_gain", p_gain_, 1.0);
    node.param("cob_cartesian_trajectories_PID/i_gain", i_gain_, 0.002);
    node.param("cob_cartesian_trajectories_PID/d_gain", d_gain_, 0.0);
    ROS_INFO("Starting PID controller with P: %e, I: %e, D: %e", p_gain_, i_gain_, d_gain_);
    
    cart_state_sub_ = n.subscribe("/arm_controller/cart_state", 1, &cob_cartesian_trajectories::cartStateCallback, this);
    joint_state_sub_ = n.subscribe("/joint_states", 1, &cob_cartesian_trajectories::jointStateCallback, this);
    cart_command_pub = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
    debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/mm/debug",1);
    serv_prismatic = n.advertiseService("/mm/move_pri", &cob_cartesian_trajectories::movePriCB, this);      // new service
    serv_rotational = n.advertiseService("/mm/move_rot", &cob_cartesian_trajectories::moveRotCB, this);     // new service
    serv_model = n.advertiseService("/mm/move_model", &cob_cartesian_trajectories::moveModelCB, this);       // new service to work with models
    map_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    twist_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);   // publish twist to be visualized inj rviz
    track_pub_ = n.advertise<articulation_msgs::TrackMsg>("/track", 1);                 // publish generated trajectory for debugging
    bRun = false;
    as_.start();
    as2_.start();
    as_model_.start();
    targetDuration = 0;
    currentDuration = 0;
    mode = "prismatic";     // prismatic, rotational, trajectory, model
   
    pub_timer = ros::Time::now();
    pub_counter = 0;
    double const PI = 4.0*std::atan(1.0);

    getJointLimits(UpperLimits, LowerLimits);   // get arm joint limits from topic /robot_description
}


// to avoid reaching the arm joint limits
// get arm joint limits from topic /robot_description
void cob_cartesian_trajectories::getJointLimits(std::vector<double> &UpperLimits, std::vector<double> &LowerLimits)
{
    ros::NodeHandle param_node;
    const unsigned int DOF = 7;
    std::vector<std::string> JointNames;
    std::string param_name = "robot_description";
    std::string full_param_name;
    std::string xml_string;

    for (unsigned int i = 1; i <= DOF; i++)
    {
        stringstream ss;
        ss << i;
        JointNames.push_back("arm_" + ss.str() + "_joint");
    }

    param_node.searchParam(param_name, full_param_name);
    if (param_node.hasParam(full_param_name))
    {
        param_node.getParam(full_param_name.c_str(), xml_string);
        //std::cout << "Parameter name: " << full_param_name << "\n";
    }

    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
        param_node.shutdown();
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
        param_node.shutdown();
    }
    ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

    /// Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        param_node.shutdown();
    }
    ROS_DEBUG("Successfully parsed urdf file");

    /// Get lower limits out of urdf model
    for (unsigned int i = 0; i < DOF; i++)
    {
        LowerLimits.push_back(model.getJoint(JointNames[i].c_str())->limits->lower);
    }

    // Get upper limits out of urdf model
    for (unsigned int i = 0; i < DOF; i++)
    {
        UpperLimits.push_back(model.getJoint(JointNames[i].c_str())->limits->upper);
    }
    param_node.shutdown();
}

// get arm joint states 
std::vector<double> cob_cartesian_trajectories::parseJointStates(std::vector<std::string> names, std::vector<double> positions)
{
    std::vector<double> q_temp(7);
    bool parsed = false;
    unsigned int count = 0;
    for(unsigned int i = 0; i < names.size(); i++)
    {
            if(strncmp(names[i].c_str(), "arm_", 4) == 0)
            {
                q_temp[count] = positions[i];
                count++;
                parsed = true;
            }
    }

    if(!parsed)
        return q_last;

    q_last = q_temp;
    return q_temp;

}

// check if an arm joint limit is reached
void cob_cartesian_trajectories::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (bRun)
    {
        std::vector<std::string> names = msg->name;
        std::vector<double> positions = msg->position;
        jointStates = parseJointStates(names,positions);

        // stopping to run trajectory 
        for (unsigned int i = 0; i < jointStates.size(); i++)
        {
            if (jointStates[i] <= (LowerLimits[i] + 0.04))
            {
                ROS_INFO("Stopping trajectory because arm joint %d reached almost lower joint limit!", i+1);
                success = false;
                stopTrajectory();
                //std::cout << "Stopping trajectory because arm joint " << i+1 << " reached almost lower joint limit!" << "\n";
            }
            else if (jointStates[i] >= (UpperLimits[i] - 0.04))
            {
                ROS_INFO("Stopping trajectory because arm joint %d reached almost upper joint limit!", i+1);
                success = false;
                stopTrajectory();
                //std::cout << "Stopping trajectory because arm joint " << i+1 << " reached almost upper joint limit!" << "\n";
            }
            //std::cout << "arm_" << i+1 << "_joint is " << jointStates[i] << " >> " << -1.0 *(fabs(LowerLimits[i])+jointStates[i]) << " to lower and " << UpperLimits[i]-jointStates[i] << " to upper limit" << "\n";
        }
    }

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
// action for model 
void cob_cartesian_trajectories::moveModelActionCB(const cob_mmcontroller::ArticulationModelGoalConstPtr& goal)
{
    mode = goal->model.name;
    targetDuration = goal->target_duration.data.toSec();
    params = goal->model.params;
    if(start())
    {
        while(bRun)
        {
            //wait until finished
        
            //publish feedback
            feedback_.time_left = targetDuration - currentDuration;
            as_model_.publishFeedback(feedback_);

            sleep(1);
        }
        if (success)
        {
            result_.finished = 0;
            as_model_.setSucceeded(result_);
        }
        else
        {
            result_.finished = 1;
            as_model_.setAborted(result_);
        }
    }
    return;
}

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
        success = false;
        Error_last = Twist::Zero();
        return true;
    }    
}

//Pose is global pose with odometry
void cob_cartesian_trajectories::cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(bRun)
    {
        ros::Duration dt = ros::Time::now() - timer;
        cout << "\n" << "===================================" << "\n\n" << "dt: " << dt << "\n";
        timer = ros::Time::now();
        if((targetDuration-currentDuration) <= 0)
        {
            geometry_msgs::Twist twist;
            cart_command_pub.publish(twist);
            ROS_INFO("finished trajectory in %f", ros::Time::now().toSec() - tstart.toSec());
            success = true;
            stopTrajectory();
            return;
        }
        KDL::Frame current;
        KDL::Frame myhinge;
        tf::PoseMsgToKDL(msg->pose, current);
        tf::PoseMsgToKDL(current_hinge.pose, myhinge);
        KDL::Vector unitz = myhinge.M.UnitZ();
        std::cout << "Radius because of Hinge: " << (myhinge.p - current.p) << "UnitZ of hinge: " << unitz.z() << "\n";

        // get twist to be published
        geometry_msgs::Twist twist;
        twist = getTwist(currentDuration, current); 

        // publish twist
        cart_command_pub.publish(twist);

        // add needed time
        currentDuration+=dt.toSec();

        // add position to be visualized in rviz
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

void cob_cartesian_trajectories::stopTrajectory()
{
    geometry_msgs::Twist twist;
    cart_command_pub.publish(twist);
    bRun = false;
    bStarted = false;
    sendMarkers();
    Error = Twist::Zero();
    Error_sum = Twist::Zero();
    Error_dot = Twist::Zero();
}

geometry_msgs::Twist cob_cartesian_trajectories::getTwist(double dt, Frame F_current)
{
    KDL::Frame F_target;
    KDL::Frame F_diff;
    geometry_msgs::Twist ControllTwist;
    double start_roll, start_pitch, start_yaw = 0.0;
    double current_roll, current_pitch, current_yaw = 0.0;
    double target_roll, target_pitch, target_yaw = 0.0;

    if(!bStarted)
    {
        F_start = F_current;
        F_start.M.GetRPY(last_rpy_angles["target_roll"], last_rpy_angles["target_pitch"], last_rpy_angles["target_yaw"]);
        F_start.M.GetRPY(last_rpy_angles["current_roll"], last_rpy_angles["current_pitch"], last_rpy_angles["current_yaw"]);
        bStarted = true;
    }
    
    getTargetPosition(dt, F_target);
   
    F_start.M.GetRPY(start_roll, start_pitch, start_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);

    //std::cout << "AngleDiff: " << (current_yaw-start_yaw) << " vs " << (soll_angle) << " error: " << (soll_angle-current_yaw-start_yaw) << "\n";

    
    std::cout << "Target (x,y):  " << F_target.p.x() << ", " << F_target.p.y() << "\n";
    std::cout << "Error (x,y):   " << F_target.p.x()-F_current.p.x() << ", " << F_target.p.y()-F_current.p.y() << "\n";
    std::cout << "Start (x,y):   " << F_start.p.x() << ", " << F_start.p.y() << "\n";
    std::cout << "Current (x,y): " << F_current.p.x() << ", " << F_current.p.y() << "\n";

    // calling PIDController to calculate the actuating variable and get back twist
    ControllTwist = PIDController(dt, F_target, F_current);
    
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

    return ControllTwist;
}

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

// linear trajectory 
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
    
    double start_roll, start_pitch, start_yaw = 0.0;        //debug
    double target_roll, target_pitch, target_yaw = 0.0;     //debug

    // parameter from parameter server
    angle = getParamValue("angle");
    rot_center_x = getParamValue("rot_center.x");
    rot_center_y = getParamValue("rot_center.y");
    rot_center_z = getParamValue("rot_center.z");
    
    // calculating target angle with respect to the time
    partial_angle = angle * (dt/targetDuration);
    
    // calculating radius
    radius = sqrt(rot_center_x*rot_center_x + rot_center_y*rot_center_y); // + rot_center_z*rot_center_z);
    
    // calulating target position with respect to the time and the start position
    F_target.p.x(F_start.p.x() + radius*sin(partial_angle));
    F_target.p.y(F_start.p.y() + radius*(1-cos(partial_angle)));
    F_target.p.z(F_start.p.z()); // keep initial height
    //F_target.p.z(F_start.p.z + radius*sin(partial_angle));
    
    
    // calculating the target rotation (at the moment only around the z-axis)
    temp_rot = Rotation::Identity();    // TODO Rotation::Rot(vector,angle);
    std::cout << "temp_rot" << "\n" << temp_rot << "\n";
    temp_rot.DoRotZ(partial_angle);     // rotation axis of the articulation
    std::cout << "temp_rot" << "\n" << temp_rot << "\n";
    temp_rot = temp_rot*F_start.M;      // rotation is composed of two rotations. 
                                        // first about the z-axis of the base_link frame and 
    std::cout << "temp_rot" << "\n" << temp_rot << "\n";
                                        // than the rotation from base_link to start frame of the gripper
    
    F_target.M = temp_rot;

    // ------------debugging output-----------------------
    cout << "radius: " << radius << "\n";
    cout << "Partial Angle: " << partial_angle << "\n";
    cout << "sin: " << sin(partial_angle) << "\n";
    cout << "cos: " << 1-cos(partial_angle) << "\n";

    std::cout << "F_X: " << F_target.p.x() << " F_Y: " << F_target.p.y() << " F_Z: " << F_target.p.z() << "\n";
    
    cout << "F_start.M: " << "\n" << F_start.M << "\n";
    cout << "F_target.M: " << "\n" << F_target.M << "\n";
    
    F_start.M.GetRPY(start_roll, start_pitch, start_yaw);
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);
    cout << "Target R-P-Y: " << target_roll << " " << target_pitch << " " << target_yaw << "\n";

    cout << "Start R-P-Y: " << start_roll << " " << start_pitch << " " << start_yaw << "\n";
    cout << "Target R-P-Y: " << target_roll << " " << target_pitch << " " << target_yaw << "\n";
    cout << "Current Duration: " << dt << "\n";
}

// to avoid the jump from positive to negative and the other way around 
// when crossing pi or -pi for roll and yaw angles and pi/2 and -pi/2 for pitch
double cob_cartesian_trajectories::unwrapRPY(std::string axis, double angle)
{
    double unwrapped_angle = 0.0;
    double fractpart, intpart;
    double discont = PI;    //point of discontinuity

    // FORMULA: phi(n)_adjusted = MODULO( phi(n) - phi(n-1) + PI, 2*PI) + phi(n-1) - PI
    
    // the pitch angle is only defined from -PI/2 to PI/2
    if (strncmp(&axis[strlen(axis.c_str())-6], "_pitch", 6) == 0)
        discont = PI/2;
    
    fractpart = modf(((angle - last_rpy_angles[axis] + discont) / (2*discont)), &intpart);    // modulo for double and getting only the fractal part of the division
    
    if (fractpart >= 0.0) // to get always the positive modulo
        unwrapped_angle = (fractpart)*(2*discont) - discont + last_rpy_angles[axis];
    else
        unwrapped_angle = (1+fractpart)*(2*discont) - discont + last_rpy_angles[axis];

    std::cout << "modulo: " << intpart << " + " << (fractpart) << "\n";
    
    std::cout << axis << " angle: " << angle << "\n";
    std::cout << "unwrapped " << axis << " angle: " << unwrapped_angle << "\n";
    last_rpy_angles[axis] = unwrapped_angle;
    return unwrapped_angle;
}

/*//rotational 6D-trajectory from rot_axis, rot_radius and angle
void cob_cartesian_trajectories::getRotTarget(double dt, KDL::Frame &F_target)
{
    double angle;
    double partial_angle;
    double rot_radius;
    double rot_center_x;
    double rot_center_y;
    double rot_center_z;
    double rot_axis_x;
    double rot_axis_y;
    double rot_axis_z;
    double rot_axis_w;
    KDL::Frame F_articulation;
    KDL::Frame F_track;
    //KDL::Rotation rot;
    //KDL::Vector rot_vec;
    //KDL::Rotation temp_rot;
    
    //double alpha, d1, d2, d3;
    //double norm;
    
    angle = getParamValue("angle");
    rot_radius = getParamValue("rot_radius");
    rot_center_x = getParamValue("rot_center.x");
    rot_center_y = getParamValue("rot_center.y");
    rot_center_z = getParamValue("rot_center.z");
    rot_axis_x = getParamValue("rot_axis.x");
    rot_axis_y = getParamValue("rot_axis.y");
    rot_axis_z = getParamValue("rot_axis.z");
    rot_axis_w = getParamValue("rot_axis.w");

    // calculating partial_angle
    partial_angle = angle * (dt/targetDuration);

    // creating articulation frame TODO: check orientation
    F_articulation.p.x(rot_center_x);
    F_articulation.p.y(rot_center_y);
    F_articulation.p.z(rot_center_z);
    F_articulation.M.Quaternion(rot_axis_x, rot_axis_y, rot_axis_z, rot_axis_w);    //TODO: check if z-axis = articulation axis

    // creating trajectory frame w.r.t. the articulation frame
    // orientation is like sdh_tip_link frame
    F_track.p.x(rot_radius*cos(partial_angle));
    F_track.p.y(rot_radius*sin(partial_angle));

    F_track.M.RPY(-PI/2.0, -(PI/2.0+partial_angle), 0.0);  //TODO: check orientation //depending also on handle orientation

    // transformation of trajectory frame into base_link frame ??? map frame
    F_target = F_track*F_articulation;       // FT_a*FA_bl




    --------------old approach------------------
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
    return 0.0;
}


geometry_msgs::Twist cob_cartesian_trajectories::PIDController(const double dt, const KDL::Frame &F_target, const KDL::Frame &F_current)
{
    geometry_msgs::Twist twist;
    double current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;
    double target_roll = 0.0, target_pitch = 0.0, target_yaw = 0.0;
    
    F_target.M.GetRPY(target_roll, target_pitch, target_yaw);
    target_yaw = unwrapRPY("target_yaw", target_yaw);
    F_current.M.GetRPY(current_roll, current_pitch, current_yaw);
    current_yaw = unwrapRPY("current_yaw", current_yaw);
        
    Error.vel.x(F_target.p.x() - F_current.p.x());
    Error.vel.y(F_target.p.y() - F_current.p.y());
    Error.vel.z(F_target.p.z() - F_current.p.z());
    Error.rot.x(target_roll - current_roll);
    Error.rot.y(target_pitch - current_pitch);
    Error.rot.z(target_yaw - current_yaw);
    
    cout << "Error twist: " << "\n" << Error << "\n";
    
    Error_sum.vel.x(Error_sum.vel.x() + Error.vel.x() * dt);
    Error_sum.vel.y(Error_sum.vel.y() + Error.vel.y() * dt);
    Error_sum.vel.z(Error_sum.vel.z() + Error.vel.z() * dt);
    Error_sum.rot.x(Error_sum.rot.x() + Error.rot.x() * dt);
    Error_sum.rot.y(Error_sum.rot.y() + Error.rot.y() * dt);
    Error_sum.rot.z(Error_sum.rot.z() + Error.rot.z() * dt); 
    
    cout << "Error_sum twist: " << "\n" << Error_sum << "\n";
    
    Error_dot.vel.x((Error.vel.x() - Error_last.vel.x()) / dt);
    Error_dot.vel.y((Error.vel.y() - Error_last.vel.y()) / dt);
    Error_dot.vel.z((Error.vel.z() - Error_last.vel.z()) / dt);
    Error_dot.rot.x((Error.rot.x() - Error_last.rot.x()) / dt);
    Error_dot.rot.y((Error.rot.y() - Error_last.rot.y()) / dt);
    Error_dot.rot.z((Error.rot.z() - Error_last.rot.z()) / dt);
    
    cout << "Error_dot twist: " << "\n" << Error_dot << "\n";
    
    // create twist
    twist.linear.x = p_gain_*Error.vel.x() + i_gain_*Error_sum.vel.x();
    twist.linear.y = p_gain_*Error.vel.y() + i_gain_*Error_sum.vel.y(); 
    twist.linear.z = p_gain_*Error.vel.z();//p_gain_*Error.vel.z());
    twist.angular.x = 0.0;//p_gain_*Error.rot.x());//(p_gain_*Error.rot.x() + i_gain_*Error_sum.rot.x());
    twist.angular.y = 0.0;//p_gain_*Error.rot.y());//(p_gain_*Error.rot.y() + i_gain_*Error_sum.rot.y());
    twist.angular.z = p_gain_*Error.rot.z() + i_gain_*Error_sum.rot.z();

    pubTrack(1, ros::Duration(0.5), F_target);
    pubTrack(9, ros::Duration(0.5), F_current);
    pubTwistMarkers(ros::Duration(1.0), twist, F_current);
    
    Error_last.vel.x(Error.vel.x());
    Error_last.vel.y(Error.vel.y());
    Error_last.vel.z(Error.vel.z());
    Error_last.rot.x(Error.rot.x());
    Error_last.rot.y(Error.rot.y());
    Error_last.rot.z(Error.rot.z());

    return twist;
}


//VISUALIZATION IN RVIZ
//
// publish generated trajectory
void cob_cartesian_trajectories::pubTrack(const int track_id, const ros::Duration pub_duration, const KDL::Frame &F_pub)
{
    articulation_msgs::TrackMsg track;
    // set up a new TrackMsg
    if (track_map.find(track_id) == track_map.end())
    {
        cout << "new track: " << track_id << "\n";
        track.header.stamp = (ros::Time::now() - pub_duration);
        track.header.frame_id = "/map";
        track.id = track_id;
        track_map[track_id] = track;
    }

    // store pose in corresponding TrackMsg
    if ((ros::Time::now() - track_map[track_id].header.stamp) >= pub_duration)
    {
        track_map[track_id].header.stamp = ros::Time::now();
        track_map[track_id].header.frame_id = "/map";
        track_map[track_id].id = track_id;
        geometry_msgs::Pose pose; 
        

        pose.position.x = F_pub.p.x();
        pose.position.y = F_pub.p.y();
        pose.position.z = F_pub.p.z();

        F_pub.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        track_map[track_id].pose.push_back(pose);
        
        track_pub_.publish(track_map[track_id]);
    }

}

// publish twist
void cob_cartesian_trajectories::pubTwistMarkers(const ros::Duration pub_duration, const geometry_msgs::Twist &Twist, const KDL::Frame &F_current)
{
    if ((ros::Time::now() - pub_timer) >= pub_duration)
    {
        double color_mixer;
        double offset = 0.0;

        //setting up marker msg
        visualization_msgs::Marker twist_marker;
        twist_marker.header.frame_id = "/map";
        twist_marker.header.stamp = ros::Time::now();
        twist_marker.ns = "twist";
        twist_marker.id = pub_counter;
        twist_marker.type = visualization_msgs::Marker::ARROW;
        twist_marker.action = visualization_msgs::Marker::ADD;
        twist_marker.lifetime = ros::Duration();

        twist_marker.scale.x = 0.01;
        twist_marker.scale.y = 0.02;

        //calculate the color of the marker depending on twist size
        color_mixer = 11.0*sqrt(Twist.linear.x*Twist.linear.x + Twist.linear.y*Twist.linear.y + Twist.linear.z*Twist.linear.z);
        if (color_mixer > 1.0) offset = color_mixer - 1.0;
        color_mixer = color_mixer - offset;
        twist_marker.color.a = 1.0;
        twist_marker.color.g = 1.0 - color_mixer;
        twist_marker.color.r = color_mixer;

        //add two markers for start and end point of the arrow
        geometry_msgs::Point p;
        p.x = F_current.p.x();
        p.y = F_current.p.y();
        p.z = F_current.p.z();
        twist_marker.points.push_back(p);
        p.x = F_current.p.x() + Twist.linear.x;
        p.y = F_current.p.y() + Twist.linear.y;
        p.z = F_current.p.z() + Twist.linear.z;
        twist_marker.points.push_back(p);
        
        twist_pub_.publish(twist_marker);
        pub_timer = ros::Time::now();
        pub_counter++;
    }
}

//old function to publish trajectory markers
void cob_cartesian_trajectories::sendMarkers()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_values";
    marker.id = 10;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    for(unsigned int i=0; i<trajectory_points.size(); i++)
    {
        //ROS_INFO("line %f %f %f %f\n", iX1, iY1, iX2, iY2);
        marker.points.push_back(trajectory_points[i]);
    }
    //map_pub_.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_cartesian_trajectories");
    cob_cartesian_trajectories controller ;
    ros::spin();

    return 0;
}
