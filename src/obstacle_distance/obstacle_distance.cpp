#include <obstacle_distance/obstacle_distance.h>

class CreateCollisionWorld : public collision_detection::CollisionWorldFCL
{
public:
    CreateCollisionWorld(const collision_detection::WorldPtr &world) :
            CollisionWorldFCL(world)
    {}

    void getCollisionObject(std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj)
    {
        std::map<std::string, collision_detection::FCLObject>::iterator it = fcl_objs_.begin();
        obj.reserve(fcl_objs_.size());

        for (it; it != fcl_objs_.end(); ++it)
        {
            obj.push_back(it->second.collision_objects_[0]);
        }
    }
};

class CreateCollisionRobot : public collision_detection::CollisionRobotFCL
{
public:
    CreateCollisionRobot(const robot_model::RobotModelConstPtr &model) :
            CollisionRobotFCL(model)
    {}

    void getCollisionObject(const robot_state::RobotState &state,
                            std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj)
    {
        collision_detection::FCLObject fcl_obj;
        constructFCLObject(state, fcl_obj);
        obj = fcl_obj.collision_objects_;
    }
};

void ObstacleDistance::updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
{
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    std::vector<boost::shared_ptr<fcl::CollisionObject> > robot_obj, world_obj;
    robot_state::RobotState robot_state(planning_scene_ptr->getCurrentState());

    CreateCollisionWorld collision_world(planning_scene_ptr->getWorldNonConst());
    collision_world.getCollisionObject(world_obj);

    CreateCollisionRobot collision_robot(robot_state.getRobotModel());
    collision_robot.getCollisionObject(robot_state, robot_obj);

    this->robot_links_.clear();
    this->collision_objects_.clear();

    for (unsigned int i = 0; i < robot_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData *robot_link =
                static_cast<const collision_detection::CollisionGeometryData *>(robot_obj[i]->collisionGeometry()->getUserData());
        this->robot_links_[robot_link->getID()] = robot_obj[i];
    }
    for (unsigned int i = 0; i < world_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData *collision_object =
                static_cast<const collision_detection::CollisionGeometryData *>(world_obj[i]->collisionGeometry()->getUserData());
        this->collision_objects_[collision_object->getID()] = world_obj[i];
    }
}

bool ObstacleDistance::planningSceneCallback(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res)
{
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    planning_scene_ptr->getPlanningSceneMsg(res.scene, req.components);

    return true;
}

bool ObstacleDistance::registerCallback(cob_srvs::SetString::Request &req,
                                        cob_srvs::SetString::Response &res)
{
    boost::mutex::scoped_lock lock(registered_links_mutex_);
    std::pair<std::set<std::string>::iterator, bool> ret = registered_links_.insert(req.data);

    res.success = true;
    res.message = (ret.second) ? (req.data + " successfully registered") : (req.data + " already registered");
    return true;
}

bool ObstacleDistance::unregisterCallback(cob_srvs::SetString::Request &req,
                                          cob_srvs::SetString::Response &res)
{
    boost::mutex::scoped_lock lock(registered_links_mutex_);
    std::set<std::string>::iterator it = registered_links_.find(req.data);
    
    if (it != registered_links_.end())
    {
        registered_links_.erase(it);

        res.success = true;
        res.message = req.data + " successfully unregistered";
    }
    else
    {
        res.success = true;
        res.message = req.data + " has not been registered before";
    }

    return true;
}

void ObstacleDistance::calculateDistanceTimerCallback(const ros::TimerEvent& event)
{
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links = this->robot_links_;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects = this->collision_objects_;

    boost::mutex::scoped_lock lock(registered_links_mutex_);
    obstacle_distance::DistanceInfos distance_infos;
    
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    collision_detection::AllowedCollisionMatrix acm = planning_scene_ptr->getAllowedCollisionMatrix();
    std::string planning_frame = planning_scene_ptr->getPlanningFrame();

    std::set<std::string>::iterator link_it;
    for (link_it = registered_links_.begin(); link_it!=registered_links_.end(); ++link_it)
    {
        std::string robot_link_name = *link_it;
        const boost::shared_ptr<fcl::CollisionObject> robot_object = robot_links[robot_link_name];
        ROS_DEBUG_STREAM("RobotLink: " << robot_link_name << ", Type: " << robot_object->getObjectType());
        
        ///// testing
        //std::string robot_link_name = "test_primitive";
        //if (collision_objects.find(robot_link_name) == collision_objects.end())
        //{
            //ROS_INFO("Object not yet available");
            //return;
        //}
        //const boost::shared_ptr<fcl::CollisionObject> robot_object = collision_objects[robot_link_name];
        //ROS_DEBUG_STREAM("RobotLink: " << robot_link_name << ", Type: " << robot_object->getObjectType());

        std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator obj_it;
        for (obj_it = collision_objects.begin(); obj_it != collision_objects.end(); ++obj_it)
        {
            std::string collision_object_name = obj_it->first;
            const boost::shared_ptr<fcl::CollisionObject> collision_object = collision_objects[collision_object_name];
            ROS_DEBUG_STREAM("CollisionLink: " << collision_object_name << ", Type: " << collision_object->getObjectType());

            obstacle_distance::DistanceInfo info;
            info = getDistanceInfo(robot_object, collision_object);

            info.header.frame_id = planning_frame;
            info.header.stamp = event.current_real;
            info.link_of_interest = robot_link_name;
            info.obstacle_id = collision_object_name;

            distance_infos.infos.push_back(info);
        }

        std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator selfcollision_it;
        for (selfcollision_it = robot_links.begin(); selfcollision_it != robot_links.end(); ++selfcollision_it)
        {
            std::string robot_self_name = selfcollision_it->first;
            collision_detection::AllowedCollision::Type type;
            if(acm.getEntry(robot_link_name, robot_self_name, type))
            {
                if(type == collision_detection::AllowedCollision::NEVER)
                {
                    const boost::shared_ptr<fcl::CollisionObject> robot_self_object = robot_links[robot_self_name];
                    ROS_DEBUG_STREAM("CollisionLink: " << robot_self_name << ", Type: " << robot_self_object->getObjectType());
                    
                    obstacle_distance::DistanceInfo info;
                    info = getDistanceInfo(robot_object, robot_self_object);

                    info.header.frame_id = planning_frame;
                    info.header.stamp = event.current_real;
                    info.link_of_interest = robot_link_name;
                    info.obstacle_id = robot_self_name;

                    distance_infos.infos.push_back(info);
                }
                else
                {
                    // This is diagonal of allowed collision matrix
                }
            }
        }
    }
    distance_pub_.publish(distance_infos);
}

void ObstacleDistance::planningSceneTimerCallback(const ros::TimerEvent& event)
{
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    moveit_msgs::PlanningScene scene;
    planning_scene_ptr->getPlanningSceneMsg(scene);
    monitored_scene_pub_.publish(scene);
}


bool ObstacleDistance::calculateDistanceServiceCallback(obstacle_distance::GetObstacleDistance::Request &req,
                                                        obstacle_distance::GetObstacleDistance::Response &resp)
{
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links = this->robot_links_;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects = this->collision_objects_;
    
    // Links
    for (unsigned int i=0; i< req.links.size(); ++i)
    {
        if (req.objects.size() == 0)
        {
            // All objects
            std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator it;
            for (it = collision_objects.begin(); it != collision_objects.end(); ++it)
            {
                resp.link_to_object.push_back(req.links[i] + "_to_" + it->first);
                resp.distances.push_back(ObstacleDistance::getDistanceInfo(robot_links[req.links[i]], collision_objects[it->first]).distance);
            }
        }
        else
        {
            // Specific objects
            for (int y = 0; y < req.objects.size(); y++)
            {
                resp.link_to_object.push_back(req.links[i] + " to " + req.objects[y]);
                resp.distances.push_back(ObstacleDistance::getDistanceInfo(robot_links[req.links[i]], collision_objects[req.objects[y]]).distance);
            }
        }
    }
    return true;
}

obstacle_distance::DistanceInfo ObstacleDistance::getDistanceInfo(const boost::shared_ptr<fcl::CollisionObject> robot_link,
                                                                  const boost::shared_ptr<fcl::CollisionObject> collision_object)
{
    fcl::DistanceRequest req(true);  // enable_nearest_points
    fcl::DistanceResult res;
    res.update(MAXIMAL_MINIMAL_DISTANCE, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE);

    Eigen::Vector3d jnt_rl_origin_to_np;
    Eigen::Vector3d obj_origin_to_np;

    double dist = fcl::distance(robot_link.get(), collision_object.get(), req, res);

    // this is to prevent what seems to be a nasty bug in fcl
    if(robot_link->getObjectType() == fcl::OT_GEOM && collision_object->getObjectType() == fcl::OT_BVH)
    {
        // res.nearest_points seem swapped in this case
        jnt_rl_origin_to_np(0) = res.nearest_points[1][0];
        jnt_rl_origin_to_np(1) = res.nearest_points[1][1];
        jnt_rl_origin_to_np(2) = res.nearest_points[1][2];

        obj_origin_to_np(0) = res.nearest_points[0][0];
        obj_origin_to_np(1) = res.nearest_points[0][1];
        obj_origin_to_np(2) = res.nearest_points[0][2];
    }
    else
    {
        jnt_rl_origin_to_np(0) = res.nearest_points[0][0];
        jnt_rl_origin_to_np(1) = res.nearest_points[0][1];
        jnt_rl_origin_to_np(2) = res.nearest_points[0][2];

        obj_origin_to_np(0) = res.nearest_points[1][0];
        obj_origin_to_np(1) = res.nearest_points[1][1];
        obj_origin_to_np(2) = res.nearest_points[1][2];
    }
    // ToDo: are there other cases? OT_OCTREE? see fcl::OBJECT_TYPE

    geometry_msgs::Vector3 rl_np_msg;
    tf::vectorEigenToMsg(jnt_rl_origin_to_np, rl_np_msg);
    ROS_DEBUG_STREAM("NearestPoint RL: \n" << rl_np_msg);

    geometry_msgs::Vector3 obj_np_msg;
    tf::vectorEigenToMsg(obj_origin_to_np, obj_np_msg);
    ROS_DEBUG_STREAM("NearestPoint OBJ: \n" << obj_np_msg);

    // Transformation for robot frame
    fcl::CollisionObject rf = *robot_link.get();
    fcl::Transform3f rf_trans_fcl = rf.getTransform();
    fcl::Vec3f rf_translation = rf_trans_fcl.getTranslation();
    fcl::Quaternion3f rf_rotation = rf_trans_fcl.getQuatRotation();

    Eigen::Affine3d rf_trans_eigen;
    tf::Transform rf_trans_tf(tf::Quaternion(rf_rotation.getX(),rf_rotation.getY(),rf_rotation.getZ(),rf_rotation.getW()),
                           tf::Vector3(rf_translation[0],rf_translation[1],rf_translation[2]));
    tf::transformTFToEigen(rf_trans_tf, rf_trans_eigen);

    geometry_msgs::Transform rf_trans_msg;
    tf::transformTFToMsg(rf_trans_tf, rf_trans_msg);
    ROS_DEBUG_STREAM("Transform RL: \n" << rf_trans_msg);

    // Transformation for collision object
    fcl::CollisionObject co = *collision_object.get();
    fcl::Transform3f co_trans_fcl = co.getTransform();
    fcl::Vec3f co_translation = co_trans_fcl.getTranslation();
    fcl::Quaternion3f co_rotation = co_trans_fcl.getQuatRotation();

    Eigen::Affine3d co_trans_eigen;
    tf::Transform co_trans_tf(tf::Quaternion(co_rotation.getX(),co_rotation.getY(),co_rotation.getZ(),co_rotation.getW()),
                           tf::Vector3(co_translation[0],co_translation[1],co_translation[2]));
    tf::transformTFToEigen(co_trans_tf, co_trans_eigen);

    geometry_msgs::Transform co_trans_msg;
    tf::transformTFToMsg(co_trans_tf, co_trans_msg);
    ROS_DEBUG_STREAM("Transform OBJ: \n" << co_trans_msg);

    //  in case both objects are of OBJECT_TYPE OT_BVH the nearest points are already given in PlanningFrame coordinates
    if(!(robot_link->getObjectType() == fcl::OT_BVH && collision_object->getObjectType() == fcl::OT_BVH))
    {
        jnt_rl_origin_to_np = rf_trans_eigen * jnt_rl_origin_to_np;
        obj_origin_to_np = co_trans_eigen * obj_origin_to_np;
    }
    // ToDo: are there other cases? OT_OCTREE? see fcl::OBJECT_TYPE

    if (dist < 0) dist = 0;

    obstacle_distance::DistanceInfo info;
    info.distance = dist;

    tf::vectorEigenToMsg(jnt_rl_origin_to_np, info.nearest_point_frame_vector);
    tf::vectorEigenToMsg(obj_origin_to_np, info.nearest_point_obstacle_vector);    
    ROS_DEBUG_STREAM("NearestPointTransformed RL: \n" << info.nearest_point_frame_vector);
    ROS_DEBUG_STREAM("NearestPointTransformed OBJ: \n" << info.nearest_point_obstacle_vector);

    return info;
}

ObstacleDistance::ObstacleDistance()
{
    MAXIMAL_MINIMAL_DISTANCE = 5.0; //m
    double update_frequency = 50.0; //Hz

    std::string robot_description = "/robot_description";
    std::string distance_service = "/calculate_distance";
    std::string register_service = "/register_links";
    std::string unregister_service = "/unregister_links";
    std::string distance_topic = "/obstacle_distances";

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description, tf_listener_);

    planning_scene_monitor_->setStateUpdateFrequency(update_frequency);
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->addUpdateCallback(boost::bind(&ObstacleDistance::updatedScene, this, _1));

    registered_links_.clear();

    calculate_obstacle_distance_ = nh_.advertiseService(distance_service, &ObstacleDistance::calculateDistanceServiceCallback, this);
    register_server_ = nh_.advertiseService(register_service, &ObstacleDistance::registerCallback, this);
    unregister_server_ = nh_.advertiseService(unregister_service, &ObstacleDistance::unregisterCallback, this);
    distance_timer_ = nh_.createTimer(ros::Duration(1.0/update_frequency), &ObstacleDistance::calculateDistanceTimerCallback, this);
    distance_pub_ = nh_.advertise<obstacle_distance::DistanceInfos>(distance_topic, 1);

    monitored_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("/monitored_planning_scene", 1);
    monitored_scene_server_ = nh_.advertiseService("/get_planning_scene", &ObstacleDistance::planningSceneCallback, this);
    planning_scene_timer_ = nh_.createTimer(ros::Duration(1.0/update_frequency), &ObstacleDistance::planningSceneTimerCallback, this);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_distance_node");

    ObstacleDistance ob;
    ros::spin();

    ros::shutdown();
    return 0;
}
