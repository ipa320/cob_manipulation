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

    robot_links_list.clear();
    collision_objects_list.clear();
    kinematic_list.empty();



    for (int i = 0; i < robot_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData *robot_link =
                static_cast<const collision_detection::CollisionGeometryData *>(robot_obj[i]->collisionGeometry()->getUserData());


        // Filter OT_GEOM collision primitives and replace them with Meshes
        if(robot_obj[i].get()->getObjectType() == fcl::OT_GEOM)
        {
            if(robot_obj[i].get()->collisionGeometry()->getNodeType() == fcl::GEOM_BOX)
            {
                ROS_WARN("GEOM_BOX");

                shapes::Box box(robot_obj[i].get()->collisionGeometry()->aabb_local.width(),
                                robot_obj[i].get()->collisionGeometry()->aabb_local.height(),
                                robot_obj[i].get()->collisionGeometry()->aabb_local.height());

                shapes::Mesh *m = shapes::createMeshFromShape(box);

//                m->print(std::cout);
                prt_fcl_bvh_.reset(new fcl::BVHModel<fcl::RSS>);
                prt_fcl_bvh_->beginModel();

                // ToDo: Check the vertice order.

                for(unsigned int j = 0; j < m->triangle_count-3; j++)
                {
                    for(unsigned int k = 0; k < m->vertex_count-3; k++)
                    {
                        // 8 vertices and 12 triangles for the box (Checked with m->print(std::cout);)
                        fcl::Vec3f v1(m->vertices[j*k],m->vertices[j*k+1], m->vertices[j*k+2]);
                        fcl::Vec3f v2(m->vertices[(j+1)*k],m->vertices[(j+1)*k+1], m->vertices[(j+1)*k+2]);
                        fcl::Vec3f v3(m->vertices[(j+2)*k],m->vertices[(j+2)*k+1], m->vertices[(j+2)*k+2]);

                        prt_fcl_bvh_->addTriangle(v1, v2, v3);
                    }
                }
                prt_fcl_bvh_->endModel();
                prt_fcl_bvh_->computeLocalAABB();

                // Get original Frame
                fcl::Transform3f tf = robot_obj[i]->getTransform();

                // Get new collision mesh
                fcl::CollisionObject cobj(prt_fcl_bvh_, tf);

                boost::shared_ptr<fcl::CollisionObject> ptr_co;
                ptr_co.reset(new fcl::CollisionObject(cobj));
                robot_links_list[robot_link->getID()] = ptr_co;
            }
            else if(robot_obj[i].get()->collisionGeometry()->getNodeType() == fcl::GEOM_CYLINDER)
            {
                ROS_WARN("GEOM_CYLINDER");

            }
            else if(robot_obj[i].get()->collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE)
            {
                ROS_WARN("GEOM_SPHERE");

            }
            else
            {
                ROS_WARN("Shape currently not supported.");
            }
        }
        else
        {
            robot_links_list[robot_link->getID()] = robot_obj[i];
        }


//        robot_links_list[robot_link->getID()] = robot_obj[i];

        kinematic_list.push_back(robot_link->getID());
    }
    for (int i = 0; i < world_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData *collision_object =
                static_cast<const collision_detection::CollisionGeometryData *>(world_obj[i]->collisionGeometry()->getUserData());
        collision_objects_list[collision_object->getID()] = world_obj[i];
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

bool ObstacleDistance::getRootFrame(const boost::shared_ptr<fcl::CollisionObject> co)
{
    // Set rootframe for robot_link (If the collision object is a mesh, then the rootframe will be the RobotFrame of the joint.
    // Otherwise planningFrame is the rootframe)


    if(co->getObjectType() == fcl::OT_BVH)
    {
        return false; // Frame is fine
    }
    else if(co->getObjectType() == fcl::OT_GEOM)
    {
        return true; // Should be transformed
    }
    else
    {
        ROS_ERROR("Collision Object Type not handled correctly. Handling as OT_GEOM");
        return true; // Should be transformed
    }
}

void ObstacleDistance::calculateDistances(const ros::TimerEvent& event)
{
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links_list = this->robot_links_list;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects_list = this->collision_objects_list;

    boost::mutex::scoped_lock lock(registered_links_mutex_);
    obstacle_distance::DistanceInfos distance_infos;
    
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    collision_detection::AllowedCollisionMatrix acm = planning_scene_ptr->getAllowedCollisionMatrix();
    std::string planning_frame = planning_scene_ptr->getPlanningFrame();

    collision_detection::AllowedCollision::Type type;



    std::set<std::string>::iterator link_it;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator selfcollision_it;

    for (link_it = registered_links_.begin(); link_it!=registered_links_.end(); ++link_it)
    {
        std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator obj_it;
        const boost::shared_ptr<fcl::CollisionObject> robot_link_object = robot_links_list[*link_it];

        // ToDo: Re-implement the collision objects (Published with python script)
//        for (obj_it = collision_objects_list.begin(); obj_it != collision_objects_list.end(); ++obj_it)
//        {
//            obstacle_distance::DistanceInfo info;
//            info = getDistanceInfo(robot_links_list[*link_it], collision_objects_list[obj_it->first], true);
//
//            info.header.stamp = event.current_real;
//            info.link_of_interest = *link_it;
//            info.obstacle_id = obj_it->first;
//            info.nearest_point_frame_vector.header.frame_id = getRootFrame(model_,*link_it, planning_frame);
//            info.nearest_point_obstacle_vector.header.frame_id = getRootFrame(model_,obj_it->first, planning_frame);
//
//            distance_infos.infos.push_back(info);
//        }

        for (selfcollision_it = robot_links_list.begin(); selfcollision_it != robot_links_list.end(); ++selfcollision_it)
        {
//        std::string test = "arm_right_7_link";
          std::string test = selfcollision_it->first;
            if(acm.getEntry(*link_it, test, type))
            {
                if(type == collision_detection::AllowedCollision::NEVER)
                {
                    obstacle_distance::DistanceInfo info;

                    const boost::shared_ptr<fcl::CollisionObject> selfcollision_object = robot_links_list[test];

//                    selfcollision_object->getCollisionGeometry()->aabb_center;
                    std::string robot_link_root_frame, selfcollision_link_root_frame;

                    bool do_transform_robot_link = getRootFrame(robot_link_object);
                    bool do_transform_selfcollision_object = getRootFrame(selfcollision_object);

                    info = getDistanceInfo(robot_link_object, selfcollision_object, do_transform_robot_link, do_transform_selfcollision_object);

                    info.header.stamp = event.current_real;
                    info.link_of_interest = *link_it;
                    info.obstacle_id = test;

                    info.nearest_point_frame_vector.header.frame_id = planning_frame;
                    info.nearest_point_obstacle_vector.header.frame_id = planning_frame;

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


bool ObstacleDistance::calculateDistanceCallback(obstacle_distance::GetObstacleDistance::Request &req,
                                                 obstacle_distance::GetObstacleDistance::Response &resp)
{
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > robot_links_list = this->robot_links_list;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > collision_objects_list = this->collision_objects_list;
    std::vector<std::string> kinematic_list = this->kinematic_list;

    // Chains
    for (unsigned int c=0; c< req.chains.size(); ++c)
    {
        // Link chain to objects
        bool start = false;
        for (int i = 0; i < kinematic_list.size(); i++)
        {
            if (!start && kinematic_list[i] == req.chains[c].chain_base) start = true;
            if (start) {
                if (req.objects.size() == 0)
                {
                    // All objects
                    std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator it;
                    for (it = collision_objects_list.begin(); it != collision_objects_list.end(); ++it)
                    {
                        resp.link_to_object.push_back(kinematic_list[i] + "_to_" + it->first);
                        resp.distances.push_back(ObstacleDistance::getDistanceInfo(robot_links_list[kinematic_list[i]], collision_objects_list[it->first], true, false).distance);
                    }
                }
                else
                {
                    // Specific objects
                    for (int y = 0; y < req.objects.size(); y++)
                    {
                        resp.link_to_object.push_back(kinematic_list[i] + " to " + req.objects[y]);
                        resp.distances.push_back(ObstacleDistance::getDistanceInfo(robot_links_list[kinematic_list[i]], collision_objects_list[req.objects[y]],true, false).distance);

                    }
                }

                if (kinematic_list[i] == req.chains[c].chain_tip) break;
            }
        }
    }
    
    // Links
    for (unsigned int i=0; i< req.links.size(); ++i)
    {
        if (req.objects.size() == 0)
        {
            // All objects
            std::map<std::string, boost::shared_ptr<fcl::CollisionObject> >::iterator it;
            for (it = collision_objects_list.begin(); it != collision_objects_list.end(); ++it)
            {
                resp.link_to_object.push_back(req.links[i] + "_to_" + it->first);
                resp.distances.push_back(ObstacleDistance::getDistanceInfo(robot_links_list[req.links[i]], collision_objects_list[it->first], true, false).distance);
            }
        }
        else
        {
            // Specific objects
            for (int y = 0; y < req.objects.size(); y++)
            {
                resp.link_to_object.push_back(req.links[i] + " to " + req.objects[y]);
                resp.distances.push_back(ObstacleDistance::getDistanceInfo(robot_links_list[req.links[i]], collision_objects_list[req.objects[y]], true, false).distance);
            }
        }
    }
    return true;
}

obstacle_distance::DistanceInfo ObstacleDistance::getDistanceInfo(const boost::shared_ptr<fcl::CollisionObject> robot_link,
                                                                  const boost::shared_ptr<fcl::CollisionObject> collision_object,
                                                                  bool do_transform_robot_link, bool do_transform_selfcollision_object)
{
    fcl::DistanceResult res;
    res.update(MAXIMAL_MINIMAL_DISTANCE, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE);

    double dist = fcl::distance(robot_link.get(),
                                collision_object.get(),
                                fcl::DistanceRequest(true, 0.0, 0.01),  // ToDo: Tune parameter
                                res);

    // Transformation for collision object
    fcl::CollisionObject co = *collision_object.get();
    fcl::Transform3f co_trans_fcl = co.getTransform();
    fcl::Vec3f co_translation = co_trans_fcl.getTranslation();
    fcl::Quaternion3f co_rotation = co_trans_fcl.getQuatRotation();

    Eigen::Affine3d co_trans_eigen;
    tf::Transform co_trans_tf(tf::Quaternion(co_rotation.getX(),co_rotation.getY(),co_rotation.getZ(),co_rotation.getW()),
                           tf::Vector3(co_translation[0],co_translation[1],co_translation[2]));
    tf::transformTFToEigen(co_trans_tf, co_trans_eigen);


    // Transformation for robot frame
    fcl::CollisionObject rf = *robot_link.get();
    fcl::Transform3f rf_trans_fcl = rf.getTransform();
    fcl::Vec3f rf_translation = rf_trans_fcl.getTranslation();
    fcl::Quaternion3f rf_rotation = rf_trans_fcl.getQuatRotation();

    Eigen::Affine3d rf_trans_eigen;
    tf::Transform rf_trans_tf(tf::Quaternion(rf_rotation.getX(),rf_rotation.getY(),rf_rotation.getZ(),rf_rotation.getW()),
                           tf::Vector3(rf_translation[0],rf_translation[1],rf_translation[2]));

    tf::transformTFToEigen(rf_trans_tf, rf_trans_eigen);

    Eigen::Vector3d jnt_rl_origin_to_np(res.nearest_points[0][0],
                                        res.nearest_points[0][1],
                                        res.nearest_points[0][2]);

    Eigen::Vector3d obj_origin_to_np(res.nearest_points[1][0],
                                     res.nearest_points[1][1],
                                     res.nearest_points[1][2]);

    if(!(!do_transform_robot_link && !do_transform_selfcollision_object))
    {
        obj_origin_to_np = co_trans_eigen * obj_origin_to_np;   // COLLISION OBJECT
        jnt_rl_origin_to_np = rf_trans_eigen * jnt_rl_origin_to_np; // ROBOT FRAME
    }

//    if()
//    {
//        obj_origin_to_np = co_trans_eigen * obj_origin_to_np;   // COLLISION OBJECT
//    }
//
//    if(!do_transform_robot_link && do_transform_selfcollision_object)
//    {
//        jnt_rl_origin_to_np = rf_trans_eigen * jnt_rl_origin_to_np; // ROBOT FRAME
//    }

    if (dist < 0) dist = 0;

    obstacle_distance::DistanceInfo info;
    info.distance = dist;

    tf::vectorEigenToMsg(obj_origin_to_np, info.nearest_point_obstacle_vector.vector);
    tf::vectorEigenToMsg(jnt_rl_origin_to_np, info.nearest_point_frame_vector.vector);

    return info;
}

ObstacleDistance::ObstacleDistance()
{
    MAXIMAL_MINIMAL_DISTANCE = 5.0; //m
    double update_frequency = 100.0; //Hz
    bool error = false;

    std::string robot_description = "/robot_description";
    std::string distance_service = "/calculate_distance";
    std::string register_service = "/register_links";
    std::string unregister_service = "/unregister_links";
    std::string distance_topic = "/obstacle_distances";

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    try
    {
        planning_scene_monitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description, tf_listener_);
    }
    catch (ros::InvalidNameException)
    {
        error = true;
    }

    calculate_obstacle_distance_ = nh_.advertiseService(distance_service, &ObstacleDistance::calculateDistanceCallback, this);
    register_server_ = nh_.advertiseService(register_service, &ObstacleDistance::registerCallback, this);
    unregister_server_ = nh_.advertiseService(unregister_service, &ObstacleDistance::unregisterCallback, this);
    distance_timer_ = nh_.createTimer(ros::Duration(0.1), &ObstacleDistance::calculateDistances, this);
    distance_pub_ = nh_.advertise<obstacle_distance::DistanceInfos>(distance_topic, 1);

    monitored_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("/monitored_planning_scene", 1);
    monitored_scene_server_ = nh_.advertiseService("/get_planning_scene", &ObstacleDistance::planningSceneCallback, this);
    planning_scene_timer_ = nh_.createTimer(ros::Duration(0.1), &ObstacleDistance::planningSceneTimerCallback, this);


    if (!model_.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
    }

    if (!error)
    {
        planning_scene_monitor_->setStateUpdateFrequency(update_frequency);
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->addUpdateCallback(boost::bind(&ObstacleDistance::updatedScene, this, _1));

        ROS_INFO("%s: Node started!", ros::this_node::getName().c_str());
    }
    else
    {
        ROS_ERROR("%s: Node failed!", ros::this_node::getName().c_str());
    }

    registered_links_.clear();
    distance_infos_ = obstacle_distance::DistanceInfos();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_distance_node");

    ObstacleDistance ob;
    ros::spin();

    ros::shutdown();
    return 0;
}
