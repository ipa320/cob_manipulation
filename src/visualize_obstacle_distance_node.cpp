/*
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_obstacle_distance
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Debug node visualizing the vector to the closest obstacle as well as the distance value through visualization_msgs/Marker.
 ****************************************************************/

#include <string.h>
#include <map>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <obstacle_distance/obstacle_distance.h>


class DebugObstacleDistance
{
ros::NodeHandle nh_;
ros::Publisher marker_pub_;
ros::Subscriber obstacle_distances_sub_;

Eigen::Affine3d transform_;
tf::TransformListener tf_listener_;

public:

    int init()
    {

        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("obstacle_distance/distance_markers", 1, true);
        obstacle_distances_sub_ = this->nh_.subscribe("obstacle_distances", 1, &DebugObstacleDistance::obstacleDistancesCallback, this);

        ros::Duration(1.0).sleep();
        ROS_WARN("Debug initialized.");
        return 0;
    }


    void obstacleDistancesCallback(const obstacle_distance::DistanceInfos::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;
        std::map<std::string, std::vector<obstacle_distance::DistanceInfo> > relevant_obstacle_distances;

        for(uint32_t i=0; i < msg->infos.size(); i++)
        {
            const std::string id = msg->infos[i].link_of_interest;

//                if(relevant_obstacle_distances[id].distance > msg->infos[i].distance)
//                {
                    relevant_obstacle_distances[id].push_back(msg->infos[i]);
//                }

        }

        for(std::map<std::string, std::vector<obstacle_distance::DistanceInfo> >::const_iterator it = relevant_obstacle_distances.begin();
                it != relevant_obstacle_distances.end(); ++it)
        {
            for(unsigned i = 0; i < it->second.size(); i++)
            {
                //show distance vector as arrow
                visualization_msgs::Marker marker_vector;
                marker_vector.type = visualization_msgs::Marker::ARROW;
                marker_vector.lifetime = ros::Duration();
                marker_vector.action = visualization_msgs::Marker::ADD;
                marker_vector.ns = it->first;
                marker_vector.id = 2*i;
                marker_vector.header.frame_id = "base_link";

                marker_vector.scale.x = 0.01;
                marker_vector.scale.y = 0.05;

                Eigen::Vector3d obst_np(it->second[i].nearest_point_obstacle_vector.vector.x,
                                        it->second[i].nearest_point_obstacle_vector.vector.y,
                                        it->second[i].nearest_point_obstacle_vector.vector.z);
                Eigen::Vector3d frame_np(it->second[i].nearest_point_frame_vector.vector.x,
                                         it->second[i].nearest_point_frame_vector.vector.y,
                                         it->second[i].nearest_point_frame_vector.vector.z);

                // Transform the coordinate systems
                Eigen::Vector3d obst_bl_np = transform(marker_vector.header.frame_id , it->second[i].nearest_point_obstacle_vector.header.frame_id) * obst_np;
                Eigen::Vector3d frame_bl_np = transform(marker_vector.header.frame_id , it->second[i].nearest_point_frame_vector.header.frame_id) * frame_np;

                geometry_msgs::Point start;
                start.x = obst_bl_np[0];
                start.y = obst_bl_np[1];
                start.z = obst_bl_np[2];

                geometry_msgs::Point end;
                end.x = frame_bl_np[0];
                end.y = frame_bl_np[1];
                end.z = frame_bl_np[2];

                marker_vector.color.a = 1.0;
                marker_vector.color.g = 1.0;

                marker_vector.points.push_back(start);
                marker_vector.points.push_back(end);
                marker_array.markers.push_back(marker_vector);


                //show distance as text
                visualization_msgs::Marker marker_distance;
                marker_distance.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker_distance.lifetime = ros::Duration();
                marker_distance.action = visualization_msgs::Marker::ADD;
                marker_distance.ns = it->first;
                marker_distance.id = 2*i+1;
                marker_distance.header.frame_id = "base_link";
                marker_distance.text = boost::lexical_cast<std::string>(boost::format("%.3f") % it->second[i].distance);

                marker_distance.scale.x = 0.1;
                marker_distance.scale.y = 0.1;
                marker_distance.scale.z = 0.1;

                marker_distance.color.a = 1.0;
                //marker_distance.color.r = 1.0;
                //marker_distance.color.g = 1.0;
                //marker_distance.color.b = 1.0;
                marker_distance.color.r = 0.0;
                marker_distance.color.g = 0.0;
                marker_distance.color.b = 0.0;

                marker_distance.pose.position.x = frame_bl_np[0];
                marker_distance.pose.position.y = frame_bl_np[1] + 0.05;
                marker_distance.pose.position.z = frame_bl_np[2];

                marker_array.markers.push_back(marker_distance);
            }
        }

        marker_pub_.publish(marker_array);
    }

    Eigen::Affine3d transform(std::string target, std::string source)
    {

        Eigen::Affine3d ret;
        try
        {
            tf::StampedTransform transform;
            ros::Time time = ros::Time::now();
            if(tf_listener_.waitForTransform(target, source, time, ros::Duration(1.0)))
            {
                tf_listener_.lookupTransform(target, source, time, transform);
                tf::transformTFToEigen(transform, ret);
            }
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        return ret;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_obstacle_distance_node");

    DebugObstacleDistance dod;
    if (dod.init() != 0)
    {
        ROS_ERROR("Failed to initialize DebugDistanceManager.");
        return -1;
    }

    ros::spin();
}

