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
#include <cob_control_msgs/ObstacleDistances.h>


class DebugObstacleDistance
{
ros::NodeHandle nh_;
ros::Publisher marker_pub_;
ros::Subscriber obstacle_distances_sub_;

public:

    int init()
    {
        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("obstacle_distance/distance_markers", 1, true);
        obstacle_distances_sub_ = this->nh_.subscribe("obstacle_distances", 1, &DebugObstacleDistance::obstacleDistancesCallback, this);

        ros::Duration(1.0).sleep();
        ROS_WARN("Debug initialized.");
        return 0;
    }


    void obstacleDistancesCallback(const cob_control_msgs::ObstacleDistances::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;

        for(unsigned int i = 0; i < msg->distances.size(); i++)
        {
            cob_control_msgs::ObstacleDistance info = msg->distances[i];
            
            //show distance vector as arrow
            visualization_msgs::Marker marker_vector;
            marker_vector.type = visualization_msgs::Marker::ARROW;
            marker_vector.lifetime = ros::Duration();
            marker_vector.action = visualization_msgs::Marker::ADD;
            marker_vector.ns = "arrows";
            marker_vector.id = 2*i;
            marker_vector.header.frame_id = info.header.frame_id;

            marker_vector.scale.x = 0.01;
            marker_vector.scale.y = 0.05;

            geometry_msgs::Point start;
            start.x = info.nearest_point_obstacle_vector.x;
            start.y = info.nearest_point_obstacle_vector.y;
            start.z = info.nearest_point_obstacle_vector.z;

            geometry_msgs::Point end;
            end.x = info.nearest_point_frame_vector.x;
            end.y = info.nearest_point_frame_vector.y;
            end.z = info.nearest_point_frame_vector.z;

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
            marker_distance.ns = "distances";
            marker_distance.id = 2*i+1;
            marker_distance.header.frame_id = info.header.frame_id;
            marker_distance.text = boost::lexical_cast<std::string>(boost::format("%.3f") % info.distance);

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

            marker_distance.pose.position.x = info.nearest_point_frame_vector.x;
            marker_distance.pose.position.y = info.nearest_point_frame_vector.y + 0.05;
            marker_distance.pose.position.z = info.nearest_point_frame_vector.z;

            marker_array.markers.push_back(marker_distance);
        }

        marker_pub_.publish(marker_array);
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

