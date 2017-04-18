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

double dist_threshold_, dist_critical_, dist_mid_;

public:

    int init()
    {
        dist_threshold_ = 0.5;
        dist_critical_ = 0.1;
        dist_mid_ = (dist_threshold_ + dist_critical_)/2.0;

        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("obstacle_distance/distance_markers", 1, true);
        obstacle_distances_sub_ = this->nh_.subscribe("obstacle_distances", 1, &DebugObstacleDistance::obstacleDistancesCallback, this);

        ros::Duration(1.0).sleep();
        ROS_WARN("Distance visualization initialized.");
        return 0;
    }


    void obstacleDistancesCallback(const cob_control_msgs::ObstacleDistances::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::MarkerArray arrows, texts;

        double min_distance = std::numeric_limits<double>::max();
        unsigned int min_index = 0;

        for(unsigned int i = 0; i < msg->distances.size(); i++)
        {
            cob_control_msgs::ObstacleDistance info = msg->distances[i];
            if (info.distance < min_distance)
            {
                min_distance = info.distance;
                min_index = i;
            }
            
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

            marker_vector.color = dist_to_color(info.distance);

            marker_vector.points.push_back(start);
            marker_vector.points.push_back(end);
            arrows.markers.push_back(marker_vector);


            //show distance as text
            visualization_msgs::Marker marker_distance;
            marker_distance.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_distance.lifetime = ros::Duration();
            marker_distance.action = visualization_msgs::Marker::ADD;
            marker_distance.ns = "distances";
            marker_distance.id = 2*i+1;
            marker_distance.header.frame_id = info.header.frame_id;
            marker_distance.text = boost::lexical_cast<std::string>(boost::format("%.3f") % info.distance);

            marker_distance.scale.x = 0.05;
            marker_distance.scale.y = 0.05;
            marker_distance.scale.z = 0.05;

            marker_distance.color.a = 1.0;
            //marker_distance.color.r = 1.0;
            //marker_distance.color.g = 1.0;
            //marker_distance.color.b = 1.0;
            marker_distance.color.r = 0.0;
            marker_distance.color.g = 0.0;
            marker_distance.color.b = 0.0;

            marker_distance.pose.position.x = 0.5*(info.nearest_point_frame_vector.x+info.nearest_point_obstacle_vector.x);
            marker_distance.pose.position.y = 0.5*(info.nearest_point_frame_vector.y+info.nearest_point_obstacle_vector.y);
            marker_distance.pose.position.z = 0.5*(info.nearest_point_frame_vector.z+info.nearest_point_obstacle_vector.z);

            texts.markers.push_back(marker_distance);
        }

        //show distance severity and min_distance
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.ns = "severity";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 1.5;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color = dist_to_color(min_distance);
        marker_array.markers.push_back(marker);

        marker.ns = "min_distance";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text = boost::lexical_cast<std::string>(boost::format("%.3f") % min_distance);
        marker.pose.position.z = 1.6;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);

        marker_array.markers.insert(marker_array.markers.end(), arrows.markers.begin(), arrows.markers.end());
        marker_array.markers.insert(marker_array.markers.end(), texts.markers.begin(), texts.markers.end());

        marker_pub_.publish(marker_array);
    }

    std_msgs::ColorRGBA dist_to_color(double distance)
    {
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        if (distance < dist_critical_)
        {
            color.r = 1.0;
            color.g = 0.0;
        }
        else if (distance <= dist_mid_)
        {
            color.r = 1.0;
            color.g = (distance-dist_critical_)/(dist_mid_-dist_critical_);
        }
        else if (distance <= dist_threshold_)
        {
            color.r = (dist_threshold_-distance)/(dist_threshold_-dist_mid_);
            color.g = 1.0;
        }
        else if (distance > dist_threshold_)
        {
            color.r = 0.0;
            color.g = 1.0;
        }
        color.b = 0.0;

        return color;
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

