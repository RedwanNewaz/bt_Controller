//
// Created by roboticslab on 11/11/22.
//

#ifndef CREATE3_CONTROLLER_VIZ_OBJECTS_H
#define CREATE3_CONTROLLER_VIZ_OBJECTS_H
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include "COM.h"

class viz_objects: public rclcpp::Node{
    using MARKER = visualization_msgs::msg::Marker;
public:
    viz_objects():Node("voz")
    {

    }

    template<class T>
    MARKER get_marker(const T&)
    {

    }

protected:
    template<class T>
    MARKER gen_path(const T& path)
    {
        MARKER marker;
        marker.header.frame_id = "map";
        rclcpp::Time t = this->get_clock()->now();
        marker.header.stamp.sec = t.seconds();
        marker.header.stamp.nanosec = t.nanoseconds();
        marker.ns = "irobot";
        marker.id = 0;
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::POINTS;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 0.75;
        marker.color.r = 0.0 ;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (int i = 1; i < path.size(); ++i) {
            geometry_msgs::msg::Point xx, yy;
            xx.x = path[i-1][0];
            xx.y = path[i-1][1];
            xx.z = path[i-1][2];

            yy.x = path[i][0];
            yy.y = path[i][1];
            yy.z = path[i][2];

            marker.points.push_back(xx);
            marker.points.push_back(yy);
        }

        return marker;

    }

};

#endif //CREATE3_CONTROLLER_VIZ_OBJECTS_H
