#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include "laser_geometry/laser_geometry.h"
#include <vector>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign.hpp>
#include <cmath>

using namespace boost::assign;
using namespace boost::geometry;

ros::Publisher pub;

void lidarFree(const sensor_msgs::LaserScanPtr& msg)
{

    const int slice = 40;
    float adder = (msg->angle_max-msg->angle_min)/msg->ranges.size();
    float angle = msg->angle_min;
    std::vector<float> ms_angles;
    std::vector<float> separators;
    std::vector<float> ms_ranges;
    ms_angles.clear();
    separators.clear();
    ms_ranges.clear();

    int i = 0;
    while(i < msg->ranges.size())
    {
        if(msg->ranges[i] < 18 && msg->ranges[i] > 0.08)
        {
            if(adder < msg->angle_max) {ms_angles.push_back(angle);}
            ms_ranges.push_back(msg->ranges[i]);
        }
        if (i<40){separators.push_back(angle);}
        
        angle += adder;
        i++;
    }
    //Later try with STL lib 
    
    std::vector<int> m;
    float adit = ms_angles.size()/40;
    int adand = static_cast<int>(adit);
    int j = 0;
    for(const auto& sep:separators)
    {
        m.push_back(j);
        j += adand;
        
    }
    
    typedef boost::geometry::model::d2::point_xy<float> xy;
    
    boost::geometry::model::linestring<xy> line;
    for(const auto& m:m)
    {
        //std::cout << m << "-";
        float x = ms_ranges[m]*cos(ms_angles[m]);
        float y = ms_ranges[m]*sin(ms_angles[m]);
        line += xy(x,y);

    }
    
    
    boost::geometry::model::linestring<xy> simplified;
    boost::geometry::simplify(line, simplified, 0.6);
    
    visualization_msgs::Marker line_strip;

    line_strip.header.frame_id = "/laser";
    
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 1;
    line_strip.scale.x = 0.2;

    ros::Rate r(25.0);

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.color.r = 1.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;
    line_strip.pose.position.x = line_strip.pose.position.y = line_strip.pose.position.z = 0.0;
    
    while (ros::ok())
    {

        geometry_msgs::Point p,p2;
        p.x = -0.2;
        p.y = -1.0;
        p.z = 0.0;
        p2.x = -0.2;
        p2.y = 0.6;
        p2.z = 0.0;

        line_strip.points.push_back(p);
        line_strip.points.push_back(p2);
        
        for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin();it != simplified.end();it++)
        {
            geometry_msgs::Point p;
            p.x = get<0>(*it);
            p.y = get<1>(*it);
            p.z = 0.0;

            line_strip.points.push_back(p);

        }
        
        pub.publish(line_strip);
        r.sleep();

        ros::spinOnce();
    }
    
}

int main(int argc, char **argv)
{

    std::cout << "Helo" << "\n";
    ros::init(argc, argv, "free_space_lidar2d");
    ros::NodeHandle nodeh;  
    
    ros::Subscriber sub = nodeh.subscribe("/scan",1,lidarFree);
    std::cout << "Subbed" << "\n";
    pub = nodeh.advertise<visualization_msgs::Marker>("free_space_marker",1);

    ros::spin();
    
    return 0;

}