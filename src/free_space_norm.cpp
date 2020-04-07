
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

class LidarFree
{
    private:
    std::vector<float> ms_angles;
    std::vector<float> separators;
    std::vector<float> ms_ranges;
    std::vector<int> m;
    typedef boost::geometry::model::d2::point_xy<float> xy;
    boost::geometry::model::linestring<xy> line;
    boost::geometry::model::linestring<xy> simplified;

    public:
    /*
    void Publisher()
    {
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
        
        while (ros::ok)
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
        }
    }*/
    void simplyfy()
    {
        for(const auto& m:m)
        {
            float x = ms_ranges[m]*cos(ms_angles[m]);
            float y = ms_ranges[m]*sin(ms_angles[m]);
            line += xy(x,y);
            std::cout << "X:" << x << "Y:" << y << '\n';
        }
        boost::geometry::simplify(line, simplified, 0.4);
    }
    void lidarData(const sensor_msgs::LaserScanPtr& msg)
    {

        
        const int slice = 40;
        float adder = (msg->angle_max-msg->angle_min)/msg->ranges.size();
        float angle = msg->angle_min;

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
        float adit = ms_angles.size()/40;
        int adand = static_cast<int>(adit);
        int j = 0;
        for(const auto& sep:separators)
        {
            m.push_back(j);
            j += adand;
        }
        simplyfy();
    }
    
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "free_space_lidar2d");
    
    ros::NodeHandle nodeh;  
    LidarFree lid;
    ros::Subscriber sub = nodeh.subscribe("/scan",1,&LidarFree::lidarData, &lid);

    pub = nodeh.advertise<visualization_msgs::Marker>("free_space_marker",1);
    
    ros::spin();

    return 0;

}