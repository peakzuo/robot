#ifndef LASER_POSE_HPP
#define LASER_POSE_HPP
#include<sensor_msgs/LaserScan.h>
#include<std_msgs/Float64.h>
#include"tf/transform_listener.h"
#include"tf/transform_broadcaster.h"
class Gmapping{
public:
    Gmapping();//constructor
	std::string base_frame_;
	std::string laser_frame_;
	std::string odom_frame_;
	std::string map_frame_;
    //tf::TransformListener tf_;

};

#endif
