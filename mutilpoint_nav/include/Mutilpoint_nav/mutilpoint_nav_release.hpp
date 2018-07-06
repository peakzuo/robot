#ifndef MUTILPONIT_NAV_RELEASE_H
#define MUTILPONIT_NAV_RELEASE_H
#include<ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <ecl/exceptions.hpp>
#include <ecl/time.hpp>
#include "./yaml_parser.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseclient;

#define number 4

class Mutil_Point{
 public:
   Mutil_Point();//constractor

   void Initposecallback(const geometry_msgs::PoseWithCovarianceStamped &initialPose);
   void posecallback(const geometry_msgs::PoseWithCovarianceStamped &Now_pose);
   void set_point(const std_msgs::Int16 &data);
   void Get_startstate(bool &sign) {if(first_state==true&&second_sate==true) sign=true;else sign=false;}
   void Get_file_name(std::string &name){ name=filename; }

 private:
   std::string  frame_id;
   double x;
   double y;
   double w;
   bool first_state;
   bool second_sate;
   ros::NodeHandle nh;
   ros::Subscriber sub_initpose,sub_pose,sub_key;


   geometry_msgs::PoseWithCovarianceStamped Get_pose;
   YAML::Node node,wp_node;

   std::string filename;

};


#endif
