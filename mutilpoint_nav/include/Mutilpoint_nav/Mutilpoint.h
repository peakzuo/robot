#ifndef MUTILPONIT_H
#define MUTILPONIT_H
#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<string.h>
#include "std_msgs/String.h"
#include <termios.h> // for keyboard input
#include <ecl/threads.hpp>
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
   ~Mutil_Point();//destructor
   bool init();
   void Initposecallback(const geometry_msgs::PoseWithCovarianceStamped &initialPose);
   void posecallback(const geometry_msgs::PoseWithCovarianceStamped &Now_pose);
   void keyboardInputLoop();
   void set_point();
   void Get_startstate(bool &sign) {if(start_state==true&&second_sate==true) sign=true;else sign=false;}
   void Get_file_name(std::string &name){ name=filename; }

 private:
   std::string  frame_id;
   double x;
   double y;
   double w;
   bool start_state;
   bool second_sate;
   ros::NodeHandle nh;
   ros::Subscriber sub_initpose,sub_pose;

   int key_file_descriptor_;
   struct termios original_terminal_state_;
   ecl::Thread thread_;

   geometry_msgs::PoseWithCovarianceStamped Get_pose;
   YAML::Node node,wp_node;

   std::string filename;

};


#endif
