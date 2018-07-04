#ifndef LEISHEN_ROS_H
#define LEISHEN_ROS_H
#include<pthread.h>
#include "package_analysis.h"
#include "leishen_odometry.h"
#include "leishen_diff.h"

#define  seril_numb 87
void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg);

#endif
