#ifndef LS_ROS_H
#define LS_ROS_H
#include<pthread.h>
#include "package_analysis.h"
#include "ls_odometry.h"
#include "ls_diff.h"

#define  seril_numb 87
void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg);

#endif
