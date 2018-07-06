#ifndef ROBOT_DOCK_HPP
#define ROBOT_DOCK_HPP
#include<ros/ros.h>
#include<kobuki_msgs/DockInfraRed.h>
#include<geometry_msgs/Twist.h>

namespace robot {
class  Robot_dock{
 public:
    Robot_dock();//constructor
    void docking_callback(const kobuki_msgs::DockInfraRed &IR_Value);
    void operation_dock();

 private:
    // the current robot states
    enum State{
        Init_state,
        Left_near,
        Left_far,
        Right_near,
        Right_far,
        Calibration_centrel,
        Suspend,
    };


    ros::Subscriber sub;
    ros::NodeHandle nh;
    ros::Publisher pub;
    geometry_msgs::Twist velocity;
    double vx;
    double vw;
    State new_state,pre_state;
};
}
#endif
