#ifndef AGV_RAISE_AND_DOWN_H
#define AGV_RAISE_AND_DOWN_H
#include<ros/ros.h>
#include<kobuki_msgs/Led.h>
#include<std_msgs/Int16.h>
#include<kobuki_msgs/SensorState.h>

#define Loading 1
#define Unloading 2

namespace agv {
class Agv_sign{
 public:
    Agv_sign();//constructor
    void dest_signcallback(const std_msgs::Int16 &value);
    void obst_signcallback(const kobuki_msgs::SensorStateConstPtr &core_data);
 private:
    ros::NodeHandle nh;
    ros::Subscriber sub_dest_sign;
    ros::Subscriber sub_obst_sign;
    ros::Publisher  pub;

    kobuki_msgs::Led Oparetion_sign;
};
}
#endif
