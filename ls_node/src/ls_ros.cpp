#include<ros/ros.h>
#include  "../include/ls_node/ls_ros.h"

unsigned char tmp[seril_numb]={0};
double angle,angle_rate;
void *Receive_SerialData(void*data)
{
   
  int ret,ret1;
   while(1)
   {
     //serial port receive data then analysis
    ret=package_Get.GetPackage(tmp,seril_numb);
    if(ret>=seril_numb){

      ret1=package_Get.Data_analysis(tmp,&demand_data);
      if(ret1==true){
            angle = demand_data.Angle;
            angle_rate = demand_data.Angle_rate;
          } 
     }

   }

}



void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  //if (kobuki.isEnabled())
  {
    // For now assuming this is in the robot frame, but probably this
    // should be global frame and require a transform
    //double vx = msg->linear.x;        // in (m/s)
    //double wz = msg->angular.z;       // in (rad/s)
    ROS_DEBUG_STREAM("Kobuki : velocity command received [" << msg->linear.x << "],[" << msg->angular.z << "]");
    //diff_drive.setBaseControl(msg->linear.x, msg->angular.z);
    diff_drive.setVelocityCommands(msg->linear.x, msg->angular.z);
    //ROS_INFO("linear.x=%f,angular.z=%f",msg->linear.x,msg->angular.z);
    //odometry.resetTimeout();
  }
  return;
}



/*****************************************************************************
** Implementation}
*****************************************************************************/
int main(int argc, char **argv)
 {

  pthread_t threadId;
  int ret;
  bool ret1;
  int ret2;
  int read_StartHeading_count;
 // double angle,angle_rate;
  short speed=0,radius=0;
 // unsigned char tmp[seril_numb]={0};
  ros::init(argc,argv,"leishen_node");
  ros::NodeHandle node;

  std::string name ="leishen_node";
 
  Odometry odometry;
  // Take latest encoders and gyro data
  Pose_2d pose_update;
  Pose_3d pose_update_rates;

  ret=package_Get.Init_serial();
  if(ret < 0){
    ROS_INFO("Can't read the data through the serial port!");
  }


 //create thread
 ret2 = pthread_create(&threadId,NULL,Receive_SerialData,NULL);
 if(ret2 !=0)
 {
   ROS_INFO("create thread error!");
 }



 //read startHeading
 for(read_StartHeading_count=0;read_StartHeading_count <= 5;read_StartHeading_count++){
  ret=package_Get.GetPackage(tmp,seril_numb);
  if(ret>=seril_numb){
      ret1=package_Get.Data_analysis(tmp,&demand_data);
      if(ret1 == true){
         ROS_INFO("Data_analysis is ok!");
         angle = demand_data.Angle;
         angle_rate = demand_data.Angle_rate;
         break;
      }
   }

 }

  ros::Subscriber  velocity_command_subscriber = node.subscribe(std::string("/mobile_base/commands/velocity"), 10, &subscribeVelocityCommand);
  ROS_INFO("setup success!");

  odometry.init(node, name);

  odometry.heading_offset = 0.0/0.0;

  if (std::isnan(odometry.heading_offset) == true)
  odometry.heading_offset = (static_cast<double>(angle) / 100.0) * pi / 180.0;

  ros::Rate loop_rate(40);

  while(node.ok()){
 
    //get odometry
    diff_drive.updateOdometry(pose_update, pose_update_rates);

    //Update and publish odometry
    odometry.update(pose_update, pose_update_rates, odometry.getHeading(angle,odometry.heading_offset),odometry.getAngularVelocity(angle_rate));

    //serial port send data
    diff_drive.sendBaseControlCommand(speed,radius);
    package_Get.SendData(speed,radius);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
 }

