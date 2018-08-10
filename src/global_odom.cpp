#include <ros/ros.h>
#include "GlobalOdom.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_odom") ;
  
  ros::NodeHandle n_handle ;
  
  GlobalOdom g_odom(n_handle) ;
  
  ros::spin() ;
  return 0 ;
}
