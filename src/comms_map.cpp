#include <ros/ros.h>
#include "CommsMap.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "comms_map") ;
  
  ros::NodeHandle n_handle ;
  
  CommsMap comms_map(n_handle) ;
  
  ros::spin() ;
  return 0 ;
}
