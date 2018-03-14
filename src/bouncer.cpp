#include <ros/ros.h>
#include "bouncer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bouncer") ;

  ros::NodeHandle nHandle ;
  
  bouncer bouncerPedestrian(nHandle) ;
  
  ros::spin();
  return 0;
}
