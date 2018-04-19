#include <ros/ros.h>
#include "multiRobotNE.h"

using std::vector ;
using std::shuffle ;
using namespace Eigen ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multiRobotNE") ;

  ros::NodeHandle nHandle ;
  
  MultiRobotNE testMultiRobotNE(nHandle) ;
  
  ros::spin();
  return 0 ;
}
