#include <ros/ros.h>
#include "multiRobotPG.h"

using std::vector ;
using std::shuffle ;
using namespace Eigen ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multiRobotPG") ;

  ros::NodeHandle nHandle ;
  
  MultiRobotPG testMultiRobotPG(nHandle) ;
  
  ros::spin();
  return 0 ;
}
