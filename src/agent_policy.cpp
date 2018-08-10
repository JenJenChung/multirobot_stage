#include <ros/ros.h>
#include "AgentPolicy.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent_policy") ;
  
  ros::NodeHandle n_handle ;
  
  AgentPolicy policy(n_handle) ;
  
  ros::spin() ;
  return 0 ;
}
