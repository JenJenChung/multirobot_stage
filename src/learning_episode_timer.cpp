#include <ros/ros.h>
#include <ros/console.h>
#include "learning_episode_timer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "learning_episode_timer") ;

  ros::NodeHandle nHandle ;
  
  EpisodeTimer eTimer(nHandle) ;
  
  double ep ;
  ros::param::get("/episode_length",ep) ;
  
  ROS_INFO_STREAM("Each episode length: " << ep) ;
  
  ros::Timer timer = nHandle.createTimer(ros::Duration(ep), &EpisodeTimer::shutdownCallback, &eTimer) ;
  
  ros::spin();
  return 0;
}
