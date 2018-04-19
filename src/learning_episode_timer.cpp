#include <ros/ros.h>
#include "learning_episode_timer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "learning_episode_timer") ;

  ros::NodeHandle nHandle ;
  
  EpisodeTimer eTimer(nHandle) ;
  
  double ep ;
  ros::param::get("/learning/episode_length",ep) ;
  ros::Timer timer = nHandle.createTimer(ros::Duration(ep), &EpisodeTimer::shutdownCallback, &eTimer) ;
  
  ros::spin();
  return 0;
}
