#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>

void shutdownCallback(const ros::TimerEvent&)
{
  ROS_INFO("Shutdown triggered.") ;
  system("rosrun multirobot_stage terminate-ros-episode") ;
  ros::shutdown() ;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "learning_episode_timer") ;
  ros::NodeHandle nh ;

//  double ep = 30.0 ;
  ros::param::get("learning/episode_length",ep) ;

  ros::Timer timer = nh.createTimer(ros::Duration(ep), shutdownCallback, true) ;

  ros::spin() ;

  return 0 ;
}
