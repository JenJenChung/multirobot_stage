#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>

#include "std_msgs/Float64.h"

class EpisodeTimer{
  public:
    EpisodeTimer(ros::NodeHandle) ;
    ~EpisodeTimer(){}
    void shutdownCallback(const ros::TimerEvent&) ;
  private:
    ros::Subscriber subReward ;
    ros::Publisher pubReward ;
    
    double curReward ;
    
    void rewardCallback(const std_msgs::Float64&) ;
};

EpisodeTimer::EpisodeTimer(ros::NodeHandle nh){
  subReward = nh.subscribe("/reward", 10, &EpisodeTimer::rewardCallback, this) ;
  pubReward = nh.advertise<std_msgs::Float64>("/episode_result", 10, true) ;
  
  curReward = 0.0 ;
}

void EpisodeTimer::rewardCallback(const std_msgs::Float64& msg){
  curReward = msg.data ;
}

void EpisodeTimer::shutdownCallback(const ros::TimerEvent&){
  std_msgs::Float64 finalReward ;
  finalReward.data = curReward ;
  
  ROS_INFO("Shutdown triggered.") ;
  system("rosrun multirobot_stage terminate-ros-episode") ;
  pubReward.publish(finalReward) ;
  ros::shutdown() ;
}

