#include <iostream>
#include <stdlib.h>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <Eigen/Eigen>
#include <float.h>

#include "multirobot_stage/PolicyGrad.h"
#include "multirobot_stage/NeuralNet.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>

class MultiRobotPG{
  public:
    MultiRobotPG(ros::NodeHandle) ;
    ~MultiRobotPG(){};
  private:
    bool headless;
    int nRob ;     // number of robots
    int nIn ;      // input state vector size
    int nOut ;     // output action vector size
    int nHidden ;  // number of hidden neurons
    int nEps ;     // number of learning epochs
    actFun afType;
    
    int epochCount ;   // epoch counter (number of evolutions)
    
    unsigned seed ;
    vector<double> rewards ;
    vector<double> rewardLog ;
    //double maxR ;
    //int maxTeam ;
    
    ros::Subscriber subResult ;
    void episodeCallback(const std_msgs::Float64&) ;
    
    void NewEpisode() ;
};

MultiRobotPG::MultiRobotPG(ros::NodeHandle nh){
  
  ROS_INFO("Initialising multi-robot policy-gradient...") ;
  
  // Initialise learning domain
  // Domain contains 2 PolicyGrad agents, each agent maintains a population of policies which are encoded as neural networks
  // Neural networks are initialised according to size of input and output vectors, number of nodes in the hidden layer, and the activation function
  ros::param::get("headless", headless);
  ros::param::get("/learning/nRob", nRob);
  ros::param::get("/learning/nIn", nIn);
  ros::param::get("/learning/nOut", nOut);
  ros::param::get("/learning/nHidden", nHidden);
  ros::param::get("/learning/nEps", nEps);
  std::string aFun;
  ros::param::get("/learning/actFun", aFun);

  if (aFun == "tanh"){
    afType = TANH;
  } else if (aFun == "logistic"){
    afType = LOGISTIC;
  } else {
    ROS_INFO("Invalid activation function type, using logistic.");
    afType = LOGISTIC;
  }
  
  // Logging
  epochCount = 0 ;
  seed = std::chrono::system_clock::now().time_since_epoch().count() ;
  
  // Subscriber
  subResult = nh.subscribe("/episode_result", 10, &MultiRobotPG::episodeCallback, this) ;
  
  NewEpisode() ;
}

void MultiRobotPG::episodeCallback(const std_msgs::Float64& msg){
  // Read out reward
  double r = msg.data ;
  rewardLog.push_back(r) ; 
  NewEpisode() ;
}

void MultiRobotPG::NewEpisode(){ 
  if (epochCount < nEps){ // Keep training!
    ROS_INFO_STREAM("[Policy Gradient] Epoch " << epochCount) ;  
    // Run stage simulation
    // Simulation nodes must initialise robots in stage
    // Nodes must also read in ROS params for robot control policies
    // This implementation expects the reward to be written to a rosparam
    // Simulation timer will automatically terminal the episode
    if (nRob>1){
      std::string command = "rosrun multirobot_stage run-multi-robot-explore " + std::to_string(nRob) + " " + std::to_string(headless);
      system(command.c_str()) ;
    } else {
      std::string command = "rosrun multirobot_stage run-single-robot-explore " + std::to_string(nRob) + " " + std::to_string(headless);
      system(command.c_str()) ;
    }
  }
  else{
    ROS_INFO("All learning episodes complete!") ;
  }
}