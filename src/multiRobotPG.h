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
    ~MultiRobotPG() ;
  private:
    bool headless;
    int nRob ;     // number of robots
    int nIn ;      // input state vector size
    int nOut ;     // output action vector size
    int nHidden ;  // number of hidden neurons
    int nPop ;     // population size
    int nEps ;     // number of learning epochs
    actFun afType;
    
    int epochCount ;   // epoch counter (number of evolutions)
    // int teamCount ;    // episode counter (number of tested policies in one evolution)
    
    unsigned seed ;
    vector<int> teams ; //vector< vector<int> > teams ;
    vector<double> rewards ; //vector< vector<double> > rewards ;
    vector<PolicyGrad *> robotTeam ;
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
  ros::param::get("/learning/nPop", nPop);
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
  
  for (int n = 0; n < nRob; n++){
    PolicyGrad *PG = new PolicyGrad(nIn, nOut, nHidden, afType); 
    robotTeam.push_back(PG) ;
  }
  
  // Logging
  // maxTeam = 0 ;
  epochCount = -1 ;
  seed = std::chrono::system_clock::now().time_since_epoch().count() ;
  
  // Subscriber
  subResult = nh.subscribe("/episode_result", 10, &MultiRobotPG::episodeCallback, this) ;
  
  NewEpisode() ;
}
  
MultiRobotPG::~MultiRobotPG(){
  for (int n = 0; n < nRob; n++){
    delete robotTeam[n] ;
    robotTeam[n] = 0 ;
  }
}

void MultiRobotPG::episodeCallback(const std_msgs::Float64& msg){
  // Read out reward
  double r = msg.data ;
  for (int n = 0; n < nRob; n++){
    rewards[n] = r ;
  }

  rewardLog.push_back(r) ;
  char strA[50] ;
  char strB[50] ;
  for (int n = 0; n < nRob; n++){
    sprintf(strA,"weightsA%d.txt",n) ;
    sprintf(strB,"weightsB%d.txt",n) ;
    robotTeam[n]->OutputNN(strA,strB) ;
  }
  
  // Compete (halves population size)
  //for (int n = 0; n < nRob; n++){
  //  robotTeam[n]->EvolvePopulation(rewards[n]) ;
  //}
  //}

  //TODO; implement policy gradient HERE:
  for (int n = 0; n < nRob; n++){
    //robotTeam[n]->PolicyGradientStep(rewards[n]);
  }
  
  NewEpisode() ;
}

void MultiRobotPG::NewEpisode(){ 
  if (epochCount < nEps){ // Keep training!
    ROS_INFO_STREAM("[Policy Gradient] Epoch " << epochCount) ;
    
    // Get policies for this team
    for (int n = 0; n < nRob; n++){
      // Reshape weight matrices to single vector
      MatrixXd A = robotTeam[n]->GetWeightsA() ;
      vector<double> AA ;
      for (int ii = 0; ii < A.rows(); ii++){
        for (int jj = 0; jj < A.cols(); jj++){
          AA.push_back(A(ii,jj)) ;
        }
      }
      
      MatrixXd B = robotTeam[n]->GetWeightsB() ;
      vector<double> BB ;
      for (int ii = 0; ii < B.rows(); ii++){
        for (int jj = 0; jj < B.cols(); jj++){
          BB.push_back(B(ii,jj)) ;
        }
      }
      
      // Write NN policies to rosparams
      char buffer[50] ;
      sprintf(buffer,"/robot_%d/A",n) ;
      ros::param::set(buffer,AA) ;
      sprintf(buffer,"/robot_%d/B",n) ;
      ros::param::set(buffer,BB) ;
    }
    
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