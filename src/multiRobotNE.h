#include <iostream>
#include <stdlib.h>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <Eigen/Eigen>
#include <float.h>

#include "multirobot_stage/NeuroEvo.h"
#include "multirobot_stage/NeuralNet.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>

class MultiRobotNE{
  public:
    MultiRobotNE(ros::NodeHandle) ;
    ~MultiRobotNE() ;
  private:
    int nRob ;     // number of robots
    int nIn ;      // input state vector size
    int nOut ;     // output action vector size
    int nHidden ;  // number of hidden neurons
    int nPop ;     // population size
    int nEps ;     // number of learning epochs
    actFun afType;
    
    int epochCount ;   // epoch counter (number of evolutions)
    int teamCount ;    // episode counter (number of tested policies in one evolution)
    
    unsigned seed ;
    vector< vector<int> > teams ;
    vector< vector<double> > rewards ;
    vector<NeuroEvo *> robotTeam ;
    vector<double> rewardLog ;
    double maxR ;
    int maxTeam ;
    
    ros::Subscriber subResult ;
    void episodeCallback(const std_msgs::Float64&) ;
    
    void NewEpisode() ;
};

MultiRobotNE::MultiRobotNE(ros::NodeHandle nh){
  
  ROS_INFO("Initialising multi-robot neuro-evolution...") ;
  
  // Initialise learning domain
  // Domain contains 2 NeuroEvo agents, each agent maintains a population of policies which are encoded as neural networks
  // Neural networks are initialised according to size of input and output vectors, number of nodes in the hidden layer, and the activation function
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
    NeuroEvo * NE = new NeuroEvo(nIn, nOut, nHidden, nPop, afType); 
    robotTeam.push_back(NE) ;
  }
  
  // Logging
  maxR = -DBL_MAX ;
  maxTeam = 0 ;
  epochCount = -1 ;
  teamCount = 0 ;
  seed = std::chrono::system_clock::now().time_since_epoch().count() ;
  
  // Subscriber
  subResult = nh.subscribe("/episode_result", 10, &MultiRobotNE::episodeCallback, this) ;
  
  NewEpisode() ;
}
  
MultiRobotNE::~MultiRobotNE(){
  for (int n = 0; n < nRob; n++){
    delete robotTeam[n] ;
    robotTeam[n] = 0 ;
  }
}

void MultiRobotNE::episodeCallback(const std_msgs::Float64& msg){
  // Read out reward
  double r = msg.data ;
  for (int n = 0; n < nRob; n++){
    rewards[n][teams[n][teamCount]] = r ;
  }
  if (r > maxR){
    maxR = r ;
    maxTeam = teamCount ;
  }
  
  teamCount++ ;
  teamCount = teamCount % (nPop*2) ;
  
  if (teamCount % (nPop*2) == 0){
    // Record champion team
    rewardLog.push_back(maxR) ;
    char strA[50] ;
    char strB[50] ;
    for (int n = 0; n < nRob; n++){
      sprintf(strA,"weightsA%d.txt",n) ;
      sprintf(strB,"weightsB%d.txt",n) ;
      NeuralNet * bestNN = robotTeam[n]->GetNNIndex(teams[n][maxTeam]) ;
      bestNN->OutputNN(strA,strB) ;
    }
    
    // Compete (halves population size)
    for (int n = 0; n < nRob; n++){
      robotTeam[n]->EvolvePopulation(rewards[n]) ;
    }
  }
  
  NewEpisode() ;
}

void MultiRobotNE::NewEpisode(){
  // Initialise new epoch if all policies in current round have been tested
  if (teamCount % (nPop*2) == 0){ // new epoch
    epochCount++ ;
    
    // Container for storing population indices
    vector<int> tt ;
    for (int p = 0; p < nPop*2; p++){
      tt.push_back(p) ;
    }
    teams.clear() ;
      
    // Container to store rewards
    vector<double> rr(nPop*2,0.0) ;
    rewards.clear() ;
    
    // Mutate populations (doubles population size)
    for (int n = 0; n < nRob; n++){
      robotTeam[n]->MutatePopulation() ;
      
      // Create randomised teams for this epoch
      shuffle (tt.begin(), tt.end(), std::default_random_engine(seed)) ;
      teams.push_back(tt) ;
      
      // Initialise reward container for this epoch
      rewards.push_back(rr) ;
    }
  }
  
  if (epochCount < nEps){ // Keep training!
    ROS_INFO_STREAM("[Epoch " << epochCount << "] testing team number: " << teamCount) ;
    
    // Get policies for this team
    for (int n = 0; n < nRob; n++){
      NeuralNet * curNN = robotTeam[n]->GetNNIndex(teams[n][teamCount]) ;
      
      // Reshape weight matrices to single vector
      MatrixXd A = curNN->GetWeightsA() ;
      vector<double> AA ;
      for (int ii = 0; ii < A.rows(); ii++){
        for (int jj = 0; jj < A.cols(); jj++){
          AA.push_back(A(ii,jj)) ;
        }
      }
      
      MatrixXd B = curNN->GetWeightsB() ;
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
    system("rosrun multirobot_stage run-multi-robot-explore") ;
  }
  else{
    ROS_INFO("All learning episodes complete!") ;
  }
}
