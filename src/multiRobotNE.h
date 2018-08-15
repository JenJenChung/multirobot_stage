#include <iostream>
#include <stdlib.h>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <Eigen/Eigen>
#include <float.h>
#include <string>
#include <fstream>
#include <sstream>

#include "multirobot_stage/NeuroEvo.h"
#include "multirobot_stage/NeuralNet.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>

using std::vector ;
using std::string ;

class MultiRobotNE{
  public:
    MultiRobotNE(ros::NodeHandle) ;
    ~MultiRobotNE() ;
  private:
    bool headless;
    int nRob ;        // number of robots
    int code_size ;   // encoding size
    int nPop ;        // population size
    int nEps ;        // number of learning epochs
    int nIn ;         // input state vector size
    int nOut ;        // output action vector size
    int nHidden ;     // number of hidden neurons
    string root_ns ;  // root namespace, default "robot"
    string r_folder;  // results folder directory
    string r_file ;   // file to write team evaluation results to
    string nn_file ;  // file to write champion NN weights to
    
    int epochCount ;  // epoch counter (number of evolutions)
    int teamCount ;   // episode counter (number of tested policies in one evolution)
    
    unsigned seed ;
    vector< vector<int> > teams ;
    vector< vector<double> > rewards ;
    vector<NeuroEvo *> robotTeam ;
    vector<double> rewardLog ;
    double maxR ;
    int maxTeam ;
    char strR[100] ;
    
    std::ofstream evalFile ;
    
    ros::Subscriber subResult ;
    void episodeCallback(const std_msgs::Float64&) ;
    
    void NewEpisode() ;
};

MultiRobotNE::MultiRobotNE(ros::NodeHandle nh){
  
  ROS_INFO("Initialising multi-robot neuro-evolution...") ;
  
  // Initialise learning domain
  // Domain contains 2 NeuroEvo agents, each agent maintains a population of policies which are encoded as neural networks
  // Neural networks are initialised according to size of input and output vectors, number of nodes in the hidden layer, and the activation function
  ros::param::get("headless", headless);
  ros::param::get("num_agents", nRob);
  ros::param::get("encoding_size", code_size) ;
  ros::param::get("nPop", nPop);
  ros::param::get("nEps", nEps);
  ros::param::get("root_namespace", root_ns) ;
  ros::param::get("results_folder", r_folder) ;
  ros::param::get("results_file", r_file) ;
  ros::param::get("nn_file", nn_file) ;
  
  nIn = code_size + 2*(nRob-1) ;
  nOut = 2 ; // [true bearing, distance]
  nHidden = nIn * 2 ;
  
  for (int n = 0; n < nRob; n++){
    NeuroEvo * NE = new NeuroEvo(nIn, nOut, nHidden, nPop, TANH); 
    robotTeam.push_back(NE) ;
  }
  
  // Logging
  maxR = -DBL_MAX ;
  maxTeam = 0 ;
  epochCount = -1 ;
  teamCount = 0 ;
  seed = std::chrono::system_clock::now().time_since_epoch().count() ;
  
  if (evalFile.is_open())
    evalFile.close() ;
  sprintf(strR,"%s%s", r_folder.c_str(), r_file.c_str()) ;
  evalFile.open(strR, std::ios::app) ;
  
  std::cout << "Writing evaluation outputs to file: " << strR << "\n" ;
  
  // Subscriber
  subResult = nh.subscribe("/episode_result", 10, &MultiRobotNE::episodeCallback, this) ;
  
  NewEpisode() ;
}
  
MultiRobotNE::~MultiRobotNE(){
  for (int n = 0; n < nRob; n++){
    delete robotTeam[n] ;
    robotTeam[n] = 0 ;
  }
  if (evalFile.is_open())
    evalFile.close() ;
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
  
  ROS_INFO_STREAM("Episode reward for team " << teamCount << ": " << r) ;
  
  teamCount++ ;
  teamCount = teamCount % (nPop*2) ;
  ROS_INFO_STREAM("Team count: " << teamCount);
  
  if (teamCount % (nPop*2) == 0){
    // Record champion team for this epoch
    rewardLog.push_back(maxR) ;
    
    vector<int> championIDs ;
    for (int n = 0; n < nRob; n++){
      championIDs.push_back(teams[n][maxTeam]) ;
    }
    
    ROS_INFO("Neural Evolution is writing to %s with maxR = %f", strR, maxR);
    // Write to file
    evalFile << maxR << "," ;
    for (size_t i = 0; i < championIDs.size(); i++){
      evalFile << championIDs[i] << "," ;
    }
    evalFile << "\n" ;
    
    if (epochCount == nEps-1){
      char strA[100] ;
      char strB[100] ;
      for (int n = 0; n < nRob; n++){
        sprintf(strA,"%s%s_%d_A_%s",r_folder.c_str(),root_ns.c_str(),n,nn_file.c_str()) ;
        sprintf(strB,"%s%s_%d_B_%s",r_folder.c_str(),root_ns.c_str(),n,nn_file.c_str()) ;
        NeuralNet * bestNN = robotTeam[n]->GetNNIndex(teams[n][maxTeam]) ;
        bestNN->OutputNN(strA, strB) ;
      }
    }
    
    // Compete (halves population size)
    for (int n = 0; n < nRob; n++){
      robotTeam[n]->EvolvePopulation(rewards[n]) ;
    }
    
    // Reset maxR and maxTeam
    maxR = -DBL_MAX ;
    maxTeam = 0 ;
  }
  
  ROS_INFO("Episode callback end, starting new episode");
  NewEpisode() ;
}

void MultiRobotNE::NewEpisode(){
  // Initialise new epoch if all policies in current round have been tested
  if (teamCount % (nPop*2) == 0){ // new epoch
    epochCount++ ;
    ROS_INFO("Epoch count: %d", epochCount);
    
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
      sprintf(buffer,"/%s_%d/A", root_ns.c_str(), n) ;
      ros::param::set(buffer,AA) ;
      sprintf(buffer,"/%s_%d/B", root_ns.c_str(), n) ;
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
