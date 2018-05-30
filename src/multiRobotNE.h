#include <iostream>
#include <stdlib.h>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <float.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

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
    bool headless;
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

    boost::filesystem::path log_dir;
    std::string rewards_file_name = "rewards.csv";

    ros::Subscriber subResult;
    void episodeCallback(const std_msgs::Float64 &);

    void NewEpisode();
    void writeRewardsToFile(double reward);
    void readFile(std::string file_path, std::vector<double> &file);
    bool loadWeights(std::string weights_dir, int index,
                     std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> &loaded_weights);
};

MultiRobotNE::MultiRobotNE(ros::NodeHandle nh) {

  ROS_INFO("Initialising multi-robot neuro-evolution...") ;
  
  // Initialise learning domain
  // Domain contains 2 NeuroEvo agents, each agent maintains a population of policies which are encoded as neural networks
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
  
  std::string weights_dir = "";
  ros::param::get("/learning/pretrained_weights_dir", weights_dir);

  for (int n = 0; n < nRob; n++){
    // if pretrained_weights_dir rosparam is set, load those weights and continue training from there
    if (weights_dir != "") {
      // reuse pre-trained weights
      ROS_INFO("Reusing pre-trained weights\n");
      std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> pretrained_weights;
      if (loadWeights(weights_dir, n, pretrained_weights)) {
        ROS_INFO("loading pretrained weights succeeded...\n");
        NeuroEvo *NE = new NeuroEvo(nIn, nOut, nHidden, nPop, pretrained_weights, afType);
        robotTeam.push_back(NE);
      } else {
        ROS_WARN("Error loading weights, starting with fresh weights...\n");
        NeuroEvo *NE = new NeuroEvo(nIn, nOut, nHidden, nPop, afType);
        robotTeam.push_back(NE);
      }
    } else {
      // start with fresh weights
      ROS_INFO("Starting with some fresh weights\n");
      NeuroEvo *NE = new NeuroEvo(nIn, nOut, nHidden, nPop, afType);
      robotTeam.push_back(NE);
    }
  }

  ROS_INFO("Finished setup\n");
  
  // Logging
  maxR = -DBL_MAX ;
  maxTeam = 0 ;
  epochCount = -1 ;
  teamCount = 0 ;
  seed = std::chrono::system_clock::now().time_since_epoch().count() ;

  // Subscriber
  subResult = nh.subscribe("/episode_result", 10, &MultiRobotNE::episodeCallback, this);

  // initialise logging to file
  // auto cur_time = std::chrono::system_clock::now();
  // auto lt = std::localtime(cur_time);
  auto ct = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream cur_time_and_date;
  cur_time_and_date << std::put_time(std::localtime(&ct), "%F %T");
  log_dir = boost::filesystem::path("logs" + cur_time_and_date.str());
  boost::filesystem::create_directory(log_dir);
  ros::param::set("/learning/log_dir", log_dir.string());

  std::ofstream rewards_file;
  rewards_file.open(log_dir.string() + "/" + rewards_file_name, std::ios_base::app);
  rewards_file << "episode_reward,max_reward,epoch_num,episode_num" << std::endl;

  NewEpisode() ;
}
  
MultiRobotNE::~MultiRobotNE(){
  for (int n = 0; n < nRob; n++){
    delete robotTeam[n] ;
    robotTeam[n] = 0 ;
  }
}

void MultiRobotNE::readFile(std::string file_path, std::vector<double>& file) {
  std::ifstream file_stream;
  file_stream.open(file_path, std::ios_base::in);
  // std::string wA = wA_file
  // Eigen::MatrixXd
  std::string cell;

  // read entire file into a vector and reshape it into the size we need.
  while (std::getline(file_stream, cell, ',')) {
    if (file_stream && !cell.empty() && (cell != "\n")) {
      // printf("<%s>", cell.c_str());
      // file.push_back(boost::lexical_cast<float>(cell));
      file.push_back(std::stof(cell.c_str()));
    } else {
      printf("Finished reading weights file\n");
    }
  }
  // // This checks for a trailing comma with no data after it.
  // if (!file_stream && cell.empty())
  // {
  //     // If there was a trailing comma then add an empty element.
  //     file.push_back("");
  // }
  printf("\n");
  // ROS_INFO("Len of file vector: %d", file.size());
}

bool MultiRobotNE::loadWeights(std::string weights_dir, int index,
                               std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> &loaded_weights) {
  // TODO: currently NE only stores the best weights after each epoch. The way things are setup now is that for every
  // robot, each population starts with the same weights. Consider another approach.
  ROS_INFO("loadWeights()\n");
  std::string wA_path = weights_dir + "weightsA" + std::to_string(index) + ".txt";
  std::string wB_path = weights_dir + "weightsB" + std::to_string(index) + ".txt";
  std::vector<double> wA, wB;
  readFile(wA_path, wA);
  readFile(wB_path, wB);
  ROS_INFO("Len of wA vector: %d\n", wA.size());
  ROS_INFO("Len of wB vector: %d\n", wB.size());

  Eigen::VectorXd vec_wA = Eigen::VectorXd::Map(wA.data(), wA.size());
  Eigen::VectorXd vec_wB = Eigen::VectorXd::Map(wB.data(), wB.size());
  ROS_INFO("Len of vec_wA vector: %d\n", vec_wA.size());

  // reshape vec_wA to matrix of size numIn x numHidden and vec_wB to numHidden + 1 x numOut
  // Eigen::MatrixXd mat_wA(vec_wA.matrix().data(), nIn, nHidden);
  // Eigen::MatrixXd mat_wB(vec_wB.matrix().data(), nHidden + 1, nOut);
  Eigen::Map<Eigen::MatrixXd> mat_wA(&wA.data()[0], nIn, nHidden);
  Eigen::Map<Eigen::MatrixXd> mat_wB(&wB.data()[0], nHidden + 1, nOut);
  
  ROS_INFO("Size of mat_wA matrix: (%d x %d)\n", mat_wA.rows(), mat_wA.cols());
  ROS_INFO("Size of mat_wB matrix: (%d x %d)\n", mat_wB.rows(), mat_wB.cols());

  loaded_weights.push_back(std::make_pair(mat_wA, mat_wB));  //TODO: need eigen aligned_allocator?

  return true;
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

  // write rewards to file
  writeRewardsToFile(r);
  
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

void MultiRobotNE::writeRewardsToFile(double reward) {
  std::ofstream rewards_file;
  rewards_file.open(log_dir.string() + "/" + rewards_file_name, std::ios_base::app);
  rewards_file << std::to_string(reward) << "," << std::to_string(maxR) << "," << 
        std::to_string(epochCount) << "," << std::to_string(teamCount) << std::endl;
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

    // store current episode and epoch number
    ros::param::set("/learning/curEpisode", std::to_string(teamCount));
    ros::param::set("/learning/curEpoch", std::to_string(epochCount));
    
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