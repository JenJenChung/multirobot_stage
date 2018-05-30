#ifndef NEUR0_EVO_H_
#define NEUR0_EVO_H_

#include <chrono>
#include <algorithm>
#include <random>
#include <vector>
#include <Eigen/Eigen>
#include <math.h>
#include <iostream>
#include "NeuralNet.h"

using std::vector ;
using std::sort ;
using namespace Eigen ;

class NeuroEvo{
public:
  NeuroEvo(size_t, size_t, size_t, size_t, actFun afType = TANH); // nIn, nOut, nHidden, popSize, activation function
  NeuroEvo(size_t nIn, size_t nOut, size_t nHidden, size_t pSize,
           std::vector<std::pair<MatrixXd, MatrixXd>> weights_and_biases, actFun afType = TANH);
  ~NeuroEvo();

  void MutatePopulation();
  void EvolvePopulation(vector<double>);
  vector<double> GetAllEvaluations();

  NeuralNet *GetNNIndex(size_t i) { return populationNN[i]; }
  size_t GetCurrentPopSize() { return populationNN.size(); }

  void SetMutationNormLog(bool b = true) { computeMutationNorms = b; }
  vector<double> GetMutationNorm() { return mutationFrobeniusNorm; }

private:
  size_t numIn;
  size_t numOut;
  size_t numHidden;
  actFun activationFunction;

  size_t populationSize;
  vector<NeuralNet *> populationNN;

  void (NeuroEvo::*SurvivalFunction)();
  void BinaryTournament();
  void RetainBestHalf();
  static bool CompareEvaluations(NeuralNet *, NeuralNet *);

  bool computeMutationNorms;
  vector<double> mutationFrobeniusNorm;
  double ComputeFrobeniusNorm(MatrixXd, MatrixXd, MatrixXd, MatrixXd);
};

// Constructor: Initialises all NN in population, given NN layer sizes and population size, also sets SurvivalFunction
NeuroEvo::NeuroEvo(size_t nIn, size_t nOut, size_t nHidden, size_t pSize, actFun afType)
    : numIn(nIn), numOut(nOut), numHidden(nHidden), activationFunction(afType), populationSize(pSize) {
  for (size_t i = 0; i < populationSize; i++) {
    ROS_INFO("Normal NeuroEvo constructor\n");
    populationNN.push_back(new NeuralNet(numIn, numOut, numHidden, afType));
  }
  SurvivalFunction = &NeuroEvo::BinaryTournament; // how to decide which NNs to retain after each round of evolution
}

// Overloaded constructor for cases where we want to load the weights from an existing file.
// Need to pass one weight and one bias file per member of the population
NeuroEvo::NeuroEvo(size_t nIn, size_t nOut, size_t nHidden, size_t pSize,
                   std::vector<std::pair<MatrixXd, MatrixXd>> weights_and_biases, actFun afType)
    : numIn(nIn), numOut(nOut), numHidden(nHidden), activationFunction(afType), populationSize(pSize) {

  ROS_INFO("Loading existing weights from file...\n");
  for (size_t i = 0; i < populationSize; i++) {
    // call NeuralNet constructor that takes weight matrixes as arguments
    populationNN.push_back(
        new NeuralNet(numIn, numOut, numHidden, weights_and_biases[0].first, weights_and_biases[0].second, afType)); //TODO: change index from 0 to i, make sure vector contains enough elements
  }
  SurvivalFunction = &NeuroEvo::BinaryTournament; // how to decide which NNs to retain after each round of evolution
}

// Destructor: Deletes all NN objects from population
NeuroEvo::~NeuroEvo(){
  for (size_t i = 0; i < populationNN.size(); i++){
    delete(populationNN[i]) ;
    populationNN[i] = 0 ;
  }
}

// Double population size by adding NN with mutated weights of existing NN 
void NeuroEvo::MutatePopulation(){
  if (computeMutationNorms){
    mutationFrobeniusNorm.clear() ;
  }
  
  for (size_t i = 0; i < populationSize; i++){
    size_t j = i + populationSize ;
    populationNN.push_back(new NeuralNet(numIn, numOut, numHidden, activationFunction)) ;
    populationNN[j]->SetWeights(populationNN[i]->GetWeightsA(),populationNN[i]->GetWeightsB()) ;
    populationNN[j]->MutateWeights() ;
    
    if (computeMutationNorms){
      mutationFrobeniusNorm.push_back(ComputeFrobeniusNorm(populationNN[i]->GetWeightsA(),populationNN[i]->GetWeightsB(),populationNN[j]->GetWeightsA(),populationNN[j]->GetWeightsB())) ;
    }
  }
}

// Evolve population according to evaluation signal and survival function
void NeuroEvo::EvolvePopulation(vector<double> evaluation){
  for (size_t i = 0; i < 2*populationSize; i++)
    populationNN[i]->SetEvaluation(evaluation[i]) ;
  
  // Shuffle in preparation for comparisons
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count() ;
  shuffle(populationNN.begin(), populationNN.end(), std::default_random_engine(seed)) ;
  
  (this->*SurvivalFunction)() ;
}

// Binary tournament for survival, head to head competition between random pairs of NNs
void NeuroEvo::BinaryTournament(){
  vector<size_t> toErase ;
  for (size_t i = 0; i < populationSize; i++){
    size_t j = i + populationSize ;
    if (populationNN[i]->GetEvaluation() >= populationNN[j]->GetEvaluation()){
      delete(populationNN[j]) ;
      populationNN[j] = 0 ;
      toErase.push_back(j) ;
    }
    else {
      delete(populationNN[i]) ;
      populationNN[i] = 0 ;
      toErase.push_back(i) ;
    }
  }
  std::sort(toErase.begin(),toErase.end()) ;
  for (size_t i = 0; i < toErase.size(); i++){
    size_t j = toErase.size()-1-i ;
    populationNN.erase(populationNN.begin()+toErase[j]) ;
  }
}

// Comparitor function to sort NNs according to evaluation signal (must have strict weak ordering)
bool NeuroEvo::CompareEvaluations(NeuralNet* NN0, NeuralNet* NN1){
  return (NN0->GetEvaluation() > NN1->GetEvaluation()) ;
}

// Retain the best half of the population
void NeuroEvo::RetainBestHalf(){
  std::sort(populationNN.begin(),populationNN.end(),CompareEvaluations) ;
  
  for (size_t i = populationSize; i < 2*populationSize; i++){
    delete(populationNN[i]) ;
    populationNN[i] = 0 ;
  }
  
  // Pop from the back
  for (size_t i = 0; i < populationSize; i++)
    populationNN.pop_back() ;
}

// Return evaluations of all current NNs in population (used for debugging)
vector<double> NeuroEvo::GetAllEvaluations(){
  vector<double> evals ;
  for (size_t i = 0 ; i < populationNN.size(); i++)
    evals.push_back(populationNN[i]->GetEvaluation()) ;
  return evals ;
}

double NeuroEvo::ComputeFrobeniusNorm(MatrixXd A, MatrixXd B, MatrixXd Am, MatrixXd Bm){
  MatrixXd diffA = A-Am ;
  MatrixXd diffB = B-Bm ;
  MatrixXd diffAT = diffA.transpose() ; // congugate transpose of real matrix = transpose of matrix
  MatrixXd diffBT = diffB.transpose() ;
  MatrixXd diffATA = diffAT*diffA ;
  MatrixXd diffBTB = diffBT*diffB ;
  double traceA = diffATA.trace() ;
  double traceB = diffBTB.trace() ;
  return sqrt(traceA) + sqrt(traceB) ;
}

#endif // NEUR0_EVO_H_
