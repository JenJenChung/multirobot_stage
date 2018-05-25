#ifndef POLICY_GRAD_H_
#define POLICY_GRAD_H_

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

class PolicyGrad : public NeuralNet {
  public:
    PolicyGrad(size_t, size_t, size_t, actFun, double) ; // nIn, nOut, nHidden, activation function, learning rate
    ~PolicyGrad(){};
    
    VectorXd getGradient(){};
    void PolicyGradientStep(double);
    void PolicyGradientADAMStep(double);
    VectorXd EvaluateNNSoftmax(VectorXd);
    VectorXd EvaluateNNSoftmax(VectorXd, VectorXd &);
  private:
    VectorXd Softmax(VectorXd);
    double learning_rate_;
    // TODO: insert other learning variables here
} ;
PolicyGrad::PolicyGrad(size_t nIn, size_t nOut, size_t nHidden, actFun afType=TANH, double learning_rate=0.001):
  NeuralNet::NeuralNet(nIn, nOut, nHidden, afType, UNBOUNDED),
  learning_rate_(learning_rate) {}

// Evaluate NN output given input vector
VectorXd PolicyGrad::EvaluateNNSoftmax(VectorXd inputs){
  VectorXd outputs = Softmax(NeuralNet::EvaluateNN(inputs));
  return outputs ;
}

// Evaluate NN output given input vector
VectorXd PolicyGrad::EvaluateNNSoftmax(VectorXd inputs, VectorXd & hiddenLayer){
  VectorXd outputs = Softmax(NeuralNet::EvaluateNN(inputs, hiddenLayer));
  return outputs ;
}

// extend the neural network class with a member to perform a policy gradient step
void PolicyGrad::PolicyGradientStep(double reward){
  //calculate gradient
  //calculate weights
  //set new NN weights
}

void PolicyGrad::PolicyGradientADAMStep(double reward){
  //calculate gradient
  //calculate weights
  //set new NN weights
}

VectorXd PolicyGrad::Softmax(VectorXd inputs){
  VectorXd outputs(inputs.size());
  double Z = 0;
  for (int i = 0; i < inputs.size(); i++){
      Z = Z + exp(inputs(i));
  }
  for (int i = 0; i < inputs.size(); i++){
      outputs(i) = exp(inputs(i))/Z;
  }
  return outputs;
}

#endif // POLICY_GRAD_H_
