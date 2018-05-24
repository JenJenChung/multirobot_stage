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
    PolicyGrad(size_t, size_t, size_t, actFun afType=TANH) ; // nIn, nOut, nHidden, activation function
    ~PolicyGrad(){};
    
    void PolicyGradientStep(double);
} ;

PolicyGrad::PolicyGrad(size_t nIn, size_t nOut, size_t nHidden, actFun afType):
  NeuralNet::NeuralNet(nIn, nOut, nHidden, afType){}

// extend the neural network class with a member to perform a policy gradient step
void PolicyGrad::PolicyGradientStep(double reward){
  //calculate gradient
  //calculate weights
  //set new NN weights
}
#endif // POLICY_GRAD_H_
