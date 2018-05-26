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
    PolicyGrad(size_t, size_t, size_t, actFun, double, double, double, double) ; // nIn, nOut, nHidden, activation function, learning rate, beta1, beta2, epsilon
    ~PolicyGrad(){};
    
    void OutputGradient(VectorXd, size_t, VectorXd&, VectorXd&);
    void PolicyGradientStep(VectorXd, size_t, double);
    void PolicyGradientADAMStep(VectorXd, size_t, double);
    VectorXd EvaluateNNSoftmax(VectorXd);
    VectorXd EvaluateNNSoftmax(VectorXd, VectorXd &);
  private:
    VectorXd Softmax(VectorXd);
    actFun afType_;
    double learning_rate_;
    double beta1_;
    double beta2_;
    double eps_;
} ;
PolicyGrad::PolicyGrad(size_t nIn, size_t nOut, size_t nHidden, actFun afType=TANH,
double learning_rate=0.001, double beta1=0.9, double beta2=0.999, double eps=pow(10,-8)):
  NeuralNet::NeuralNet(nIn, nOut, nHidden, afType, UNBOUNDED),
  afType_(afType), learning_rate_(learning_rate), beta1_(beta1), beta2_(beta2), eps_(eps) {}

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
void PolicyGrad::PolicyGradientStep(VectorXd state, size_t action_index, double reward){
  //calculate gradient:
  VectorXd action, dydA, dydB, GA, GB, Anew, Bnew;
  action = EvaluateNNSoftmax(state);
  OutputGradient(state, action_index, dydA, dydB);
  GA = reward * 1/action(action_index)*dydA;
  GB = reward * 1/action(action_index)*dydB;
  //calculate weights
  Anew = GetWeightsA() + learning_rate_ * GA;
  Bnew = GetWeightsB() + learning_rate_ * GB;
  //set new NN weights
  SetWeights(Anew,Bnew);
}

void PolicyGrad::PolicyGradientADAMStep(VectorXd state, size_t action_index, double reward){
  //calculate gradient:
  
  //calculate weights:

  //set new NN weights:
}

void PolicyGrad::OutputGradient(VectorXd inputs, size_t output_index, VectorXd& dydA, VectorXd& dydB){
  //1. gradient of input  u = x'A:        du/dA = [x x x ...]
  //2. gradient of hidden h = sig(u):     dh/du = dsig(u)/du
  //3. gradient of output o = [h' 1]'B:   do/dh = B(0:end-1,:)
  //4. gradient of output o = [h' 1]'B:   do/dB = [[h; 1] [h; 1] [h; 1] ...]
  //5. gradient of softmax y = sm(o):     dy/do = dsm(o)/do
  //6. dy/dB = dy/do * do/dB
  //7. dy/dA = dy/do * do/dh * dh/du * du/dA

  MatrixXd A, B;
  A = this->GetWeightsA();
  B = this->GetWeightsB();

  //1. concatenate input vectors
  VectorXd x = inputs;
  MatrixXd dudA = MatrixXd::Zero(A.cols(),A.rows());
  for (size_t i = 0; i<A.cols(); i++){
    dudA.row(i) = x;
  }
  //2. derivative of actfun evaluated at u
  VectorXd u = x.transpose()*A ;
  MatrixXd dhdu(u.cols(),u.rows());
  for (size_t i = 0; i<dhdu.rows(); i++){
    for (size_t j = 0; j<dhdu.cols(); j++){
      if (afType_ == TANH){
        dhdu(i,j) = 1/pow(cosh(u(j,i)),2) ;
      } else if (afType_ == LOGISTIC){
        dhdu(i,j) = exp(u(j,i))/pow(exp(u(j,i))+1,2) ;
      }
    }
  }
  
  //3. B(0:end-1,:)
  MatrixXd dodh = B.block(0,0,B.rows()-1,B.cols()).transpose();

  //4. 
  VectorXd h(u.size());
  for (int i = 0; i < h.size(); i++){
      if (afType_ == TANH){
        h(i) = tanh(u(i)) ;
      } else if (afType_ == LOGISTIC){
        h(i) = 1/(1+exp(-u(i))) ;
      }
  }
  VectorXd hp(h.size()+1);
  hp << h, 1;
  MatrixXd dodB = MatrixXd::Zero(B.cols(),B.rows());
  for (size_t i = 0; i<B.cols(); i++){
    dodB.row(i) = hp;
  }

  //5. derivative of softmax evaluated at o
  VectorXd o = hp.transpose() * B ;
  MatrixXd dydo = MatrixXd::Zero(o.size(),o.size()) ;
  for (size_t i=0; i<o.size(); i++){
    for (size_t j=0; j<o.size(); j++){
      if (i==j){
        dydo(i,j) = o(i)*(1-o(j)) ;
      } else {
        dydo(i,j) = -o(i)*o(j) ;
      }
    }
  }

  VectorXd y = Softmax(o) ;

  //6.
  std::cout << "dydo.size(): " << dydo.rows() <<"," << dydo.cols() << std::endl;
  std::cout << "dodh.size(): " << dodh.rows() <<"," << dodh.cols() << std::endl;
  std::cout << "dhdu.size(): " << dhdu.rows() <<"," << dhdu.cols() << std::endl;
  std::cout << "dudA.size(): " << dudA.rows() <<"," << dudA.cols() << std::endl;
  dydA = dydo * dodh * dhdu * dudA ;
  dydA = dydA.transpose();
  //7.
  std::cout << "dydo.size(): " << dydo.rows() <<"," << dydo.cols() << std::endl;
  std::cout << "dodB.size(): " << dodB.rows() <<"," << dodB.cols() << std::endl;
  dydB = MatrixXd::Zero(B.rows(),B.cols());
  for (size_t i=0; i<dydo.rows(); i++){
    dydB.row(i) = dydo(output_index,i) * dodB.row(i); 
  }
  dydB = dydB.transpose();
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
