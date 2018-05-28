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
    PolicyGrad(int, int, int, actFun, double, double, double, double) ; // nIn, nOut, nHidden, activation function, learning rate, beta1, beta2, epsilon
    ~PolicyGrad(){};
    
    void OutputGradient(VectorXd, int, MatrixXd&, MatrixXd&);
    void PolicyGradientStep(VectorXd, int, double);
    void PolicyGradientADAMStep(VectorXd, int, double);
    VectorXd EvaluateNNSoftmax(VectorXd);
    VectorXd EvaluateNNSoftmax(VectorXd, VectorXd &);
  private:
    VectorXd Softmax(VectorXd);
    actFun afType_;
    double alpha_;
    double beta1_;
    double beta2_;
    double eps_;

    // ADAM variables:
    int t_;
    ArrayXd m_A_;
    ArrayXd m_B_;
    ArrayXd m_hA_;
    ArrayXd m_hB_;
    ArrayXd v_A_;
    ArrayXd v_B_;
    ArrayXd v_hA_;
    ArrayXd v_hB_;
} ;
PolicyGrad::PolicyGrad(int nIn, int nOut, int nHidden, actFun afType=TANH,
double learning_rate=0.001, double beta1=0.9, double beta2=0.999, double eps=10e-8):
  NeuralNet::NeuralNet(nIn, nOut, nHidden, afType, UNBOUNDED),
  afType_(afType), alpha_(learning_rate), beta1_(beta1), beta2_(beta2), eps_(eps) {
  t_ = 0;
  m_A_ .resize(nIn, nHidden);
  m_B_ .resize(nHidden+1, nOut);
  m_hA_.resize(nIn, nHidden);
  m_hB_.resize(nHidden+1, nOut);
  v_A_ .resize(nIn, nHidden);
  v_B_ .resize(nHidden+1, nOut);
  v_hA_.resize(nIn, nHidden);
  v_hB_.resize(nHidden+1, nOut);
  }

// Evaluate NN output given input vector
VectorXd PolicyGrad::EvaluateNNSoftmax(VectorXd inputs){
  VectorXd outputs = this->Softmax(this->EvaluateNN(inputs));
  return outputs ;
}

// Evaluate NN output given input vector
VectorXd PolicyGrad::EvaluateNNSoftmax(VectorXd inputs, VectorXd & hiddenLayer){
  VectorXd outputs = this->Softmax(this->EvaluateNN(inputs, hiddenLayer));
  return outputs ;
}

// extend the neural network class with a member to perform a policy gradient step
void PolicyGrad::PolicyGradientStep(VectorXd state, int action_index, double reward){
  // calculate gradients dydA and dydB:
  VectorXd action = this->EvaluateNNSoftmax(state);
  MatrixXd A = this->GetWeightsA();
  MatrixXd B = this->GetWeightsB();
  std::cout << "[PolicyGrad.h]\n|A|_2: " << A.norm() << std::endl << "|B|_2: " << B.norm() << std::endl;  
  MatrixXd dydA(A.rows(),A.cols());
  MatrixXd dydB(B.rows(),B.cols());
  this->OutputGradient(state, action_index, dydA, dydB);
  
  //calculate weights
  // std::cout << "[PolicyGrad.h] action(action_index) " << action(action_index) <<std::endl;
  if (action(action_index)>0){
    MatrixXd dA  = alpha_ * reward * 1/action(action_index) * dydA;
    // std::cout << "[PolicyGrad.h]  A.size() " << A.rows() << "," << A.cols() <<std::endl;
    // std::cout << "[PolicyGrad.h] dA.size() " << dA.rows() << "," << dA.cols() <<std::endl;
    MatrixXd dB  = alpha_ * reward * 1/action(action_index) * dydB;
    // std::cout << "[PolicyGrad.h]  B.size() " << B.rows() << "," << B.cols() <<std::endl;
    // std::cout << "[PolicyGrad.h] dB.size() " << dB.rows() << "," << dB.cols() <<std::endl;
    std::cout << "[PolicyGrad.h]\n|dA|_2: " << dA.norm() <<std::endl << "|dB|_2 " << dB.norm() << std::endl;
    A += dA ;
    B += dB ;
  }
  
  //set new NN weights
  std::cout << "[PolicyGrad.h]\n|A_new|_2: " << A.norm() << std::endl << "|B_new|_2: " << B.norm() << std::endl;  
  // std::cout << "[PolicyGrad.h] Setting new weights" <<std::endl;
  this->SetWeights(A,B);
}

void PolicyGrad::PolicyGradientADAMStep(VectorXd state, int action_index, double reward){
  //calculate gradient:
  VectorXd action = EvaluateNNSoftmax(state);
  MatrixXd A = this->GetWeightsA();
  MatrixXd B = this->GetWeightsB();
  std::cout << "[PolicyGrad.h]\n|A|_2: " << A.norm() <<std::endl << "|B|_2 " << B.norm() << std::endl;
  MatrixXd dydA(A.rows(),A.cols());
  MatrixXd dydB(B.rows(),B.cols());
  this->OutputGradient(state, action_index, dydA, dydB);

  //calculate weights:  
  if (action(action_index)>0){
    MatrixXd dRdA  = alpha_ * reward * 1/action(action_index) * dydA;
    MatrixXd dRdB  = alpha_ * reward * 1/action(action_index) * dydB;
  
    t_++;
    m_A_ = beta1_ * m_A_ + (1-beta1_) * dRdA.array();
    m_B_ = beta1_ * m_B_ + (1-beta1_) * dRdB.array();

    v_A_ = beta2_ * v_A_ + (1-beta2_) * dRdA.array().square();
    v_B_ = beta2_ * v_B_ + (1-beta2_) * dRdB.array().square();

    m_hA_ = m_A_ / (1-pow(beta1_,t_));
    m_hB_ = m_B_ / (1-pow(beta1_,t_));

    v_hA_ = v_A_ / (1-pow(beta2_,t_));
    v_hB_ = v_B_ / (1-pow(beta2_,t_));

    MatrixXd dA = alpha_ * m_hA_ * (v_hA_.sqrt()+eps_).inverse();
    MatrixXd dB = alpha_ * m_hB_ * (v_hB_.sqrt()+eps_).inverse();

    std::cout << "[PolicyGrad.h]\n|dA|_2: " << dA.norm() <<std::endl << "|dB|_2 " << dB.norm() << std::endl; 
    A += dA;  
    B += dB;  
  }
  
  //set new NN weights:
  std::cout << "[PolicyGrad.h]\n|A_new|_2: " << A.norm() << std::endl << "|B_new|_2: " << B.norm() << std::endl;
  this->SetWeights(A,B);
}

void PolicyGrad::OutputGradient(VectorXd inputs, int oi, MatrixXd& dydA_oi, MatrixXd& dydB_oi){
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
  std::cout << "[PolicyGrad.h]" << std::endl;

  //1. concatenate input vectors
  VectorXd x = inputs;
  MatrixXd dudA = MatrixXd::Zero(A.cols(),A.rows());
  std::cout << "dudA.size(): " << dudA.rows() << "," << dudA.cols() << std::endl;
  for (size_t i = 0; i<A.cols(); i++){
    dudA.row(i) = x;
  }
  //2. derivative of actfun evaluated at u
  VectorXd u = x.transpose()*A ;
  VectorXd dhdu(u.rows());
  std::cout << "dhdu.size(): " << dhdu.rows() <<"," << dhdu.cols() << std::endl;
  for (size_t i = 0; i<u.rows(); i++){
    //for (size_t j = 0; j<u.rows(); j++){
    if (afType_ == TANH){
      dhdu(i) = 1/pow(cosh(u(i)),2) ;
    } else if (afType_ == LOGISTIC){
      dhdu(i) = exp(u(i))/pow(exp(u(i))+1,2) ;
    }
    //}
  }
  //2.1 dh/dA
  MatrixXd dhdA(dhdu.rows(),dudA.cols());
  for (size_t i=0; i<dhdu.rows(); i++){
    //for (size_t j=0; j<dudA.rows(); j++){
    dhdA.row(i) = dhdu(i) * dudA.row(i); // dhdu(i) * x; 
    //}
  }
  
  //3. B(0:end-1,:)
  MatrixXd dodh = B.block(0,0,B.rows()-1,B.cols()).transpose();
  std::cout << "dodh.size(): " << dodh.rows() <<"," << dodh.cols() << std::endl;

  //4. do/dB
  VectorXd h(u.rows());
  for (int i = 0; i < h.rows(); i++){
    if (afType_ == TANH){
      h(i) = tanh(u(i)) ;
    } else if (afType_ == LOGISTIC){
      h(i) = 1/(1+exp(-u(i))) ;
    }
  }
  VectorXd hp(h.rows()+1);
  hp << h, 1;
  MatrixXd dodB = MatrixXd::Zero(B.cols(),B.rows());
  std::cout << "dodB.size(): " << dodB.rows() <<"," << dodB.cols() << std::endl;
  for (size_t i = 0; i<B.cols(); i++){
    dodB.row(i) = hp;
  }

  //5. derivative of softmax evaluated at o
  VectorXd o = hp.transpose() * B ;
  MatrixXd dydo = MatrixXd::Zero(o.rows(),o.rows()) ;
  std::cout << "dydo.size(): " << dydo.rows() <<"," << dydo.cols() << std::endl;
  for (size_t i=0; i<o.rows(); i++){
    for (size_t j=0; j<o.rows(); j++){
      if (i==j){
        dydo(i,j) = o(i)*(1-o(j)) ;
      } else {
        dydo(i,j) = -o(i)*o(j) ;
      }
    }
  }

  //6. dy(oi)/dA
  dydA_oi.resize(A.rows(),A.cols());
  MatrixXd dydAp = MatrixXd::Zero(A.cols(),A.rows());
  for (size_t i=0; i<dodh.rows(); i++){
    MatrixXd dodA = MatrixXd::Zero(A.cols(),A.rows());
    for (size_t j=0; j<dhdA.rows(); j++){
      dodA.row(j) += dodh(i,j) * dhdA.row(j); // dodh(i,j) * dhdu(j) * x; // TODO verify this!
    }
    dydAp += dydo(oi,i) * dodA; // TODO verify this!
  }
  dydA_oi = dydAp.transpose();
  std::cout << "dydA_oi.size(): " << dydA_oi.rows() <<"," << dydA_oi.cols() << std::endl;

  //7. dy(oi)/dB
  dydB_oi.resize(B.rows(),B.cols());
  MatrixXd dydBp = MatrixXd::Zero(B.cols(),B.rows());
  for (size_t j=0; j<dodB.rows(); j++){
    dydBp.row(j) += dydo(oi,j) * dodB.row(j); 
  }
  dydB_oi = dydBp.transpose();
  std::cout << "dydB_oi.size(): " << dydB_oi.rows() <<"," << dydB_oi.cols() << std::endl;
}

VectorXd PolicyGrad::Softmax(VectorXd inputs){
  VectorXd outputs(inputs.rows());
  double Z = 0;
  for (size_t i = 0; i < inputs.rows(); i++){
      Z += exp(inputs(i));
  }
  for (size_t i = 0; i < inputs.rows(); i++){
      outputs(i) = exp(inputs(i))/Z;
  }
  return outputs;
}

#endif // POLICY_GRAD_H_
