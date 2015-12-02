#include "pr2ControlSet.h"
#include <Eigen/Dense>

pr2ControlSet::pr2ControlSet() {
  this->setType("pr2ControlSet");
  this->setName("pr2_control_set");
   
} 

pr2ControlSet::pr2ControlSet(const pr2ControlSet& cs) : ControlSet(cs) {};

pr2ControlSet::~pr2ControlSet() {
  this->terminate();  
}

void pr2ControlSet::initialize() {
  for(auto controller : _controllers) {
    controller.second->initialize();  
  }
   
} 

void pr2ControlSet::terminate() {
  for(auto controller : _controllers) {
    controller.second->terminate();  
  }
}

::Eigen::MatrixXd pr2ControlSet::step(const double& t) {
  return ::Eigen::MatrixXd();
}
