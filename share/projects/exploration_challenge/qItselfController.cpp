#include "qItselfController.h"
#include "pr2System.h"
#include <Eigen/Dense>
#include <string>
#include <sstream>


#define CONST_INTERFACE const_cast<ActionSwigInterface*>(&(boost::dynamic_pointer_cast<const pr2System>(_system)->interface))

qItselfController::qItselfController() : Controller(), _running(false) {}

void qItselfController::initialize() {
  _fact = _create_fact();
  CONST_INTERFACE->setFact(_fact.c_str());
  _running = true;
}

void qItselfController::terminate() {
  CONST_INTERFACE->stopFact("(FollowReferenceActivity qItself)");
  _running = false;
}

::Eigen::MatrixXd qItselfController::step(const double& t) {
  return ::Eigen::MatrixXd();  
}

::Eigen::MatrixXd qItselfController::relativeGoalToAbsolute(const Eigen::MatrixXd& goalRel) const {
  return _goal + goalRel;  
}

void qItselfController::setGoal(const Eigen::MatrixXd& new_goal) {
  if(_running) {
    CONST_INTERFACE->stopFact("(FollowReferenceActivity qItself)");
    //CONST_INTERFACE->stopFact(_fact.c_str());
  }

  _goal = new_goal;
  _fact = _create_fact();  

  if(_running) {
    CONST_INTERFACE->stopFact(_fact.c_str());
    //CONST_INTERFACE->setFact(_fact.c_str());
  }
}

std::string qItselfController::_create_fact() const {
  std::stringstream buf;
  buf << "(FollowReferenceActivity qItself)" <<
         "{ type=qItself ref1=" << _endeff << " target=[" << _goal(0,0) << "]}";
  return buf.str();
}

