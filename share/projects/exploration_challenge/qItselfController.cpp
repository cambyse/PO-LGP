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
         "{ type=qItself ref1=" << _index_vec << " target=[" << _goal << "]}";
  return buf.str();
}

void qItselfController::setEndeff(const std::string& endeff) {
    _index_vec = ::Eigen::MatrixXd::Zero(_system->getDof(), 1);
    std::stringstream ss;
    ss << CONST_INTERFACE->getJointByName(endeff)["qIndex"];
    int qi;
    ss >> qi;
    _index_vec(qi, 1) = 1;
}

