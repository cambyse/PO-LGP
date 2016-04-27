#include "OpSpaceController.h"
#include "pr2System.h"
#include <Eigen/Dense>
#include <string>
#include <sstream>


#define CONST_INTERFACE const_cast<ActionSwigInterface*>(&(boost::dynamic_pointer_cast<const pr2System>(_system)->interface))

OpSpaceController::OpSpaceController() : Controller(), _pos_fact("-"), _rot_fact("-"), _running(false) {}

void OpSpaceController::initialize() {
  _create_facts();
  CONST_INTERFACE->setFact(_pos_fact.c_str());
  CONST_INTERFACE->setFact(_rot_fact.c_str());
  _running = true;
}

void OpSpaceController::terminate() {
  CONST_INTERFACE->stopFact("(Control pos)");
  CONST_INTERFACE->stopFact("(Control quat)");
  _running = false;
}

::Eigen::MatrixXd OpSpaceController::step(const double& t) {
    return ::Eigen::MatrixXd();
}

::Eigen::MatrixXd OpSpaceController::relativeGoalToAbsolute(const Eigen::MatrixXd& goalRel) const {
  return _goal + goalRel;  
}

void OpSpaceController::setGoal(const Eigen::MatrixXd& new_goal) {
  if(_running) {
    CONST_INTERFACE->stopFact("(Control pos)");
    CONST_INTERFACE->stopFact("(Control quat)");
  }

  _goal = new_goal;
  _create_facts();
  //cout << _goal << endl;

  if(_running) {
      CONST_INTERFACE->stopFact(_pos_fact.c_str());
      CONST_INTERFACE->stopFact(_rot_fact.c_str());
  }
}

void OpSpaceController::_create_facts() {
  ors::Transformation trans;
  trans.setAffineMatrix(_goal.data());


  std::stringstream buf1;
  buf1 << "(Control pos)" <<
         "{ ref1=" << _endeff << " target=[" << _goal.block<3, 1>(0, 3) << "]}";
  _pos_fact = buf1.str();

  std::stringstream buf2;
  buf2 << "(Control quat)" <<
         "{ ref1=" << _endeff << " target=[" << ARRAY(trans.rot) << "]}";
  _rot_fact = buf2.str();
}

