#include "OpSpaceController.h"
#include "pr2System.h"
#include "util.h"
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
  return _system->getFramePose(_endeff) + goalRel;
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
  ::Eigen::MatrixXd goal = _goal;
  if(this->getGoalIsRelative())
      goal = relativeGoalToAbsolute(_goal);

  ors::Transformation trans;
  trans.setAffineMatrix(eigen2mt(goal).p);

  std::stringstream buf1;
  std::stringstream buf2;

  buf1 << "(Control pos)";
  buf2 << "(Control quat)";

  buf1 << "{ ref1=" << _endeff << " target=[" << trans.pos.x << " " << trans.pos.y << " " << trans.pos.z << " ] }";
  _pos_fact = buf1.str();

  buf2 <<  "{ ref1=" << _endeff << " target=[ " << trans.rot.w << " " << trans.rot.x << " " << trans.rot.y << " " << trans.rot.z << " ] }";
  _rot_fact = buf2.str();

}

