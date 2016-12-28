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
  if (not this->_only_displacement)
    CONST_INTERFACE->setFact(_rot_fact.c_str());
  _running = true;
}

void OpSpaceController::terminate() {
  CONST_INTERFACE->stopFact("(Control pos)");
  CONST_INTERFACE->stopFact("(Control quat)");
  CONST_INTERFACE->stopFact("(Control proxy)");
  _running = false;
}

::Eigen::MatrixXd OpSpaceController::step(const double& t) {
    return ::Eigen::MatrixXd();
}

::Eigen::MatrixXd OpSpaceController::relativeGoalToAbsolute(const Eigen::MatrixXd& goalRel) const {
    Eigen::MatrixXd frame_pose = _system->getFramePose(_endeff);
    ::Eigen::MatrixXd _goal(4,4);
  if(this->_only_displacement) {
      _goal << 1, 0, 0, goalRel(0),
               0, 1, 0, goalRel(1),
               0, 0, 1, goalRel(2),
               0, 0, 0, 1;
  }
  else _goal = goalRel;
  return frame_pose * _goal;
}

void OpSpaceController::setGoal(const Eigen::MatrixXd& new_goal) {
  if(_running) {
    CONST_INTERFACE->stopFact("(Control pos)");
    CONST_INTERFACE->stopFact("(Control quat)");
    CONST_INTERFACE->stopFact("(Control proxy)");
  }

  _goal = new_goal;

  if(this->_only_displacement) {
      _goal = _goal.col(3);
      _goal.conservativeResize(3,1);
  }

  _create_facts();
  //cout << _goal << endl;

  if(_running) {
      CONST_INTERFACE->stopFact(_pos_fact.c_str());
      CONST_INTERFACE->stopFact(_rot_fact.c_str());
  }
}

void OpSpaceController::_create_facts() {
  ::Eigen::MatrixXd goal = _goal;
  if(this->getGoalIsRelative()) {
      goal = relativeGoalToAbsolute(_goal);
  }

  cout << "Goal: " << goal << endl;


  mlr::Transformation trans;
  trans.setAffineMatrix(eigen2mt(goal).p);

  std::stringstream buf1;
  std::stringstream buf2;

  buf1 << "(Control proxy) {} " << endl << "(Control pos)";
  buf2 << "(Control quat)";

  buf1 << "{ ref1=" << _endeff << " target=[" << trans.pos.x << " " << trans.pos.y << " " << trans.pos.z << " ] }";
  _pos_fact = buf1.str();

  buf2 <<  "{ ref1=" << _endeff << " target=[ " << trans.rot.w << " " << trans.rot.x << " " << trans.rot.y << " " << trans.rot.z << " ] }";
  _rot_fact = buf2.str();

}

