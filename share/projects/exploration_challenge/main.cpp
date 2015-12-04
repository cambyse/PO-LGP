#include "util.h"
#include <Core/array.h>
#include <Eigen/Dense>
#include <iostream>

#include "pr2ControlSet.h"
#include "pr2System.h"
#include "qItselfController.h"

#include <hybrid_automaton/JumpCondition.h>
#include <hybrid_automaton/SubjointConfigurationSensor.h>
#include <hybrid_automaton/HybridAutomaton.h>

#include <Actions/swig.h>

int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);
  ActionSwigInterface interface;

  pr2System::Ptr system(new pr2System(interface));

  // First control mode raises head
  ha::ControlMode::Ptr first_cm(new ha::ControlMode("first"));
  pr2ControlSet::Ptr first_control_set(new pr2ControlSet());
  qItselfController::Ptr raise_head(new qItselfController());
  raise_head->setSystem(system);
  raise_head->setEndeff("head_tilt_joint");
  ::Eigen::MatrixXd goal(1, 1);
  goal(0, 0) = M_PI/2.;
  raise_head->setGoal(goal);
  first_control_set->appendController(raise_head);
  first_cm->setControlSet(first_control_set);

  // second control mode lowers it again
  ha::ControlMode::Ptr second_cm(new ha::ControlMode("second"));
  pr2ControlSet::Ptr second_control_set(new pr2ControlSet());
  qItselfController::Ptr lower_head(new qItselfController());
  lower_head->setEndeff("head_tilt_joint");
  lower_head->setSystem(system);
  ::Eigen::MatrixXd goal2(1, 1);
  goal2(0, 0) = 0*M_PI;
  lower_head->setGoal(goal2);
  second_control_set->appendController(lower_head);
  second_cm->setControlSet(second_control_set);


  ha::SubjointConfigurationSensor::Ptr head_tilt(new ha::SubjointConfigurationSensor());
  head_tilt->setSystem(system);
  head_tilt->setIndex(std::vector<int>({interface.getQIndex("head_tilt_joint")}));
  ha::JumpCondition::Ptr bigger(new ha::JumpCondition());;
  bigger->setEpsilon(10e-2);
  bigger->setSensor(head_tilt);
  bigger->setConstantGoal(M_PI/4.);
  bigger->setGoalAbsolute();

  ha::ControlSwitch::Ptr edge(new ha::ControlSwitch());
  edge->setName("edge1");
  edge->add(bigger);


  ha::HybridAutomaton::Ptr hybrid_automaton(new ha::HybridAutomaton());
  hybrid_automaton->addControlMode(first_cm);
  hybrid_automaton->addControlSwitchAndMode("first", edge, second_cm);
  hybrid_automaton->setCurrentControlMode("first");

  hybrid_automaton->initialize(.0);

  for(uint i=0; i<10000; ++i) {
    hybrid_automaton->step(.01);
    //std::cout << "JC active: " << bigger->isActive() << std::endl;
    //std::cout << "Sensor value: " << bigger->getSensor()->getCurrentValue() << std::endl;
    mlr::wait(.01);
  }

  hybrid_automaton->terminate();
}
