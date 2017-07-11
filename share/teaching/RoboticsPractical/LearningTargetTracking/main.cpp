#include <iostream>
#include "../interface/myBaxter.h"

#include "environment.h"
#include "agent.h"
#include "simulation.h"

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  rosCheckInit("minimalPositionControl");
  mlr::rnd.clockSeed();

  bool runACM=true;
  bool Ros=false;

  if (runACM == false){
    // run simulation of Q-Lambda Algorithm
    simulation sim;
    sim.run();
  }
  else{

    // run actor-critic

    environment env(Ros);
    Access<sensor_msgs::JointState> jointState(NULL, "jointState");
    RosCom_Spinner spinner;
    threadOpenModules(true);

    Access<arr> ctrl_q_ref(NULL, "ctrl_q_ref");
    ctrl_q_ref.waitForRevisionGreaterThan(10);

    agent ag;

    arr marker_pos = ARR(1, 1);
    arr arm_pos = ARR(0, 0);
    env.start(marker_pos, arm_pos, Ros);
    ag.learn(env, Ros);
    ag.writeParameter(Ros);
    ag.test(env, Ros);
    threadCloseModules();
  }
  cout <<"bye bye" <<endl;
  return 0;
}
