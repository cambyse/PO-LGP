#include <iostream>
#include <map>
#include <ctime>
#include <stdlib.h>
#include <cstdlib>
#include <relational/decisionMakingModule.h>
#include <relational/robotManipulationSimulator.h>



RobotManipulationSimulator sim;


void initSimulator(const char* configurationFile) {
  sim.loadConfiguration(configurationFile);
  sim.startOde();
  sim.startSwift();
  sim.simulate(60);
}



int main(int argc, char** argv){
  // prepare test graph
  initSimulator("state.ors");
  
  DecisionMakingModule dmm;
  dmm.ors = sim.C;
  dmm.open();
  
  uint MAX_NUM_ACTIONS = 10;
  uint a;
  for (a=0; a<MAX_NUM_ACTIONS; a++) {
    dmm.step();
    PRINT(dmm.action);
    PRINT(dmm.actionArgument);
    if (dmm.action == dmm.SYMBOLIC_ACTION__FINISHED) {
      cerr<<"Finished!"<<endl;
      break;
    }
    else if (dmm.action == dmm.SYMBOLIC_ACTION__GRAB) {
      cerr<<"Grab " << dmm.actionArgument << endl;
      sim.grab(dmm.actionArgument);
      sim.simulate(100);
    }
    else if (dmm.action == dmm.SYMBOLIC_ACTION__PUTON) {
      cerr<<"Puton " << dmm.actionArgument << endl;
      sim.dropObjectAbove(dmm.actionArgument);
      sim.simulate(100);
    }
    else {
      HALT("No appropriate action was found.");
    }
  }
  dmm.close();
  
  sim.shutdownAll();
	return 0;
}
