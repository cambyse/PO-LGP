#include <iostream>
#include <map>
#include <ctime>
#include <stdlib.h>
#include <cstdlib>
#include <TL/decisionMakingModule.h>
#include <TL/ors_actionInterface.h>



ActionInterface AI_2;


void initSimulator(const char* configurationFile) {
  AI_2.loadConfiguration(configurationFile);
  AI_2.startOde();
  AI_2.startSwift();
  AI_2.simulate(60);
}



int main(int argc, char** argv){
  // prepare test graph
  initSimulator("state.ors");
  
  DecisionMakingModule dmm;
  dmm.ors = AI_2.C;
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
      AI_2.grab(dmm.actionArgument);
      AI_2.simulate(100);
    }
    else if (dmm.action == dmm.SYMBOLIC_ACTION__PUTON) {
      cerr<<"Puton " << dmm.actionArgument << endl;
      AI_2.dropObjectAbove(dmm.actionArgument);
      AI_2.simulate(100);
    }
    else
      NIY;
  }
  dmm.close();
  
  AI_2.shutdownAll();
	return 0;
}
