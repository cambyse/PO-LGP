#include <Core/array.h>
#include <Kin/kin.h>
#include <Core/thread.h>

struct DecisionMakingModule : Thread {
  
  const static uint SYMBOLIC_ACTION__NO_ACTION_FOUND = 0;
  const static uint SYMBOLIC_ACTION__FINISHED = 1;
  const static uint SYMBOLIC_ACTION__GRAB = 2;
  const static uint SYMBOLIC_ACTION__PUTON = 3;
  
  //INPUT
  mlr::KinematicWorld *ors;
  
  //OUTPUT
  uint action;
  uint actionArgument; //body index in ors->bodies
  bool actionIsReady;
  
  DecisionMakingModule();

  void open();  //initialize,load knowledge, etc
  void step();  //map input to output..
  void close(); 
};
