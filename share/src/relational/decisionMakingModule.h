#include <Core/array.h>
#include <Ors/ors.h>
#include <Core/module.h>

struct DecisionMakingModule:Module {
  
  const static uint SYMBOLIC_ACTION__NO_ACTION_FOUND = 0;
  const static uint SYMBOLIC_ACTION__FINISHED = 1;
  const static uint SYMBOLIC_ACTION__GRAB = 2;
  const static uint SYMBOLIC_ACTION__PUTON = 3;
  
  //INPUT
  ors::KinematicWorld *ors;
  
  //OUTPUT
  uint action;
  uint actionArgument; //body index in ors->bodies
  bool actionIsReady;
  
  DecisionMakingModule();

  void open();  //initialize,load knowledge, etc
  void step();  //map input to output..
  void close(); 
};
