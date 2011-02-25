#include <MT/array.h>
#include <MT/ors.h>
#include <MT/threads.h>

struct DecisionMakingModule:public StepThread{
  
  const static uint SYMBOLIC_ACTION__NO_ACTION_FOUND = 0;
  const static uint SYMBOLIC_ACTION__FINISHED = 1;
  const static uint SYMBOLIC_ACTION__GRAB = 2;
  const static uint SYMBOLIC_ACTION__PUTON = 3;
  
  //INPUT
  ors::Graph *ors;
  
  //OUTPUT
  uint action;
  uint actionArgument; //body index in ors->bodies
  bool actionIsReady;
  
  DecisionMakingModule();

  void open();  //initialize,load knowledge, etc
  void step();  //map input to output..
  void close(); 
};
