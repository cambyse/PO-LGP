#include "act_Script.h"

Act_Script::Act_Script(Roopi* r, const Script& S, double beatIntervalSec)
  : Act(r), Thread("Act_Script", beatIntervalSec), script(S){
  if(beatIntervalSec<0.) threadStep();
  else threadLoop();
}

Act_Script::~Act_Script(){
  threadClose();
}

void Act_Script::step(){ int r = script(); Act::setStatus(r); }
