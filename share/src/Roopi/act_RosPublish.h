#pragma once

#include "act.h"

struct Act_RosPublish : Act{
  VariableBase& var;
  Thread *publisher;
  Act_RosPublish(Roopi *r, VariableBase& var, double beatIntervalSec=-1.);
  ~Act_RosPublish(){ delete publisher; }

  void write(ostream& os){ Act::write(os); os <<var.name <<' ' <<publisher->timer.report();  }
};

