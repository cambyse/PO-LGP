#pragma once

#include <Optim/optimization.h>

struct PR2EndPoseProblem : ConstrainedProblem{
  struct sPR2EndPoseProblem& s;

  PR2EndPoseProblem();

  arr getInitialization();
  void setState(const arr&);
  void report();
};
