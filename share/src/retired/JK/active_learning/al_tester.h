#ifndef _TESTER_H_
#define _TESTER_H_

#include "al_problem.h"

class ActiveLearner;

double test(const uint numTests, const ActiveLearner* al, const ActiveLearningProblem* problem, const mlr::String filename);


#endif
