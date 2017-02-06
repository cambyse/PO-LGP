#include "act.h"
#include <Core/util.h>

Act::Act(Roopi *r) : roopi(r), persist(false), startTime(mlr::realTime()) {}

double Act::time(){ return mlr::realTime()-startTime; }
