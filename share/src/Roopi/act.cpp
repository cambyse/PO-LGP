#include "act.h"
#include <Core/util.h>

Act::Act() : startTime(mlr::realTime()) {}

double Act::time(){ return mlr::realTime()-startTime; }
