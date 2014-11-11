#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include "Demonstration.h"
#include "InnerCostFunction.h"

Demonstration *execRun(Demonstration *demo, InnerCostFunction *icf);


#endif // SIMULATOR_H
