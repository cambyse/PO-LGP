#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include "Demonstration.h"
#include "InnerCostFunction.h"

Demonstration *execRun(Demonstration *demo, InnerCostFunction *icf);


#endif // SIMULATOR_H
