#pragma once

#include "TaskControllerModule.h"
#include "RelationalMachineModule.h"

void createSymbolsForShapes(RelationalMachine& RM, const ors::KinematicWorld& world);
void createSymbolsForActivities(RelationalMachine& RM, const Graph& activityRegistry);
