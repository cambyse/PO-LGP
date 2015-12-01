#pragma once

#include <pr2/TaskControllerModule.h>
#include <Actions/RelationalMachineModule.h>

void createSymbolsForShapes(RelationalMachine& RM, const ors::KinematicWorld& world);
void createSymbolsForActivities(RelationalMachine& RM, const Graph& activityRegistry);
