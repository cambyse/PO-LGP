#pragma once

#include <Actions/TaskControllerModule.h>
#include <Actions/RelationalMachineModule.h>

void createSymbolsForShapes(RelationalMachine& RM, const ors::KinematicWorld& world);
void createSymbolsForActivities(RelationalMachine& RM, const Graph& activityRegistry);
