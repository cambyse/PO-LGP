#pragma once

#include <Control/TaskControlThread.h>
#include <Actions/RelationalMachineModule.h>

void createSymbolsForShapes(RelationalMachine& RM, const mlr::KinematicWorld& world);
void createSymbolsForActivities(RelationalMachine& RM, const Graph& activityRegistry);
