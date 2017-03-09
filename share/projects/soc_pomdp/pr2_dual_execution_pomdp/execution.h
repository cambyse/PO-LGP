#ifndef EXECUTION_H
#define EXECUTION_H

#include <Core/array.h>
#include <vector>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Control/taskControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

#include "pomdp.h"

//extern double stickyWeight;


void getTrajectory(arr& x, arr& y, arr& dual, mlr::KinematicWorld& world, arr x0, const double& height, bool stickyness, uint horizon);
void POMDPExecution(FSC fsc, mlr::KinematicWorld& world, int num, double est_target);

#endif // EXECUTION_H
