#ifndef EXECUTION_H
#define EXECUTION_H

#include <Core/array.h>
#include <vector>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Control/taskControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

#include "pomdp.h"

//extern double stickyWeight;


void getTrajectory(arr& x, arr& y, arr& dual, mlr::KinematicWorld& world, arr y0, const arr& model, bool stickyness, uint horizon);
void POMDPExecution(const arr& allx, const arr& ally, const arr& alldual, mlr::KinematicWorld& world, int num);


void POMDPExecution(FSC fsc, mlr::KinematicWorld& world, int num, double est_target);

#endif // EXECUTION_H
