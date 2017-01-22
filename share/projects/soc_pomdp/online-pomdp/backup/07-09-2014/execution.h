#ifndef EXECUTION_H
#define EXECUTION_H

#include <Core/array.h>
#include <vector>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Control/taskController.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

extern double stickyWeight;


void getTrajectory(arr& x, arr& y, arr& dual, mlr::KinematicWorld& world, arr y0, const double& height, bool stickyness, uint horizon);
void POMDPExecution(const arr& allx, const arr& ally, const arr& alldual, mlr::KinematicWorld& world, int num);

#endif // EXECUTION_H
