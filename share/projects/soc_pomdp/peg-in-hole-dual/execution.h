#ifndef EXECUTION_H
#define EXECUTION_H

#include <Core/array.h>
#include <vector>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

//extern double stickyWeight;


void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, arr y0, const double& height, bool stickyness, uint horizon);
void POMDPExecution(const arr& allx, const arr& ally, const arr& alldual, ors::KinematicWorld& world, int num);

#endif // EXECUTION_H
