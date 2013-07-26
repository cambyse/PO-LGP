/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#ifndef MT_MotionPlanner_h
#define MT_MotionPlanner_h

#include "motion.h"

void threeStepGraspHeuristic(arr& q, MotionProblem& M, const arr& q0, uint shapeId, uint verbose);
void setGraspGoals(MotionProblem& M, uint T, uint shapeId, uint side, uint phase);
void setPlaceGoals(MotionProblem& M, uint T, uint shapeId, int belowToShapeId, const arr& locationTo);
void setHomingGoals(MotionProblem& M, uint T);

double keyframeOptimizer(arr& x, MotionProblem& M, double stopTolerance, bool x_is_initialized, uint verbose);
void interpolate_trajectory(arr &q, const arr& q0, const arr& qT, uint T);

void sineProfile(arr& q, const arr& q0, const arr& qT, uint T);

#endif