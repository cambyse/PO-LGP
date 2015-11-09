/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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

#pragma once
#include <Ors/ors.h>
#include <Optim/optimization.h>

//===========================================================================

struct KOMO{
  Graph specs;
  ors::KinematicWorld world;
  struct MotionProblem *MP;
  struct MotionProblemFunction *MPF;
  arr x, dual;

  KOMO(const Graph& specs);
  KOMO(const char* model_file, uint T, double duration);
  void init(const Graph& specs);
  void reset();
  void step();
  void run();
  Graph getReport();
  void checkGradients();
  void displayTrajectory(bool wait=true);
};

//===========================================================================

/// Return a trajectory that moves the endeffector to a desired target position
arr moveTo(ors::KinematicWorld& world, //in initial state
           ors::Shape& endeff,         //endeffector to be moved
           ors::Shape &target,         //target shape
           byte whichAxesToAlign=0,    //bit coded options to align axes
           uint iterate=1,
           int timeSteps=-1,
           double duration=-1.);            //usually the optimization methods may be called just once; multiple calls -> safety

//===========================================================================

inline arr finalPoseTo(ors::KinematicWorld& world,
                       ors::Shape &endeff,
                       ors::Shape& target,
                       byte whichAxesToAlign=0,
                       uint iterate=1){
  return moveTo(world, endeff, target, whichAxesToAlign, iterate, 0, 5.);
}

