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
#include <Motion/taskMaps.h>

//===========================================================================

struct KOMO{
  Graph specs;
  ors::KinematicWorld world;
  struct MotionProblem *MP;
  struct OptConstrained *opt;
  arr x, dual;
  arr z, splineB;

  double maxPhase;
  uint stepsPerPhase;

  int verbose;

  KOMO();
  ~KOMO();

  //-- specs gives as logic expressions
  KOMO(const Graph& specs);
  void init(const Graph& specs);
  void setFact(const char* fact);

  //-- manual specs helpers
  //-- setup
  void setConfigFromFile();
  void setModel(const ors::KinematicWorld& W,
                bool meldFixedJoints=false, bool makeConvexHulls=false, bool makeSSBoxes=false, bool activateAllContacts=false);
  void setTiming(double _phases=1, uint _stepsPerPhase=10, double durationPerPhase=5., uint k_order=2, bool useSwift=true);
  void setSinglePoseOptim(double duration=5.){ setTiming(1, 1, duration, 1); }
  void setSequenceOptim(uint frames, double duration=5.){ setTiming(frames, 1, duration, 1); }


  //-- tasks (cost/constraint terms) low-level
  struct Task* setTask(double startTime, double endTime, TaskMap* map, TermType type=sumOfSqrTT, const arr& target=NoArr, double prec=100., uint order=0);
  struct Task* setTask(double startTime, double endTime, const char* mapSpecs, TermType type=sumOfSqrTT, const arr& target=NoArr, double prec=100., uint order=0);
  void setKinematicSwitch(double time, bool before, const char *type, const char* ref1, const char* ref2, const ors::Transformation& jFrom=NoTransformation, const ors::Transformation& jTo=NoTransformation);

  //-- tasks (cost/constraint terms) mid-level
  void setHoming(double startTime=-1., double endTime=-1., double prec=1e-1);
  void setSquaredQAccelerations(double startTime=-1., double endTime=-1., double prec=1.);
  void setSquaredQVelocities(double startTime=-1., double endTime=-1., double prec=1.);
  void setSquaredFixJointVelocities(double startTime=-1., double endTime=-1., double prec=1.);
  void setSquaredFixSwitchVelocities(double startTime=-1., double endTime=-1., double prec=1.);


  void setHoldStill(double startTime, double endTime, const char* joint, double prec=1e2);
  void setPosition(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, TermType type=sumOfSqrTT, const arr& target=NoArr, double prec=1e2);
  void setAlign(double startTime, double endTime, const char* shape,  const arr& whichAxis=ARR(1.,0.,0.), const char* shapeRel=NULL, const arr& whichAxisRel=ARR(1.,0.,0.), TermType type=sumOfSqrTT, const arr& target=ARR(1.), double prec=1e2);
  void setLastTaskToBeVelocity();
  void setCollisions(bool hardConstraint, double margin=.05, double prec=1.);
  void setLimits(bool hardConstraint, double margin=.05, double prec=1.);


  //-- tasks (cost/constraint terms) high-level
  void setGrasp(double time, const char* endeffRef, const char* object, bool effKinMode=false, int verbose=0);
  void setPlace(double time, const char* endeffRef, const char* object, const char* placeRef, bool effKinMode=false, int verbose=0);
  void setSlowAround(double time, double delta);

  //-- tasks - logic level
  void setAbstractTask(double phase, const NodeL& facts, bool effKinMode=false, int verbose=0);

  void setMoveTo(ors::KinematicWorld& world, //in initial state
                 ors::Shape& endeff,         //endeffector to be moved
                 ors::Shape &target,         //target shape
                 byte whichAxesToAlign=0);   //bit coded options to align axes

  //-- optimization macros
  void setSpline(uint splineT);
  void reset();
  void step();
  void run();
  Graph getReport(bool gnuplt=false);
  void checkGradients();
  void displayTrajectory(double delay=0.01);
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
  return moveTo(world, endeff, target, whichAxesToAlign, iterate, 1, 5.);
}

