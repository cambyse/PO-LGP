/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once
#include <Kin/kin.h>
#include <Optim/optimization.h>
#include <Optim/lagrangian.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

//===========================================================================

struct KOMO{ //TODO: rename ManipOp
  Graph specs;
  mlr::KinematicWorld world;
  struct MotionProblem *MP;
  struct OptConstrained *opt;
  arr x, dual;
  arr z, splineB;

  double maxPhase;
  uint stepsPerPhase;

  int verbose;

  KOMO();
  ~KOMO();

  //-- (not much in use..) specs gives as logic expressions in a Graph (or config file)
  KOMO(const Graph& specs);
  void init(const Graph& specs);
  void setFact(const char* fact);


  //-- setup the problem
  void setModel(const mlr::KinematicWorld& W,
                bool meldFixedJoints=false, bool makeConvexHulls=false, bool makeSSBoxes=false, bool activateAllContacts=false);
  void useOnlyJointGroup(const StringA& groupNames);
  void setTiming(double _phases=1., uint _stepsPerPhase=10, double durationPerPhase=5., uint k_order=2, bool useSwift=true);

  //-- higher-level defaults
  void setConfigFromFile();
  void setPoseOpt(){
    setTiming(1., 2, 5., 1, false);
    setSquaredFixJointVelocities();
    setSquaredFixSwitchedObjects();
    setSquaredQVelocities();
  }
  void setSequenceOpt(double _phases){
    setTiming(_phases, 2, 5., 1, false);
    setSquaredFixJointVelocities();
    setSquaredFixSwitchedObjects();
    setSquaredQVelocities();
  }
  void setPathOpt(double _phases, uint stepsPerPhase=20, double timePerPhase=5.){
    setTiming(_phases, stepsPerPhase, timePerPhase, 2, false);
    setSquaredFixJointVelocities();
    setSquaredFixSwitchedObjects();
    setSquaredQAccelerations();
  }

  //-- tasks (cost/constraint terms) low-level
  struct Task* setTask(double startTime, double endTime, TaskMap* map, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=100., uint order=0);
//  struct Task* setTask(double startTime, double endTime, const char* mapSpecs, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=100., uint order=0);
  void setKinematicSwitch(double time, bool before, const char *type, const char* ref1, const char* ref2, const mlr::Transformation& jFrom=NoTransformation, const mlr::Transformation& jTo=NoTransformation);


  //-- tasks (transitions) mid-level
  void setHoming(double startTime=-1., double endTime=-1., double prec=1e-1);
  void setSquaredQAccelerations(double startTime=-1., double endTime=-1., double prec=1.);
  void setSquaredQVelocities(double startTime=-1., double endTime=-1., double prec=1.);
  void setSquaredFixJointVelocities(double startTime=-1., double endTime=-1., double prec=1e2);
  void setSquaredFixSwitchedObjects(double startTime=-1., double endTime=-1., double prec=1e2);

  //-- tasks (tasks) mid-level
  void setHoldStill(double startTime, double endTime, const char* shape, double prec=1e2);
  void setPosition(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2);
  void setVelocity(double startTime, double endTime, const char* shape, const char* shapeRel=NULL, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2);
  void setAlign(double startTime, double endTime, const char* shape,  const arr& whichAxis=ARR(1.,0.,0.), const char* shapeRel=NULL, const arr& whichAxisRel=ARR(1.,0.,0.), ObjectiveType type=OT_sumOfSqr, const arr& target=ARR(1.), double prec=1e2);
  void setTouch(double startTime, double endTime, const char* shape1, const char* shape2, ObjectiveType type=OT_sumOfSqr, const arr& target=NoArr, double prec=1e2);
  void setAlignedStacking(double time, const char* object, ObjectiveType type=OT_sumOfSqr, double prec=1e2);
  void setLastTaskToBeVelocity();
  void setCollisions(bool hardConstraint, double margin=.05, double prec=1.);
  void setLimits(bool hardConstraint, double margin=.05, double prec=1.);

  //-- kinematic switches mid-level
  void setKS_placeOn(double time, bool before, const char* obj, const char* table, bool actuated=false);
  void setKS_slider(double time, bool before, const char* obj, const char* slider, const char* table, bool actuated);


  //-- tasks (cost/constraint terms) high-level
  void setGrasp(double time, const char* endeffRef, const char* object, int verbose=0, double weightFromTop=1e1);
  void setPlace(double time, const char* endeffRef, const char* object, const char* placeRef, int verbose=0);
  void setPlaceFixed(double time, const char* endeffRef, const char* object, const char* placeRef, const mlr::Transformation& worldPose, int verbose=0);
  void setGraspSlide(double startTime, double endTime, const char* endeffRef, const char* object, const char* placeRef, int verbose=0, double weightFromTop=1e1);
  void setHandover(double time, const char* endeffRef, const char* object, const char* prevHolder, int verbose=0);
  void setAttach(double time, const char* endeff, const char* object1, const char* object2, mlr::Transformation& rel, int verbose=0);

  void setSlowAround(double time, double delta, double prec=10.);

  //-- tasks - logic level
  void setAbstractTask(double phase, const Graph& facts, int verbose=0);

  //-- tasks - high-level geometric
  void setTowersAlign();

  //-- deprecated
  void setMoveTo(mlr::KinematicWorld& world, //in initial state
                 mlr::Shape& endeff,         //endeffector to be moved
                 mlr::Shape &target,         //target shape
                 byte whichAxesToAlign=0);   //bit coded options to align axes

  //-- optimization macros
  void setSpline(uint splineT);
  void reset();
  void step();
  void run();
  Graph getReport(bool gnuplt=false);
  void checkGradients();
  bool displayTrajectory(double delay=0.01, bool watch=false);
  mlr::Camera& displayCamera();
};

//===========================================================================

/// Return a trajectory that moves the endeffector to a desired target position
arr moveTo(mlr::KinematicWorld& world, //in initial state
           mlr::Shape& endeff,         //endeffector to be moved
           mlr::Shape &target,         //target shape
           byte whichAxesToAlign=0,    //bit coded options to align axes
           uint iterate=1,
           int timeSteps=-1,
           double duration=-1.);            //usually the optimization methods may be called just once; multiple calls -> safety

//===========================================================================

inline arr finalPoseTo(mlr::KinematicWorld& world,
                       mlr::Shape &endeff,
                       mlr::Shape& target,
                       byte whichAxesToAlign=0,
                       uint iterate=1){
  return moveTo(world, endeff, target, whichAxesToAlign, iterate, 1, 5.);
}

