#ifndef ADAPTIVEMOTIONEXECUTION_H
#define ADAPTIVEMOTIONEXECUTION_H

#include <Core/util.h>
#include <Core/array.h>
#include <Core/array_t.h>
#include <Ors/ors.h>
#include <Core/util.h>
#include <stdlib.h>
#include "mobject.h"
#include <Algo/spline.h>
#include <Gui/opengl.h>

struct AdaptiveMotionExecution{

    AdaptiveMotionExecution(ors::KinematicWorld &_world, arr& _trajRef, double _dt, double _TRef, arr &_x0, arr &_q0, MObject &_goalMO, \
        bool _useOrientation);
    void printState();
    void plotState();
    void warpTrajectory();
    void iterate(arr &state, double _dtReal=0.);
    void getNextState(arr &state, arr &dstate);

    void moveGoal(arr &_pos);

    void computeIK(arr &q, arr &qd);
    ors::KinematicWorld *world;

    double dt;
    double TRef;
    double dsRef;

    bool useOrientation;

    MObject *goalMO;

    // Actual Trajectory
    arr traj;
    arr x0; // start pos
    arr q0;
    arr s;
    arr goal;
    arr lastGoal;
    arr state;

    arr desState;
    arr desVel;

    // Costs
    arr posCosts;
    arr vecCosts;
    arr colCosts;

    // Wrapped Trajectory
    MT::Path *trajWrap;

    // Reference Trajectory
    MT::Path *trajRef;
    arr dtrajRef;
    arr goalRef;
    arr sRef;

    String scene;
    arr joints_bk;
};

#endif // ADAPTIVEMOTIONEXECUTION_H
