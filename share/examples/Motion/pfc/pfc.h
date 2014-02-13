#ifndef PFC_H
#define PFC_H

#include <Core/util.h>
#include <Core/array.h>
#include <Core/array_t.h>
#include <Ors/ors.h>
#include <Core/util.h>
#include <stdlib.h>
#include "mobject.h"
#include "../splines/spline.h"
#include <Gui/opengl.h>

struct Pfc{

    Pfc(ors::KinematicWorld &_world, arr& _trajRef, double _dt, double _TRef, arr &_x0, arr &_q0, MObject &_goalMO, \
        bool _useOrientation);
    void printState();
    void plotState();
    void warpTrajectory();
    void iterate(arr &state);
    void getNextState(arr &state, arr &dstate);

    void moveGoal(arr &_pos);

    void computeIK(arr &q, arr &qd);
    ors::KinematicWorld *world;

    double dt;
    double TRef;
    double dsRef;

    bool useOrientation;
    bool useCollAvoid;

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
    Spline *trajWrap;

    // Reference Trajectory
    Spline *trajRef;
    arr dtrajRef;
    arr goalRef;
    arr sRef;

    String scene;
    arr joints_bk;

};

#endif // PFC_H
