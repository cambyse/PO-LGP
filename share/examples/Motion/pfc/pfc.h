#ifndef PFC_H
#define PFC_H

#include <Core/util.h>
#include <Core/array.h>
#include <Ors/ors.h>
#include <Ors/ors.h>
#include <Core/util.h>
#include <stdlib.h>
#include "mobject.h"
#include "../splines/spline.h"

struct Pfc{

    Pfc(arr& _trajRef,double _TRef, arr &_x0, MObject &_goalMO, bool _useOrientation );
    void printState();
    void plotState();
    void warpTrajectory();
    void iterate(arr &state);
    void moveGoal(arr &_pos);

    double dt;
    double eps_goal;

    MObject *goalMO;

    // Actual Trajectory
    arr traj;
    arr x0; // start pos
    arr s;
    arr goal;
    arr lastGoal;
    arr state;

    // Wrapped Trajectory
    Spline *trajWrap;


    // Reference Trajectory
    Spline *trajRef;
    arr dtrajRef;
    arr goalRef;
    arr sRef;

    double TRef;
    double dsRef;

    bool useOrientation;
};

#endif // PFC_H
