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

    Pfc(ors::Graph &_orsG, arr& _trajRef, double _TRef, arr &_x0, arr &_q0, MObject &_goalMO, \
        bool _useOrientation, bool _useCollAvoid, \
        double _fPos_deviation, double _fVec_deviation, double _yCol_deviation, double _w_reg);
    void printState();
    void plotState();
    void warpTrajectory();
    void iterate(arr &state);
    void moveGoal(arr &_pos);

    void computeIK(arr &q, arr &qd);

    double dt;
    double eps_goal;
    double fPos_deviation;
    double fVec_deviation;
    double yCol_deviation;
    double w_reg;

    double TRef;
    double dsRef;

    bool useOrientation;
    bool useCollAvoid;

    MObject *goalMO;
    ors::Graph *orsG;

    // Actual Trajectory
    arr traj;
    arr x0; // start pos
    arr q0;
    arr s;
    arr goal;
    arr lastGoal;
    arr state;

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
