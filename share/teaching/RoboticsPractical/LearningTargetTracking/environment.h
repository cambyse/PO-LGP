#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <iostream>
#include <stdlib.h>
#include <RosCom/baxter.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <RosCom/subscribeTabletop.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include "../interface/myBaxter.h"
//#include "agent.h"
using namespace std;

class environment{
public:
    arr marker_position;
    arr arm_position;
    MyBaxter myBaxter;
    CtrlTask *track;
    CtrlTask *track_left;
    CtrlTask *align1;
    CtrlTask *align2;
    CtrlTask *alignE;
    double z_arm;
    arr state_space;

    environment(bool useRos);
    ~environment();

    void start(arr marker_pos, arr arm_pos, bool useRos);
    arr get_reward(arr state, double threshold);
    arr get_next_state(arr state, arr action);
    arr get_next_state_baxter(arr state, arr action);
    bool in_table(arr state);
    void set_random_positions();
    void track_marker_position_baxter();
};

#endif // ENVIRONMENT_H
