#pragma once
#include <Core/thread.h>
#include <Motion/feedbackControl.h>
#include <System/engine.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================


struct G4HutoRoMap:Module
{


    ACCESS(floatA, poses);
    ACCESS(arr, gamepadState);
    ACCESS(floatA, calibrated_pose_rh);
    ACCESS(floatA, calibrated_pose_lh);
    ACCESS(float, calibrated_gripper_rh);
    ACCESS(float, calibrated_gripper_lh);

    ACCESS(floatA, poses_rh);
    ACCESS(floatA, poses_lh);

/////////////////////INIT////////////////////
    bool initphase = true;
    floatA  poselhthumbmaxopen  , poselhindexmaxopen;
    float distlhmaxopen = 0;
    floatA  poserhthumbmaxopen  , poserhindexmaxopen;
    float distrhmaxopen = 0;
    uint SN =150;
    floatA shoulderL,shoulderR;
    floatA lp_i;
    floatA rp_i;
   // void doshouldercalc();
    void doinit(floatA a,int button);
    void gripperinit(floatA a);
    floatA getshoulderpos(floatA a,floatA &b);
/////////////////////////////////////////////
    G4HutoRoMap();
    MocapID mid;
    floatA centerpos;
    void open();
    void step();
    void close();


};


