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
    floatA  poselhthumbmaxopen  , poselhindexmaxopen, poselhthumbminopen  , poselhindexminopen;
    float distlhmaxopen = 0;
    float distlhminopen = 50;
    floatA  poserhthumbmaxopen  , poserhindexmaxopen, poserhthumbminopen  , poserhindexminopen;
    float distrhmaxopen = 0;
    float distrhminopen = 50;


    uint SN =50;
    floatA shoulderL;
    floatA shoulderR;
    arr dp_i;
    arr rpmean;
    arr lpmean;
    arr rPmean;
    arr lPmean;
    arr rP;
    arr lP;
    arr rp;
    arr lp;
    arr sr;
    double rpp;
    double lpp;
    
   // void doshouldercalc();
    void doinit(floatA a,int button);
    void gripperinit(floatA a);
    void getshoulderpos(floatA a);
/////////////////////////////////////////////

    G4HutoRoMap();
    MocapID mid;
    floatA centerpos;
    void open();
    void step();
    void close();


};


