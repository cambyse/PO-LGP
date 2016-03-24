#pragma once
#include <Core/thread.h>
#include <Control/taskController.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================


struct G4HutoRoMap:Module
{
    ACCESS(bool, initmapper);

    ACCESS(floatA, poses);
    ACCESS(arr, gamepadState);

    ACCESS(floatA, calibrated_pose_rh);
    ACCESS(floatA, calibrated_pose_lh);

    ACCESS(float, calibrated_gripper_rh);
    ACCESS(float, calibrated_gripper_lh);

    ACCESS(floatA, poses_rh);
    ACCESS(floatA, poses_lh);

    ACCESS(bool, taped);
    ACCESS(arr, drive);
    ACCESS(bool, tapreconready);
    ACCESS(bool, calisaysokay);

    ACCESS(floatA, ftdata);

/////////////////////INIT////////////////////
    bool initphase = true;
    floatA  poselhthumbmaxopen  , poselhindexmaxopen, poselhthumbminopen  , poselhindexminopen;
    float distlhmaxopen = 0.1;
    float distlhminopen = 0.04;
    floatA  poserhthumbmaxopen  , poserhindexmaxopen, poserhthumbminopen  , poserhindexminopen;
    float distrhmaxopen = 0.1;
    float distrhminopen = 0.04;
    
    bool demoidle = false;
    bool tapedacc = false;
    float calarm_r_r = 0;
    float calarm_r_l = 0;

    uint SN =50;

    floatA shoulderL;
    floatA shoulderLori;
    floatA shoulderR;
    floatA shoulderRori;

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
    arr sl;
    double rpp;
    double lpp;
    bool shoulderinit = true;
   // void doshouldercalc();
    void doinit(floatA a,int button);
    void gripperinit(floatA a);
    void getshoulderpos(floatA a);
    void getARMradius(floatA a);
    void getUnitPos(floatA a);
/////////////////////////////////////////////
    

    G4HutoRoMap();
    MocapID mid;
    floatA centerpos;
    floatA centerORI;
    floatA UnitPosR;
    floatA UnitPosL;

    double x,y,phistand,phi;
    void calcparameters(floatA a);
    bool decayed=true;
    bool tapready = false;

    void transform(const floatA& a);
    void open();
    void step();
    void close();
/////////////////////////////////////////////
//
    void doinitsendROS(floatA a);
/*
    void doinitandrea(floatA a,int button);
    void caliandrea();
    void transform_andrea();

    floatA posesSideR;
    floatA posesSideL;
    floatA posesOpen;
    floatA posesFrontR;
    floatA posesFrontL;
    floatA posesClosed;
    float radiusR_andrea;    
    float radiusL_andrea;
    arrf center_andrea;
    float m_rh_andrea, m_lh_andrea;
    float q_rh_andrea, q_lh_andrea;

/////////////////////////////////////////////    
  */  
    void patterndriving(floatA tempData);
    void initdriving(floatA tempData, int button);
    bool  btnpressed = true;
    floatA turnL;
    floatA turnR;
    floatA driveposX;
    floatA drivenegX;
    floatA driveRY;
    floatA driveLY;
    bool driveready = false;

    void doinitpresaved(int button);/*
*/
};


