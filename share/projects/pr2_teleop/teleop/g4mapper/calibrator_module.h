#pragma once
#include <Core/thread.h>
#include <Motion/feedbackControl.h>
#include <System/engine.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================
struct G4HuToRoMap: Module
{

    ACCESS(floatA, poses);
    ACCESS(floatA, calibrated_pose_rh);
    ACCESS(floatA, calibrated_pose_lh);
    ACCESS(float, calibrated_gripper_rh);
    ACCESS(float, calibrated_gripper_lh);

    ACCESS(floatA, poses_rh);
    ACCESS(floatA, poses_lh);

    G4HuToRoMap();
    MocapID mid;
    void open();
    void close();
    void step();
    void transform(const floatA& poses_raw);
};


struct initG4Mapper:Thread
{


    ACCESS(float, poses);
    ACCESS(arr, gamepadState);
    ACCESS(floatA, calibrated_pose_rh);
    ACCESS(floatA, calibrated_pose_lh);
    ACCESS(float, calibrated_gripper_rh);
    ACCESS(float, calibrated_gripper_lh);

    ACCESS(floatA, poses_rh);
    ACCESS(floatA, poses_lh);


    Metronome *metronome;

    initG4Mapper();

    floatA posesSide, posesOpen, posesClosed,posesFront;
    floatA center;
    float radius;

    float m_rh, m_lh;
    float q_rh, q_lh;

    MocapID mid;
    void LoopWithBeatAndWaitForClose(double sec); //<- own mode
    void calibrate();

    void open();
    void step();
    void close();


};


