#pragma once
#include <Core/thread.h>
#include <Control/taskControl.h>
#include <Kin/kin.h>
// #include <RosCom/actionMachine.h>
// #include <RosCom/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================

struct G4MoveRecon:Module
{
    //////////////////////////////
    ACCESS(floatA, poses);
    ACCESS(arr, gamepadState);
    ACCESS(bool, taped);
    ACCESS(bool, tapreconready);
    ACCESS(bool, calisaysokay);
    ACCESS(floatA, ftdata);
    /////////////////////////////
    /////INIT////////////////////
    /////////////////////////////
    
    floatA preRecX;
    floatA preRecY;
     floatA preRecQ2;
   floatA preRecQ3;
    floatA preRecQ4;

    floatA preRecZ;//       {0.f, 0.0026f, 0.0044f, 0.0389f, 0.3350f, 0.5545f, 0.6126f, 0.3353f, 0.0828f, 0.0428f,  0.0389f,  0.0782f, 0.1453f, 0.1373f, 0.1045f, 0.088f, 0.074f, 0.057f,  0.0509f, 0.0481f, 0.0460f};
    floatA preRecQ1;// = {0.0414f, 0.0288f,  0.0346f, 0.0794f, 0.2865f, 0.4759f, 0.5728f, 0.4630f,  0.1576f, 0.0630f,  0.05950f,  0.0999f,  0.1694f,  0.1983f,  0.1197f, 0.0849f,  0.0644f,  0.04385f,  0.0230f,  0.009f, 0.f};
    bool rec_done =false;
    bool rec_succ=false;
    bool initphase = true;
    floatA sample;
    void makerec();
    void maketest();
   ////////////////////////////// 
    floatA G4DataInput;
    int decay = 0;
    G4MoveRecon();
    floatA normG4DataInput;

    void doSomeCalc();
    void open();
    void close();
    void step();

};



