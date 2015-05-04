#pragma once
#include <Core/thread.h>
#include <Motion/feedbackControl.h>
#include <System/engine.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================

 struct initG4MoveRecon:Thread
{

    const char* name = "trit";
    ACCESS(floatA, poses);
    ACCESS(arr, gamepadState);
    ACCESS(floatA, KeyReconFrame);
    uint n=1;
    Metronome *metronome;
    //const char* name ='3';
    initG4MoveRecon(uint n):Thread(STRING(name+n)), n(n) {};

    //    ~initG4MoveRecon(){}

    floatA sample;


    floatA preRecZ = {0., 0.00260602635024900, 0.00445196168167591, 0.0389818108224771, 0.335091554869541, 0.554540690446775, 0.612633361171079, 0.335308723732061, 0.0828499210516721, 0.0428908503478524,  0.0389818108224771,  0.0782893749387375, 0.145394553457652, 0.137359305544384, 0.104566807303749, 0.0880619737521708, 0.0741631665508424, 0.0576583329992647,  0.0509260982611203, 0.0481029030483511, 0.0460397988544024};


    floatA preRecQ1 = {0.0414935448060658, 0.0288475784473524,  0.0346915705000848, 0.0794704839113353, 0.286505764542079, 0.475964022827168, 0.572867381284295, 0.463077810135351,  0.157640634544750, 0.0630796778353776,  0.0595000074721417,  0.0999995330749935,  0.169481775687568,  0.198341366451576,  0.119780815526868, 0.0849120633561200,  0.0644520850132288,  0.0438539650288036,  0.0230696541362234,  0.00903025904551198, 0};


    bool start_rec_sample =false;
    MocapID mid;

    void LoopWithBeatAndWaitForClose(double sec); //<- own mode
    void doextract();


    void open();
    void step();
    void close();


};




struct G4MoveRecon:Module
{

    ACCESS(floatA, poses);
    ACCESS(floatA, KeyReconFrame);
    ACCESS(bool, ReconPositive);
    floatA G4DataInput;
    initG4MoveRecon *inth;
    G4MoveRecon();
    MocapID mid;
    floatA normG4DataInput;

    void doSomeCalc();
    void open();
    void close();
    void step();

};



