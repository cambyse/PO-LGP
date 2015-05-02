#include "taprecon.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>
#include <Core/array.h>
#include <Core/thread.h>

// ############################################################################



///////////////////////////////////////////////////////////////////////////////
////////////////////////////////Thread Init////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void initG4MoveRecon::LoopWithBeatAndWaitForClose(double sec)
{
      if(!metronome)
         metronome=new Metronome("threadTiccer", sec);
      else
         metronome->reset(sec);

        state.waitForValueEq(tsCLOSE);

}

void initG4MoveRecon::open()
{
    mid.load("g4Mapping.kvg");
    cout<<"Start recording Recon sample, press X"<<endl;
    cout<<"Stop  recording Recon sample, press Y"<<endl;
    sample.resize(0);
}
void initG4MoveRecon::step()
{
    arr gpstate = gamepadState.get();
    CHECK(gpstate.N, "ERROR: No GamePad found");
    int button = gpstate(0);
    floatA tempData = poses.get();
    tempData = mid.query(tempData,STRING("/human/rl/rf")).cols(3,4);
    if(tempData.N == 0)
        return;

    if(button & BTN_X)
    {
        start_rec_sample = true;
    }
    else if(button & BTN_Y)
    {
        sample.resize(0);
        start_rec_sample = false;
    }


    if(start_rec_sample)
    {
        sample.append(tempData);
        if(sample.d1 == preRecZ.N)
        {
            doextract();
        }
        else if(sample.d1 > preRecZ.N)
        {
            floatA tempshift = sample;
            sample = sample.resize(0);
            for(uint i = 1;i < preRecZ.N;i++)
            {
                sample.append(tempshift.row(i));
            }
            doextract();
        }

    }

}
void initG4MoveRecon::doextract()
{
    floatA feature1 = ~sample.col(0);
    floatA feature2 = ~sample.col(1);
    float lowfeature1= feature1(0);
    float lowfeature2= feature2(0);
    for(uint i = 1; i<feature1.N;i++)
    {
        if(lowfeature1>=feature1(i))
        {
            lowfeature1 = feature1(i);
        }
        if(lowfeature2>=feature2(i))
        {
            lowfeature2 = feature2(i);
        }
    }
    feature1=feature1-lowfeature1;
    feature2=feature2-lowfeature2;

    feature1 =feature1/sqrt(feature1*~feature1);
    feature2 =feature2/sqrt(feature2*~feature2);

    if(sqrt(feature1*~preRecZ)(0)*sqrt(feature2*~preRecQ1)(0) > 0.8f)
    {
        cout<<"Succsess"<<endl;
        floatA a= feature1;
        a.append(feature2);
        KeyReconFrame.set()= a;
        state.setValue(tsCLOSE);
        close();
     }

}
void initG4MoveRecon::close()
{

}




/////////////////////////////////////////////////////////////////////
//////////////////////////G4MoceRecon module/////////////////////////
/////////////////////////////////////////////////////////////////////



G4MoveRecon::G4MoveRecon()
{
}

void G4MoveRecon::open()
{
    mid.load("g4mapping.kvg");
    initG4MoveRecon *it1 = new initG4MoveRecon();
    ReconPositive.set()=false;
    cout<<"G4MoveRecon calibrating_thread Start"<<endl;
    it1->LoopWithBeatAndWaitForClose(0.05); 
    cout<<"G4MoveRecon calibrating_thread Finish"<<endl;
}
void G4MoveRecon::step()
{
    // DO LISTENING STUFF
    floatA tempData = poses.get();
    tempData = mid.query(tempData,STRING("/human/rl/rf")).cols(3,4);
    floatA KeyReconFrametemp = KeyReconFrame.get();
    if(tempData.N == 0)
        return;

    G4DataInput.append(tempData);
    if(G4DataInput.d1 == KeyReconFrametemp.d0)
    {
           doSomeCalc();
    }
    else if(G4DataInput.d1 > KeyReconFrametemp.d0)
    {
            floatA tempshift = G4DataInput;
            G4DataInput = G4DataInput.resize(0);
            for(uint i = 1;i<KeyReconFrametemp.d0;i++)
            {
                G4DataInput.append(tempshift.row(i));
            }
            doSomeCalc();
    }
    else
    {
        return;
    }
    floatA crossCORR = KeyReconFrametemp*normG4DataInput;
    if(sqrt(crossCORR(0,0)*crossCORR(1,1)) >0.8f)
    {
        ReconPositive.set() = true; 
    }


}
void G4MoveRecon::doSomeCalc()
{
    floatA feature1 = ~G4DataInput.col(0);
    floatA feature2 = ~G4DataInput.col(1);
    float lowfeature1= feature1(0);
    float lowfeature2= feature2(0);
    for(uint i = 1; i<feature1.N;i++)
    {
        if(lowfeature1>=feature1(i))
        {
            lowfeature1 = feature1(i);
        }
        if(lowfeature2>=feature2(i))
        {
            lowfeature2 = feature2(i);
        }
    }
    feature1=feature1-lowfeature1;
    feature2=feature2-lowfeature2;

    feature1 =feature1/sqrt(feature1*~feature1);
    feature2 =feature2/sqrt(feature2*~feature2);

    normG4DataInput = normG4DataInput.resize(0);
    floatA a= feature1;
    a.append(feature2);
    normG4DataInput= ~a;
}







