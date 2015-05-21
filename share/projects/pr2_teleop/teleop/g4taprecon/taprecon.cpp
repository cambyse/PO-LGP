#include "taprecon.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>
#include <Core/array.h>
#include <Core/thread.h>
#include <Gui/plot.h>
#include <Core/util.h>

// ############################################################################


///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Init//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////




void G4MoveRecon::doInit(floatA tempData,int button)
{




    if(button & BTN_X)
    {

            rec_done = true;

            sample.append(~tempData);
            makerec();
    }
    else if(rec_done)
    {
            rec_done = false;


      cout<<cout<<"\x1B[2J\x1B[H";

      cout<<" RECORDING DONE"<<endl;
      // cout<<length(preRecZ)><"     "<<preRecQ1<<endl;



    }
    else if(button & BTN_Y)
    {
             sample.append(~tempData);
             if(sample.d0 == preRecZ.N)
             {
                 maketest();
             }
             else if(sample.d0 > preRecZ.N)
             {
                floatA tempshift = sample;
                sample.clear();
                for(uint i = 1;i <= preRecZ.N;i++)
                {
                     sample.append(tempshift.row(i));
                }
                maketest();
             }

    }
    else
    {
        sample.clear();
    }



}
void G4MoveRecon::makerec()
{

    //floatA feature = sample;
    //feature =feature-feature.min();
    //preRec =feature/feature.makeConditional() ;

    floatA featureX = sample.col(0);
    floatA featureY = sample.col(1);
    floatA featureZ = sample.col(2);
    floatA featureQ1 = sample.col(3);
    floatA featureQ2 = sample.col(4);
    floatA featureQ3 = sample.col(5);
    floatA featureQ4 = sample.col(6);

        cout<<"REC SAMPLE"<<endl;
        cout<<sample<<endl;
        cout<<cout<<"\x1B[2J\x1B[H";

    float lowfeatureX= featureX.min();
    float lowfeatureY= featureY.min();
    float lowfeatureZ= featureZ.min();
    float lowfeatureQ1= featureQ1.min();
    float lowfeatureQ2= featureQ2.min();
    float lowfeatureQ3= featureQ3.min();
    float lowfeatureQ4= featureQ4.min();


   featureX = featureX-lowfeatureX  ;
     featureY = featureY -lowfeatureY;
    featureZ = featureZ -lowfeatureZ ;
     featureQ1 = featureQ1-lowfeatureQ1;
     featureQ2 = featureQ2-lowfeatureQ2 ;
    featureQ3 = featureQ3- lowfeatureQ3 ;
     featureQ4 = featureQ4- lowfeatureQ4 ;

    preRecX = featureX/length(featureX);
    preRecY = featureY/length(featureY);
      preRecZ = featureZ/length(featureZ);
   preRecQ1 = featureQ1/length(featureQ1);
    preRecQ2 = featureQ2/length(featureQ2);
   preRecQ3 = featureQ3/length(featureQ3);
    preRecQ4 = featureQ4/length(featureQ4);




}
void G4MoveRecon::maketest()
{
   floatA featureX = sample.col(0);
    floatA featureY = sample.col(1);
    floatA featureZ = sample.col(2);
    floatA featureQ1 = sample.col(3);
    floatA featureQ2 = sample.col(4);
    floatA featureQ3 = sample.col(5);
    floatA featureQ4 = sample.col(6);

        cout<<"REC SAMPLE"<<endl;
       // cout<<feature<<endl;
        cout<<"\x1B[2J\x1B[H";

    float lowfeatureX= featureX.min();
    float lowfeatureY= featureY.min();
    float lowfeatureZ= featureZ.min();

    float lowfeatureQ1= featureQ1.min();
    float lowfeatureQ2= featureQ2.min();
    float lowfeatureQ3= featureQ3.min();
    float lowfeatureQ4= featureQ4.min();
    featureX = featureX-lowfeatureX  ;
     featureY = featureY -lowfeatureY;
    featureZ = featureZ -lowfeatureZ ;
     featureQ1 = featureQ1-lowfeatureQ1;
     featureQ2 = featureQ2-lowfeatureQ2 ;
    featureQ3 = featureQ3- lowfeatureQ3 ;
     featureQ4 = featureQ4- lowfeatureQ4 ;

   float cx = scalarProduct(featureX/length(featureX),preRecX);
   float cy = scalarProduct(featureY/length(featureY),preRecY);
   float cz = scalarProduct(featureZ/length(featureZ),preRecZ);
   float cq1 = scalarProduct(featureQ1/length(featureQ1),preRecQ1);
   float cq2 = scalarProduct(featureQ2/length(featureQ2),preRecQ2);
   float cq3 = scalarProduct(featureQ3/length(featureQ3),preRecQ3);
   float cq4 = scalarProduct(featureQ4/length(featureQ4),preRecQ4);

        arr dataplot(14,preRecZ.N);
        cout<<dataplot<<endl<<endl;
        cout<<endl<<preRecZ<<endl<<endl;
      
        for(uint i = 0 ; i<preRecZ.N;i++)
        {
            dataplot(0,i)=(double)preRecZ(i,0);
            dataplot(1,i)=(double)(featureZ/length(featureZ))(i,0);
            dataplot(2,i)=(double)preRecX(i,0);
            dataplot(3,i)=(double)(featureX/length(featureX))(i,0);
           dataplot(4,i)=(double)preRecY(i,0);
            dataplot(5,i)=(double)(featureY/length(featureY))(i,0);
           dataplot(6,i)=(double)preRecQ1(i,0);
            dataplot(7,i)=(double)(featureQ1/length(featureQ1))(i,0);
           dataplot(8,i)=(double)preRecQ2(i,0);
            dataplot(9,i)=(double)(featureQ2/length(featureQ2))(i,0);
           dataplot(10,i)=(double)preRecQ3(i,0);
            dataplot(11,i)=(double)(featureQ3/length(featureQ3))(i,0);
           dataplot(12,i)=(double)preRecQ4(i,0);
            dataplot(13,i)=(double)(featureQ4/length(featureQ4))(i,0);

        }

         FILE("z.pltX") <<~dataplot;
  gnuplot("plot 'z.pltX' us 1, 'z.pltX' us 2, 'z.pltX' us 3, 'z.pltX' us 4, 'z.pltX' us 5, 'z.pltX' us 6, 'z.pltX' us 7, 'z.pltX' us 8, 'z.pltX' us 9, 'z.pltX' us 10, 'z.pltX' us 11, 'z.pltX' us 12, 'z.pltX' us 13, 'z.pltX' us 14", false,false, NULL);

        cout<<cx<<cy<<cz<<cq1<<cq2<<cq3<<cq4<<endl;
        cout<<cx*cy*cz*cq1*cq2*cq3*cq4<<endl;
        cout<<cout<<"\x1B[2J\x1B[H";

    float b =cx*cy*cz*cq1*cq2*cq3*cq4;

      if(b > 0.6)
      { cout<<"___________________________________________"<<endl;

        cout<<"------------------Succsess-----------------"<<endl;
        MT::wait(1);
        //initphase=false;
        sample.clear();
       // floatA a= feature1;
       // a.append(feature2);
       // KeyReconFrame.set()= a;


    }

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
    cout<<endl<<"-----------Start recording Recon sample,hold X------"<<endl;
    cout<<"-----------To test the recording, hold Y-----------"<<endl;

}
void G4MoveRecon::close()
{

}
void G4MoveRecon::step()
{
    /////////InPut Check///////
    arr gpstate = gamepadState.get();
    CHECK(gpstate.N, "ERROR: No GamePad found");
    int button = gpstate(0);
    floatA tempData = poses.get();
    if(tempData.N == 0)
     return;
    else
    tempData = mid.query(tempData,STRING("/human/rl/rf"));
    ///////////////////////////
    //cout<<tempData<<endl;
    if(initphase)
    {
        doInit(tempData,button);
        return;
    }
    else
    {
/*
             ////////LISTEN//////////////

             sample.append(tempData);
             if(sample.d0 == preRecZ.N)
             {
                 doSomeCalc();
             }
             else if(sample.d0 > preRecZ.N)
             {
                floatA tempshift = sample;
                sample.resize(0,2);
                for(uint i = 1;i <= preRecZ.N;i++)
                {
                     sample.append(tempshift.row(i));
                }
                 doSomeCalc();
             }
*/

            ///////////LISTEN///////////

    }
}
void G4MoveRecon::doSomeCalc()
{
    floatA feature1 = ~sample.col(0);
    floatA feature2 = ~sample.col(1);
    float lowfeature1= feature1.min();
    float lowfeature2= feature2.min();
    feature1=feature1-lowfeature1;
    feature2=feature2-lowfeature2;

    feature1 = feature1/length(feature1);
    feature2 = feature2/length(feature2);

    floatA corz = feature1*preRecZ;
    floatA corq1= feature2*preRecQ1;
    cout<<corz<<" "<<corq1<<" "<<corz*corq1<<endl;
        cout<<cout<<"\x1B[2J\x1B[H";

    float b = (corz*corq1).scalar();

      if(b > 0.6f)
      { cout<<"___________________________________________"<<endl;

        cout<<"------------------TAPED-----------------"<<endl;

        ReconPositive.set()=true;
       // floatA a= feature1;
       // a.append(feature2);
       // KeyReconFrame.set()= a;


      }
 }







