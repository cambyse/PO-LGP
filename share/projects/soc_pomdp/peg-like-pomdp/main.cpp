
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/feedbackControl.h>
#include <Core/util.h>



#include <Optim/optimization.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

#include "execution.h"
#include "pomdp.h"



extern double stickyWeight;


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));
  //GOOD START
  world.setJointState(ARR(-0.0210641, -1.01319, -1.0215, 1.97184, -0.990911, 0.0370477, -0.0174248));



  uint T = 250; //time horizon
  uint numSamples_height = 1; //sampling height
  uint numSamples_pos    = 20; //each height has 10 sampled pos. In Total, we have 100 samples (hierarchical).
  uint total = numSamples_height*numSamples_pos + numSamples_height; //each height we generate one pseudo-sample (that will help to find the best observation)


  MT::timerStart(true);
  arr y0;
  double dual;
  arr x0 = world.getJointState();
  ors::Shape *endeff = world.getShapeByName("endeff");

  y0.resize(3);

  y0(0) = endeff->X.pos.x;
  y0(1) = endeff->X.pos.y;
  y0(2) = endeff->X.pos.z;
  dual = 0.;

  FSC fsc;
  NODE*Root = fsc.getRoot();
  Root->X() = x0;
  Root->Y() = y0;
  Root->Dual() = dual;


  arr samples;
  arr xx, yy, ddual;
  samples.resize(total,2); //2 means: height and pos
  xx.resize(total,x0.d0);
  yy.resize(total,y0.d0);
  ddual.resize(total,1);

  //adding a pseudo sample to each height (the one with the largest abs(x)

  uint index = 0;
  uint best=index;
  for(uint i=0;i<numSamples_height;i++){
      double sampled_height = 0.68; //fix here
      //cout<< sampled_height <<endl;
      samples[index](0) = sampled_height;    

      for(uint j=0;j<numSamples_pos;j++){
          samples[index](0) = sampled_height;
          samples[index](1) = -0.2 + .2*rnd.uni();
          index ++;
      }
      samples[index](0) = sampled_height;
      samples[index](1) = - 0.7; //the farthest on the right
      best = index;
      index++;
  }



  cout<<"samples size = "<< samples.d0 <<endl;
  cout<<"samples = "<<samples<<endl;


  Root->AllX() = xx;
  Root->AllY() = yy;
  Root->AllDual() = ddual;
  Root->Samples() = samples;
  Root->Model() = samples[best];




  //building a FSC controller;
  FSC::Horizon = T;
  OptimizeFSC(world, Root, 0);
  //OptimizeFSC_Test(world, Root, 0);
  //write the FSC controller to .dot file
  write_to_graphviz(fsc);


  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;




  //TEST POMDP
    orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

    //There will be two uncertain factors: height (z-axis) and position (only x-axis)

    for(uint i=0;i<20;i++){
        //table with changing heights
      world.getBodyByName("table")->X.pos.z   = 0.68 ;// .0 + 0.1*rnd.gauss();
      world.getBodyByName("target")->X.pos.z   = 0.7;// .0 + 0.1*rnd.gauss();


      world.getBodyByName("table")->X.pos.x   = samples[2](1);// .0 + 0.1*rnd.gauss();
      world.getBodyByName("target")->X.pos.x  = -0.6;// .0 + 0.1*rnd.gauss();
      //world.getBodyByName("target")->X.pos.x  = samples[i](1);// .0 + 0.1*rnd.gauss();


      world.setJointState(x0);
      POMDPExecution(fsc, world, i, 0.0);
    }



/*/

  arr allx,ally,alldual;

  allx.resize(total,T+1,x0.d0);
  ally.resize(total,T+1,y0.d0);
  alldual.resize(total,T+1);

  uint index = 0;

  //adding a pseudo sample to each height (the one with the largest abs(x)
  for(uint i=0;i<numSamples_height;i++){
      double sampled_height = .0 + 0.2*rnd.gauss();  //height INFORMATION
      samples[index](0) = sampled_height;

      for(uint j=0;j<numSamples_pos;j++){
          samples[index](1) = .0 + 1.*rnd.gauss();;
          xx[index]() = x0;
          yy[index]() = y0;
          ddual[index]() = dual;


          arr x, y, dual_t;

          getTrajectory(x, y, dual_t, world, x0, samples[index], true, T);

          allx[index]() = x;
          ally[index]() = y;
          alldual[index]()=dual_t;


          index ++;


      }
  }

  //TEST POMDP
    orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

    //There will be two uncertain factors: height (z-axis) and position (only x-axis)
      index = 0;
    for(uint i=0;i<numSamples_height;i++){
        for(uint j=0;j<numSamples_pos;j++){
            //table with changing heights
              world.getBodyByName("hole")->X.pos.z   = samples[index](0);// .0 + 0.1*rnd.gauss();
              world.getBodyByName("target")->X.pos.z = samples[index](0);// .0 + 0.1*rnd.gauss();


              world.getBodyByName("hole")->X.pos.x   = samples[index](1);// .0 + 0.1*rnd.gauss();
              world.getBodyByName("target")->X.pos.x = samples[index](1);// .0 + 0.1*rnd.gauss();



              world.setJointState(allx[0][0]);
              cout<< alldual[index]<<endl;
              POMDPExecution(allx[index], ally[index], alldual[index], world, i);

              index ++;
        }
    }

/*/
  return 0;

}
