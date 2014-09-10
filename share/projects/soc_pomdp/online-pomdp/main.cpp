#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

#include "pomdp.h"
#include "execution.h"

//NOTE:
// 1. The stickyweight is currently set to 10.0 (high? vs. 1.0 previously)





int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));
  uint T = 200; //time horizon
  uint numSamples = 70;




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


  MT::timerStart(true);



  arr heights;
  heights.resize(numSamples);
  arr xx, yy, ddual;


  xx.resize(numSamples,x0.d0);
  yy.resize(numSamples,y0.d0);
  ddual.resize(numSamples,1);

  for(uint i=0;i<numSamples;i++){
      heights(i) = .65 + 0.1*rnd.gauss();  //TABLE INFORMATION
      //3. store the trajectories
      xx[i]() = x0;
      yy[i]() = y0;
      ddual[i]() = dual;
  }
  Root->AllX() = xx;
  Root->AllY() = yy;
  Root->AllDual() = ddual;
  Root->Heights() = heights;
  Root->Height() = heights(0);


  //building a FSC controller;
  FSC::Horizon = T;
  //OptimizeFSC_Test(world, Root, 0); //start optimize the FSC from level 0 (root node)
  OptimizeFSC(world, Root, 0);





  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;





  //write the FSC controller to .dot file
  write_to_graphviz(fsc);



  //TESTING: Online POMDP planning
  //POMDP
  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;


  ors::Body *est_target = world.getBodyByName("target");

  double est = 0.6;

  for(uint i=0;i<3;i++){
    world.getBodyByName("table")->X.pos.z = 0.75;
    world.setJointState(x0);
    POMDPExecution(fsc, world, i,est);
  }

}


