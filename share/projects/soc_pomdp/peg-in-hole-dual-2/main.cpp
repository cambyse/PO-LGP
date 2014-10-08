
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
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

  //initial prior: position 2e2; collision (0.04; 0.5e1)
  //world.setJointState(ARR(-0.130734, 0.200912, -0.0176154, -0.541507, -0.743056, -0.415109, -0.0146599 ));
  //To make the peg close enough to the table
  //world.setJointState(ARR(-0.305148, 0.121278, -0.0600021, -0.638602, -0.87011, -0.490926, -0.0387221));
  world.setJointState(ARR( -0.23225, 0.245013, -0.0470437, -0.691428, -0.869523, -0.368891, 0.0488829));

  uint T = 200; //time horizon
  uint numSamples_height = 1; //sampling height
  uint numSamples_pos    = 3; //each height has 10 sampled pos. In Total, we have 100 samples (hierarchical).
  uint total = numSamples_height*numSamples_pos + numSamples_height; //each height we generate one pseudo-sample (that will help to find the best observation)


  MT::timerStart(true);
  arr y0;
  double dual;
  arr x0 = world.getJointState();
  ors::Shape *endeff = world.getShapeByName("peg");

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
  for(uint i=0;i<numSamples_height;i++){
      double sampled_height = .0 + 0.2*rnd.gauss();  //height INFORMATION
      //cout<< sampled_height <<endl;
      samples[index](0) = sampled_height;

      double best = 0.;

      for(uint j=0;j<numSamples_pos;j++){
          samples[index](0) = sampled_height;
          samples[index](1) = .0 + .3*rnd.gauss(); //uncertain about x-corr of target (hole)
          if(fabs(samples[index](1)) > fabs(best))
              best = samples[index](1);
          index ++;
      }
      samples[index](0) = sampled_height;
      if(best >0)
          samples[index](1) = 1.0; //1.4 is the length (in x-axis) of the table
      else
          samples[index](1) = -1.0; //1.4 is the length (in x-axis) of the table
      index++;
  }
  //Manually testing the data
  samples[0](0) = 0.0;
  samples[0](1) = -.2;
  //samples[1](0) = 0.0;
  //samples[1](1) = 0.2;
  samples[1](0) = 0.0;
  samples[1](1) = 1.0; //pseudo datum

  samples[2](0) = 0.0;
  samples[2](1) = -0.5; //pseudo datum

  samples[3](0) = 0.5;
  samples[3](1) = 1.0;

/*/
  samples[2](0) = 0.5;
  samples[2](1) = 0.0;
  samples[3](0) = 0.5;
  samples[3](1) = 1.0;
  /*/
  //samples[2](0) = 0.5;
  //samples[2](1) = 1.6; //pseudo datum
  ////////////////////////////////////////


  cout<<"samples size = "<< samples.d0 <<endl;


  Root->AllX() = xx;
  Root->AllY() = yy;
  Root->AllDual() = ddual;
  Root->Samples() = samples;
  Root->Model() = samples[0];

  //building a FSC controller;
  FSC::Horizon = T;
  OptimizeFSC(world, Root, 0);
  //OptimizeFSC_Test(world, Root, 0);
  //write the FSC controller to .dot file
  write_to_graphviz(fsc);


  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;



  //write(fsc);

  //exit(1);




  //TEST POMDP
    orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

    //There will be two uncertain factors: height (z-axis) and position (only x-axis)

    for(uint i=0;i<10;i++){
        //table with changing heights
      world.getBodyByName("hole")->X.pos.z   = 0.0;// .0 + 0.1*rnd.gauss();
      world.getBodyByName("target")->X.pos.z = 0.5;// .0 + 0.1*rnd.gauss();


      world.getBodyByName("hole")->X.pos.x   = 0.0;// .0 + 0.1*rnd.gauss();
      world.getBodyByName("target")->X.pos.x = 0.0;// .0 + 0.1*rnd.gauss();


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

/*/

  arr x, y, dual;
  double height = 0.0;
  arr x0;
  bool stickyness = true;


  //initial prior: position 2e2; collision (0.04; 0.5e1)
  //world.setJointState(ARR(-0.130734, 0.200912, -0.0176154, -0.541507, -0.743056, -0.415109, -0.0146599 ));
  world.setJointState(ARR(-0.305148, 0.121278, -0.0600021, -0.638602, -0.87011, -0.490926, -0.0387221));

  //getTrajectory(x, y, dual, world, x0, height, stickyness, T);

 // x >> FILE("x.dat");
 //y >> FILE("y.dat");
 // dual >> FILE("dual.dat");

  x << FILE("x.dat");
  y << FILE("y.dat");
  dual << FILE("dual.dat");

  cout<<dual<<endl;



 // for(int t=160;t<dual.d0;t++)
 //     dual(t) = 0.0;
  /*/



  /*/

  for(int k=0;k<5;k++){
      //table with changing heights
      //world.getBodyByName("hole")->X.pos.z = .0 + 0.1*rnd.gauss();
      //table with changing position: for this (TODO: change target in ors file)
      //world.getBodyByName("hole")->X.pos.x =   .0 + 0.2*rnd.gauss();
      //world.getBodyByName("hole")->X.pos.y = -1.0 + 0.2*rnd.gauss();

      for(uint t=0;t<x.d0;t++){

        world.setJointState(x[t]);
        //world.stepSwift();

        world.gl().update(STRING(t), true, false, true);

      }
  }


/*/



  return 0;

}
