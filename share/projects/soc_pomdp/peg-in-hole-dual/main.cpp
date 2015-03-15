
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/feedbackControl.h>
#include <Core/util.h>



#include <Optim/optimization.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

#include <execution.h>



extern double stickyWeight;


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));

  uint T = 200; //time horizon
  MT::timerStart(true);

  arr x, y, dual;
  double height = 0.0;
  arr x0;
  bool stickyness = true;


  //initial prior: position 2e2; collision (0.04; 0.5e1)
  //world.setJointState(ARR(-0.130734, 0.200912, -0.0176154, -0.541507, -0.743056, -0.415109, -0.0146599 ));
  //world.setJointState(ARR(-0.305148, 0.121278, -0.0600021, -0.638602, -0.87011, -0.490926, -0.0387221));
  world.setJointState(ARR( -0.23225, 0.245013, -0.0470437, -0.691428, -0.869523, -0.368891, 0.0488829));

  getTrajectory(x, y, dual, world, x0, height, stickyness, T);

  x >> FILE("x.dat");
  y >> FILE("y.dat");
  dual >> FILE("dual.dat");

  x << FILE("x.dat");
  y << FILE("y.dat");
  dual << FILE("dual.dat");

  cout<< dual<<endl;


//replay of trajectory
  for(int t=0;t<x.d0;t++){
      //MT::wait(.1);
      ors::Shape *endeff = world.getShapeByName("peg");

      world.setJointState(x[t]);
      world.gl().update(STRING(t), true, false, true);

      cout<<"endeff->X.pos "<< endeff->X.pos <<endl;

     // if(t==15)
     //     cout<< x[15]<<endl;
  }


 // for(int t=160;t<dual.d0;t++)
 //     dual(t) = 0.0;
//  exit(0);

//TEST POMDP
  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

  for(uint i=0;i<10;i++){
      cout<<"time     "<<i<<endl;
      //table with changing heights
    world.getBodyByName("hole")->X.pos.z = .0;// .0 + 0.1*rnd.gauss();
    world.getBodyByName("target")->X.pos.z = .4;// .0 + 0.1*rnd.gauss();

    world.setJointState(x[0]);



    POMDPExecution(x, y, dual, world, i);
  }


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
