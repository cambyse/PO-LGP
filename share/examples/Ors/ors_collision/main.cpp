#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Gui/opengl.h>

void TEST(CollisionTiming){
  ors::KinematicWorld G("../../../configurations/schunk.ors");

  G.swift().setCutoff(1.);

  arr q0,q;
  G.getJointState(q0);
  MT::timerStart();
  uint t;
  for(t=0;t<1000;t++){
    if(!(t%1)){ q = q0;  rndGauss(q,.1,true); }
    G.setJointState(q);
    G.stepSwift();
//    G.reportProxies();
//    G.watch(false);
//    G.watch(true);
  }
  double time = MT::timerRead();
  cout <<t <<" collision queries: sec=" <<time <<endl;
  CHECK(time>0.01 && time<1.,"strange time for collision checking!");
}


int MAIN(int argc,char **argv){
  testCollisionTiming();
  
  return 0;
}
