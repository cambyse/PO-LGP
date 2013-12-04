#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Gui/opengl.h>

void compareModules(){
  ors::KinematicWorld G("../../../configurations/schunk.ors");

  G.swift().setCutoff(1.);

  arr q0,q;
  G.getJointState(q0);
  MT::timerStart();
  uint t;
  for(t=0;t<1000;t++){
    if(!(t%1)){ q = q0;  rndGauss(q,.1,true); }
    G.setJointState(q);
    G.calcBodyFramesFromJoints();
    G.swift().computeProxies(G,false);
//    G.reportProxies();
//    G.gl().update();
//    G.gl().watch();
  }
  cout <<t <<" collision queries: sec=" <<MT::timerRead() <<endl;
}


int main(int argc,char **argv){
  compareModules();
  
  return 0;
}
