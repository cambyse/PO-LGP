#define MT_IMPLEMENTATION
#define MT_algos_extern

#include<MT/ors.h>

void compareModules(){
  ors::Graph ors;
  MT::load(ors,"../../configurations/schunk.ors",true);
  ors.calcBodyFramesFromJoints();

  OpenGL gl;
  gl.add(ors::glDrawGraph,&ors);
  //gl.watch();
  
  SwiftInterface swift;
  swift.cutoff=1.;
  swift.init(ors);

  arr q0,q;
  ors.getJointState(q0);
  MT::timerStart();
  uint t;
  for(t=0;t<10000;t++){
    if(!(t%1)){ q = q0;  rndGauss(q,.1,true); }
    ors.setJointState(q);
    ors.calcBodyFramesFromJoints();
    swift.computeProxies(ors,false);
    //ors.reportProxies();
    //gl.update();
    //gl.watch();
  }
  cout <<t <<" collision queries: sec=" <<MT::timerRead() <<endl;
}


int main(int argc,char **argv){
  compareModules();
  
  return 0;
}
