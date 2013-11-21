#include <Core/util.h>
#include <Motion/motion.h>
#include <Gui/opengl.h>
#include "dmp.h"
#include <vector>
#include <GL/glu.h>
#include <stdlib.h>

const double PI = 3.1415926535897932384626;

int main(int argc,char **argv){
  //  Create some test data
  arr traj;
  uint i;
  double n = 100;
  for (i=0;i<n;i++) {
    traj.append(~ARRAY(sin(i/n*PI/2*3),2+cos(PI*i/n)));
  }

  // Create DMP
  DMP d(traj, 99, 0.01);
//  d.trainDMP(3.,ARRAY(2.,2.));
  d.trainDMP();

  for (i=0;i<100;i++) {
    d.iterate();
  }

  // Plot DMP
  d.printDMP();
  d.plotDMP();
  return 0;
}
