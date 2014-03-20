#include "puppeteer.h"

void testPuppeteer(){
  Puppeteer P;
  P.open();
  P.addGroundedAction(coreTasks, NULL, NULL, NoArr, NoArr);
  P.addGroundedAction(moveEffTo, "endeffR", NULL, ARR(.5,-.4,1.2), NoArr);
  P.addGroundedAction(alignEffTo, "endeffR", NULL, ARR(1., 0., 0.), ARR(1., 0., 0.));
  P.run(5.);
  GroundedAction *a = P.addGroundedAction(pushForce, "endeffR", NULL, ARR(10., 0., 0.), NoArr);
  P.run(1.);
  P.removeGroundedAction(a);
  P.run(1.);
  P.close();
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
//  testJoypad();
  testPuppeteer();
  return 0;
}
