#include <motion/motion.h>
#include <hardware/hardware.h>
#include <biros/biros_views.h>

#include "behaviors.h"

int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  
  // variables
  GeometricState geometricState;
  MotionFuture motions;
  HardwareReference hardwareReference;
  
  // processes
  Process* ctrl = newMotionController(&hardwareReference, NULL, &motions);
  newActionProgressor(motions);
 
  new OrsView(geometricState.ors, &geometricState.rwlock);
  new PoseView(hardwareReference.q_reference, &hardwareReference.rwlock);
  new InsideOut();

  ctrl->threadLoopWithBeat(.01);

  //pick-and-place loop
  for(uint k=0;k<2;k++){
    pickOrPlaceObject(Action::grasp, "thing2", NULL);
    pickOrPlaceObject(Action::place, "thing2", "thing1");


    pickOrPlaceObject(Action::grasp, "thing1", NULL);
    pickOrPlaceObject(Action::place, "thing1", "thing2");
  }

  MT::wait(1300);
  ctrl->threadClose();

  cout <<"bye bye" <<endl;
  return 0;
}
