#include <motion/motion.h>
#include <hardware/hardware.h>
//#include <views/views.h>
#include <MT/gtk.h>
#include <biros/control.h>

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);

  // variables
  GeometricState geometricState;
  Action action;
  MotionPrimitive motionPrimitive;
  MotionKeyframe frame0,frame1;
  HardwareReference hardwareReference;

  // processes
  Process *ctrl = newMotionController(&hardwareReference, &motionPrimitive, NULL);
  Process *planner = newMotionPlanner(action, frame0, frame1, motionPrimitive);

//   b::dump();  MT::wait();
  b::openInsideOut();
  
  cout <<"** setting grasp action" <<endl;
  action.writeAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = (char*)"target1";
  action.executed = false;
  action.deAccess(NULL);

  cout <<"** setting controller to follow" <<endl;
  motionPrimitive.set_mode(MotionPrimitive::followPlan, NULL);

  uint mode=2;
  switch(mode){
  case 1:{ //serial mode
    planner->open();
    planner->step();
  } break;
  case 2:{ //threaded mode
    ctrl->threadLoopWithBeat(.01);
    //step(P);
    //controller.threadLoopWithBeat(.01);
    MT::wait();
  } break;
  }

  close(birosInfo().processes);
  birosInfo().dump();
  
  cout <<"bye bye" <<endl;
};



