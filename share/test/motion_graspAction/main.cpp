#include <motion/motion.h>
#include <hardware/hardware.h>
#include <biros/biros_views.h>

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);

  // variables
  GeometricState geometricState;
  MotionPrimitive motionPrimitive;
  HardwareReference hardwareReference;

  // processes
  Process *ctrl = newMotionController(&hardwareReference, &motionPrimitive, NULL);
  Process *planner = newMotionPlanner(motionPrimitive);

//   b::dump();  MT::wait();
  new OrsView(geometricState.ors, &geometricState.rwlock);
  new PoseView(hardwareReference.q_reference, &hardwareReference.rwlock);
  new InsideOut();
  
  cout <<"** setting grasp action" <<endl;
  motionPrimitive.writeAccess(NULL);
  motionPrimitive.action = MotionPrimitive::grasp;
  motionPrimitive.objectRef1 = (char*)"target1";
  motionPrimitive.planConverged = false;
  motionPrimitive.deAccess(NULL);

  cout <<"** setting controller to follow" <<endl;
  motionPrimitive.set_mode(MotionPrimitive::planned, NULL);

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



