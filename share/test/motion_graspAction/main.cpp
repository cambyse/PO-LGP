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

  //views
  new OrsView(geometricState.ors, &geometricState.rwlock);
  new PoseView(hardwareReference.q_reference, &hardwareReference.rwlock);
  new PoseView(motionPrimitive.q_plan, &hardwareReference.rwlock);
  new InsideOut();
  
  cout <<"** setting grasp action" <<endl;
  motionPrimitive.writeAccess(NULL);
  motionPrimitive.action = MotionPrimitive::grasp;
  motionPrimitive.objectRef1 = (char*)"target1";
  motionPrimitive.deAccess(NULL);

  //-- do the job!!
  ctrl->threadLoopWithBeat(.01);
  while(motionPrimitive.get_mode(NULL)!=MotionPrimitive::done)
    motionPrimitive.waitForNextWriteAccess();

  close(biros().processes);
  //biros().dump();

  cout <<"bye bye" <<endl;
};



