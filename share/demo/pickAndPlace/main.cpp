#include <motion/motion.h>
#include <perception/perception.h>
#include <hardware/hardware.h>

#include <biros/biros_views.h>
#include <biros/control.h>
#include <MT/ors.h>

/* What doesn't work yet:
 
 - collisions with the grasped object UNTIL the 4/5 time using a special proxy variable
 - feedback tasks (like open hand) have not termination criterion - fixed time is not ok!
*/

#include "behaviors.h"

int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);
  
  ProcessL P;

  //-- motion
  // variables
  GeometricState geometricState;
  MotionFuture motions;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;
  
  // processes
  Process* ctrl = newMotionController(&hardwareReference, NULL, &motions);
  newActionProgressor(motions);
  
  //-- hardware
  // variables
  //(none)
  // processes
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;
  // viewers
  //(none)

  //-- perception
  // variables
  Image camL("CameraL"), camR("CameraR");
  Image hsvL("HsvL"), hsvR("HsvR");
  FloatImage hsvEviL("hsvEviL"), hsvEviR("hsvEviR");
  PerceptionOutput percOut;
  // processes
  Camera cam;
  CvtHsv cvtHsv1(camL, hsvL);
  CvtHsv cvtHsv2(camR, hsvR);
  HsvFilter hsvFilterL(hsvL, hsvEviL);
  HsvFilter hsvFilterR(hsvR, hsvEviR);
  ShapeFitter shapeFitter(hsvEviL, hsvEviR, percOut);

  /////////////////////////////////////////////////////////////////////////////
  // inside-out

  b::dump();
  b::openInsideOut();

  ctrl->threadLoopWithBeat(.01);
  
  //pick-and-place loop
  for(uint k=0;k<2;k++){
    pickOrPlaceObject(Action::grasp, "box1", NULL);
    pickOrPlaceObject(Action::place, "box1", "cyl1");

    pickOrPlaceObject(Action::grasp, "box2", NULL);
    pickOrPlaceObject(Action::place, "box2", "cyl2");
    
    pickOrPlaceObject(Action::grasp, "box1", NULL);
    pickOrPlaceObject(Action::place, "box1", "table");

    pickOrPlaceObject(Action::grasp, "box2", NULL);
    pickOrPlaceObject(Action::place, "box2", "cyl1");
    
    pickOrPlaceObject(Action::grasp, "box1", NULL);
    pickOrPlaceObject(Action::place, "box1", "cyl2");

    pickOrPlaceObject(Action::grasp, "box2", NULL);
    pickOrPlaceObject(Action::place, "box2", "table");
  }
  
  cam.threadClose();
  close(P);

  birosInfo.dump();
  cout <<"bye bye" <<endl;
  return 0;
}
