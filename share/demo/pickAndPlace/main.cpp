#include <motion/motion.h>
#include <perception/perception.h>
#include <hardware/hardware.h>

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
  Action action;
  MotionPrimitive motionPrimitive;
  MotionKeyframe frame0,frame1;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;
  // processes
  Controller controller;
  //MotionPrimitivener motionPlanner;
  ActionToMotionPrimitive actionToMotionPrimitive(action, frame0, frame1, motionPrimitive);
  // viewers
  OrsViewer<GeometricState>     view0(geometricState);
  PoseViewer<MotionPrimitive>        view7(motionPrimitive);
  PoseViewer<HardwareReference> view8(hardwareReference);
  PoseViewer<MotionKeyframe>    view9(frame1);

  //-- hardware
  // variables
  // processes
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;
  // viewers
  PoseViewer<HardwareReference> view(hardwareReference);

  
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
  // viewers
  ImageViewer<Image> view1(camL), view2(camR);
  ImageViewer<Image> view3(hsvL), view4(hsvR);
  ImageViewer<FloatImage> view5(hsvEviL), view6(hsvEviR);

  P.append(LIST<Process>(controller, actionToMotionPrimitive));
  //P.append(LIST<Process>(joystick, schunkArm, schunkHand, schunkSkin));
  //P.append(LIST<Process>(cvtHsv1, cvtHsv2, hsvFilterL, hsvFilterR, shapeFitter));

  //views don't need to be started -- they now listen!
  ProcessL PV;
  PV.append(LIST<Process>(view0));
  //PV.append(LIST<Process>(view));
  PV.append(LIST<Process>(view7, view8, view9));
  //PV.append(LIST<Process>(view1, view2, view5, view6)); //view3, view4, 
  
  //step(PV);
  loopWithBeat(PV,.1);

  //cam.threadLoop();
  loopWithBeat(P,.01);
  
  cout <<"arrange your windows..." <<endl;
  MT::wait(1.);
  
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
