#include <motion/motion.h>
#include <perception/perception.h>
#include <hardware/hardware.h>

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
  MotionPlan motionPlan;
  MotionKeyframe frame0,frame1;
  ControllerTask controllerTask;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;
  // processes
  Controller controller;
  MotionPlanner motionPlanner;
  MotionPrimitive motionPrimitive(action, frame0, frame1, motionPlan, geometricState);
  // viewers
  OrsViewer<GeometricState>     view0(geometricState, geometricState);
  PoseViewer<MotionPlan>        view7(motionPlan, geometricState);
  PoseViewer<HardwareReference> view8(hardwareReference, geometricState);
  PoseViewer<MotionKeyframe>    view9(frame1, geometricState);

  //-- hardware
  // variables
  // processes
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;
  // viewers
  PoseViewer<HardwareReference> view(hardwareReference, geometricState);

  
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

  P.append(LIST<Process>(controller, motionPlanner, motionPrimitive));
  P.append(LIST<Process>(joystick, schunkArm, schunkHand, schunkSkin));
  P.append(LIST<Process>(cvtHsv1, cvtHsv2, hsvFilterL, hsvFilterR, shapeFitter));
  P.append(LIST<Process>(view0));
  //P.append(LIST<Process>(view));
  //P.append(LIST<Process>(view7, view8, view9));
  P.append(LIST<Process>(view1, view2, view5, view6)); //view3, view4, 

  cam.threadLoop();
  loopWithBeat(P,.01);
  
  //pick-and-place loop
  for(uint k=0;k<10;k++){
    waitForPerceivedObjects(1, 1);
    pickObject("cyl1");
    //resetPlanner(planner);
    placeObject("cyl1", "table", "cyl2");
    //resetPlanner(planner);
    //plannedHoming("cyl1", "cyl2");
    //resetPlanner(planner);
    cout <<"DDOONNEE!" <<endl;
  }
  
  cam.threadClose();
  close(P);

  birosInfo.dump();
  cout <<"bye bye" <<endl;
  return 0;
}
