#include <motion/motion.h>
#include <perception/perception.h>
#include <hardware/hardware.h>

/* What doesn't work yet:
 
 - collisions with grasped objects are not turned on again?
 - feedback tasks (like open hand) have not termination criterion - fixed time is not ok!
*/

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
  MotionPrimitive motionPrimitive(action, frame0, frame1, motionPlan);
  // viewers
  OrsViewer<GeometricState>     view0(geometricState);
  PoseViewer<MotionPlan>        view7(motionPlan);
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

  P.append(LIST<Process>(controller, motionPlanner, motionPrimitive));
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
    pickObject("box1");
    placeObject("box1", "table", "cyl1");

    pickObject("box2");
    placeObject("box2", "table", "cyl2");
    
    pickObject("box1");
    placeObject("box1", "cyl1", "table");

    pickObject("box2");
    placeObject("box2", "cyl2", "cyl1");
    
    pickObject("box1");
    placeObject("box1", "table", "cyl2");

    pickObject("box2");
    placeObject("box2", "cyl1", "table");
    
  }
  
  cam.threadClose();
  close(P);

  birosInfo.dump();
  cout <<"bye bye" <<endl;
  return 0;
}
