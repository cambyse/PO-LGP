#include <motion/motion.h>
#include <perception/perception.h>
#include <hardware/hardware.h>


/* What doesn't work yet:
 
 - collisions with the grasped object UNTIL the 4/5 time using a special proxy variable
 - feedback tasks (like open hand) have not termination criterion - fixed time is not ok!
*/

#include "behaviors.h"


struct GraspMotionParameters{
  double positionPrec;
  double oppositionPrec;
  double alignmentPrec;
  double fingerDistPrec;
  double colPrec;
  double limPrec;
  double zeroQPrec;
};

GraspMotionParameters graspMotionParameters;

void getGraspMotionParameters(){
  graspMotionParameters.positionPrec = biros().getParameter<double>("graspPlanPositionPrec");
  graspMotionParameters.oppositionPrec = biros().getParameter<double>("graspPlanOppositionPrec");
  graspMotionParameters.alignmentPrec = biros().getParameter<double>("graspPlanAlignmentPrec");
  graspMotionParameters.fingerDistPrec = biros().getParameter<double>("graspPlanFingerDistPrec");
  graspMotionParameters.colPrec = biros().getParameter<double>("graspPlanColPrec");
  graspMotionParameters.limPrec = biros().getParameter<double>("graspPlanLimPrec");
  graspMotionParameters.zeroQPrec = biros().getParameter<double>("graspPlanZeroQPrec");
}
void printGraspMotionParameters(){
  printf("GRASPMOTION PARAMETERS ------------------\n");
  printf("graspPlanPositionPrec = %f\n",graspMotionParameters.positionPrec);
  printf("graspPlanOppositionPrec = %f\n",graspMotionParameters.oppositionPrec);
  printf("graspPlanAlignmentPrec = %f\n", graspMotionParameters.alignmentPrec);
  printf("graspPlanFingerDistPrec = %f\n", graspMotionParameters.fingerDistPrec);
  printf("graspPlanColPrec = %f\n",graspMotionParameters.colPrec);
  printf("graspPlanLimPrec = %f\n", graspMotionParameters.limPrec);
  printf("graspPlanZeroQPrec = %f\n", graspMotionParameters.zeroQPrec);
fflush(stdout);
}

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
  MotionController controller;
  ActionProgressor actionProgressor;
  
  // viewers
  OrsViewer<GeometricState>     view0(geometricState);
  PoseViewer<HardwareReference> view8(hardwareReference);

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
  // viewers
  ImageViewer<Image> view1(camL), view2(camR);
  ImageViewer<Image> view3(hsvL), view4(hsvR);
  ImageViewer<FloatImage> view5(hsvEviL), view6(hsvEviR);

  P.append(LIST<Process>(controller));
  //P.append(LIST<Process>(joystick, schunkArm, schunkHand, schunkSkin));
  //P.append(LIST<Process>(cvtHsv1, cvtHsv2, hsvFilterL, hsvFilterR, shapeFitter));

  //views don't need to be started -- they now listen!
  ProcessL PV;
  //PV.append(LIST<Process>(view0));
  //PV.append(LIST<Process>(view));
  PV.append(LIST<Process>(view8));
  //PV.append(LIST<Process>(view1, view2, view5, view6)); //view3, view4, 
  
  //step(PV);
  loopWithBeat(PV,.1);

  //cam.threadLoop();
  loopWithBeat(P,.01);

  actionProgressor.threadLoopWithBeat(0.01);
  
  
  getGraspMotionParameters();
  printGraspMotionParameters();

  MT::wait(1.);

  double threshold = 0.0000001;
  double step = 0.1;

  //objects box1,box2,cyl1,cyl2,table
  double oldPosPrec = 0;
  double newPosPrec = 100;
  FILE *logger = fopen("graspprec.test", "w");
  fprintf(logger, "oldprec | newprec | costMean | costVar\n");
  while( fabs(oldPosPrec - newPosPrec) > threshold ){
    oldPosPrec = newPosPrec;
    //pickOrPlaceObject(Action::grasp, "box1", NULL);

    biros().setParameter<double>("graspPlanPositionPrec", oldPosPrec);

    //average the cost over Ntrials
    uint Ntrials = 1;
    arr pcostL(Ntrials);
    for( uint i=0;i<Ntrials;i++){
      pcostL.append(reach2("box1", NULL));
      homing2("box1", "table");
    }
    double pcostMean = mean(pcostL);
    double pcostDev = var(pcostL);

    //reset the robot hand (TODO: segfault, do it manually?)
    //homing();

    newPosPrec = oldPosPrec - step * pcostMean;
    if(newPosPrec < 0) newPosPrec = 0;
    cout << "oldPrec: " << oldPosPrec << endl;
    cout << "newPrec: " << newPosPrec << endl;
    cout << "old-new: " << fabs(oldPosPrec - newPosPrec) << endl;
    fprintf(logger, "%.2f %.2f %.2f %.2f\n", oldPosPrec, newPosPrec, pcostMean, pcostDev);

  }
  cout << "Best local optima for graspPlanPosPrec: " << newPosPrec << endl;
  fclose(logger);

  cam.threadClose();
  close(P);

  //biros().dump();
  return 0;
}
