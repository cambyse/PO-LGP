#include <Motion/feedbackControl.h>
#include <System/engine.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <Motion/gamepad2tasks.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>

#include "../pcl_sprint_projections/dataStructures.h"
#include "pr2GamepadController.h"

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(arr, gamepadState)

  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)

  ACCESS(arr, wrenchL)
  ACCESS(arr, wrenchR)
  ACCESS(byteA, rgb_leftEye)
  ACCESS(byteA, rgb_rightEye)
  ACCESS(byteA, rgb_leftArm)
  ACCESS(byteA, rgb_rightArm)

  MySystem(){
    addModule<GamepadInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos", true)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_KinectSync>(NULL, Module_Thread::loopWithBeat, 1.);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
      addModule<RosCom_ForceSensorSync>(NULL, Module_Thread::loopWithBeat, 1.);
      addModule<RosCom_CamsSync>(NULL, Module_Thread::loopWithBeat, 1.);
      addModule<RosCom_ArmCamsSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
//    addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
//    addModule<ImageViewer>("ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("rgb_leftArm"), Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("rgb_rightArm"), Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("rgb_leftEye"), Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("rgb_rightEye"), Module_Thread::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module_Thread::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);
    addModule<Pr2GamepadController>(NULL, Module_Thread::loopWithBeat, .01);
    connect();
  }
};

void testSensors(){


  MySystem S;

  DisplayPrimitives primitives;
  OpenGL gl;
  gl.camera = kinectCam;
  gl.add(glStandardScene, NULL);
  primitives.G.init("model.kvg");
  ors::Shape *kinShape = primitives.G.getShapeByName("endeffKinect");
  ors::Shape *wrenchDispL = primitives.G.getShapeByName("wrenchDispL");
  ors::Shape *wrenchDispR = primitives.G.getShapeByName("wrenchDispR");
  gl.add(glDrawPrimitives, &primitives);
  gl.update();
  gl.lock.writeLock();
  primitives.P.append(new ArrCloudView(S.kinect_points, S.kinect_pointColors));
  gl.lock.unlock();

  engine().open(S);

  for(uint t=0;;t++){
    arr gamepadState = S.gamepadState.get();
    if(t>10 && stopButtons(gamepadState)) engine().shutdown.incrementValue();
    if(engine().shutdown.getValue()>0) break;
    S.gamepadState.var->waitForNextRevision();

    // joint sensors
    arr q_obs    = S.ctrl_obs.get()->q;
    arr qdot_obs = S.ctrl_obs.get()->qdot;
    if(q_obs.N==primitives.G.q.N && qdot_obs.N==primitives.G.qdot.N){
      gl.lock.writeLock();
      primitives.G.setJointState(q_obs,qdot_obs);
      gl.lock.unlock();
    }else{
      cout <<"No joint signals: q_obs.N=" <<q_obs.N <<" G.q.N=" <<primitives.G.q.N <<endl;
    }
    gl.lock.writeLock();
    primitives.P(0)->X = kinShape->X;
    gl.lock.unlock();
    if(!(t%10)) gl.update();

    // force sensors
    arr wL = S.wrenchL.get()();
    arr wR = S.wrenchR.get()();
//    cout <<wL <<wR  <<endl;
    if(wL.N==6 && wR.N==6){
      ors::Quaternion rot, tmp;
      rot.setDeg(-90, 0, 1, 0);
      tmp.setDeg(90, 0, 0, 1);
      rot = rot*tmp;
      wrenchDispR->rel.pos = ors::Vector(.1,0,0) - .01 * (rot * ors::Vector(wR.sub(0,2)));
      wrenchDispR->rel.rot.setVec(-1.*(rot*ors::Vector(wR.sub(3,-1))));;
      wrenchDispL->rel.pos = ors::Vector(.1,0,0) - .01 * (rot * ors::Vector(wL.sub(0,2)));
      wrenchDispL->rel.rot.setVec(-1.*(rot*ors::Vector(wL.sub(3,-1))));;
    }
  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testSensors();

  return 0;
}
