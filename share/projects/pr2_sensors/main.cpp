#include <System/engine.h>
#include <Hardware/joystick/joystick.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <Motion/gamepad2tasks.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>


struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(arr, joystickState)
  ACCESS(byteA, kinect_rgb)
  ACCESS(MT::Array<uint16_t>, kinect_depth)
  ACCESS(arr, wrenchL)
  ACCESS(arr, wrenchR)
  ACCESS(byteA, rgb_leftArm)
  ACCESS(byteA, rgb_rightArm)

  MySystem(){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos", true)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_KinectSync>(NULL, Module_Thread::loopWithBeat, 1.);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
      addModule<RosCom_ForceSensorSync>(NULL, Module_Thread::loopWithBeat, 1.);
      addModule<RosCom_ArmCamsSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
    addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("rgb_leftArm"), Module_Thread::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", STRINGS("rgb_rightArm"), Module_Thread::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module_Thread::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);
    connect();
  }
};

void testSensors(){
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");

  MySystem S;
  engine().open(S);

  CtrlMsg refs;

  for(uint t=0;;t++){
    S.joystickState.var->waitForNextRevision();
    arr joypadState = S.joystickState.get();
    if(t>10 && stopButtons(joypadState)) engine().shutdown.incrementValue();

    // joint state
    arr q    = S.ctrl_obs.get()->q;
    arr qdot = S.ctrl_obs.get()->qdot;
    if(q.N==world.q.N && qdot.N==world.qdot.N){
      world.setJointState(q,qdot);
      world.gl().update();
    }

    // force sensors
    arr wL = S.wrenchL.get()();
    arr wR = S.wrenchR.get()();
    cout <<wL <<wR  <<endl;

    if(engine().shutdown.getValue()/* || !rosOk()*/) break;
  }

  engine().close(S);

}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testSensors();
  return 0;
}
