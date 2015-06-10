#include <Hardware/gamepad/gamepad.h>

#include "../pcl_sprint_module/system.h"
#include "../pcl_sprint_module/methods.h"

#include "dataStructures.h"
#include <Ors/ors.h>


struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(arr, gamepadState)

  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(pcl::PointCloud<PointT>::Ptr, pcl_cloud)

  ACCESS(arr, wrenchL)
  ACCESS(arr, wrenchR)
  ACCESS(byteA, rgb_leftEye)
  ACCESS(byteA, rgb_rightEye)
  ACCESS(byteA, rgb_leftArm)
  ACCESS(byteA, rgb_rightArm)

  MySystem(){
    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos", true)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_KinectSync>(NULL, Module::loopWithBeat, 1.);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
      addModule<RosCom_ForceSensorSync>(NULL, Module::loopWithBeat, 1.);
//      addModule<RosCom_CamsSync>(NULL, Module::loopWithBeat, 1.);
//      addModule<RosCom_ArmCamsSync>(NULL, Module::loopWithBeat, 1.);
    }
//    addModule<KinectDepthPacking>("KinectDepthPacking", Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_depth", {"kinect_depthRgb"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_leftArm"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_rightArm"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_leftEye"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_rightEye"}, Module::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
    connect();
  }
};


void TEST(Projections){



  MySystem S;

  DisplayPrimitives primitives;
  OpenGL gl;
  gl.camera = kinectCam;
  gl.add(glStandardScene, NULL);
  primitives.G.init("model.kvg");
  ors::Shape *kinShape = primitives.G.getShapeByName("endeffKinect");
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

    // joint state
    arr q    = S.ctrl_obs.get()->q;
    arr qdot = S.ctrl_obs.get()->qdot;
    if(q.N==primitives.G.q.N && qdot.N==primitives.G.qdot.N){
      gl.lock.writeLock();
      primitives.G.setJointState(q,qdot);
      gl.lock.unlock();
    }else{
      cout <<"No joint signals: q.N=" <<q.N <<" G.q.N=" <<primitives.G.q.N <<endl;
    }

    gl.lock.writeLock();
    primitives.P(0)->X = kinShape->X;
    gl.lock.unlock();
    gl.update();

  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}


int main(int argc,char **argv){
  MT::initCmdLine(argc, argv);
  testProjections();

  return 0;
};
