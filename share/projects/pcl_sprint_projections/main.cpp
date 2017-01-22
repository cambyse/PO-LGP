#include <Hardware/gamepad/gamepad.h>

#include "../pcl_sprint_module/system.h"
#include "../pcl_sprint_module/methods.h"

#include "dataStructures.h"
#include <Kin/kin.h>


struct MySystem{
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
    new GamepadInterface;
    if(mlr::getParameter<bool>("useRos", true)){
      new RosCom_Spinner();
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);

      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
      new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame);
      addModule<RosCom_ForceSensorSync>(NULL, /*Module::loopWithBeat,*/ 1.);
//      addModule<RosCom_CamsSync>(NULL, /*Module::loopWithBeat,*/ 1.);
//      addModule<RosCom_ArmCamsSync>(NULL, /*Module::loopWithBeat,*/ 1.);
    }
//    addModule<KinectDepthPacking>("KinectDepthPacking" /*,Module::listenFirst*/ );
//    new ImageViewer("kinect_rgb");
//    new ImageViewer("kinect_depthRgb");
//    new ImageViewer("rgb_leftArm");
//    new ImageViewer("rgb_rightArm");
//    new ImageViewer("rgb_leftEye");
//    new ImageViewer("rgb_rightEye");
    new Kinect2PointCloud;
    new PointCloudViewer("kinect_points", "kinect_pointColors");
    //connect();
  }
};


void TEST(Projections){



  MySystem S;

  DisplayPrimitives primitives;
  OpenGL gl;
  gl.camera = kinectCam;
  gl.add(glStandardScene, NULL);
  primitives.G.init("model.kvg");
  mlr::Shape *kinShape = primitives.G.getShapeByName("endeffKinect");
  gl.add(glDrawPrimitives, &primitives);
  gl.update();
  gl.lock.writeLock();
  primitives.P.append(new ArrCloudView(S.kinect_points, S.kinect_pointColors));
  gl.lock.unlock();

  threadOpenModules(true);

  for(uint t=0;;t++){
    arr gamepadState = S.gamepadState.get();
    if(t>10 && stopButtons(gamepadState)) moduleShutdown().incrementValue();
    if(moduleShutdown().getValue()>0) break;
    S.gamepadState.data->waitForNextRevision();

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

  threadCloseModules();
  cout <<"bye bye" <<endl;
}


int main(int argc,char **argv){
  mlr::initCmdLine(argc, argv);
  testProjections();

  return 0;
};

