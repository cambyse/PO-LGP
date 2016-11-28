#include <Control/taskController.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <RosCom/roscom.h>
#include <Control/gamepad2tasks.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>

#include "pr2GamepadController.h"

struct MySystem {
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(arr, gamepadState)

  ACCESSname(byteA, kinect_rgb)
  ACCESSname(uint16A, kinect_depth)
  ACCESSname(mlr::Transformation, kinect_frame)

  ACCESSname(arr, kinect_points)
  ACCESSname(arr, kinect_pointColors)
  ACCESSname(arr, kinect_points_world)

  ACCESSname(arr, wrenchL)
  ACCESSname(arr, wrenchR)
  ACCESSname(byteA, rgb_leftEye)
  ACCESSname(byteA, rgb_rightEye)
  ACCESSname(byteA, rgb_leftArm)
  ACCESSname(byteA, rgb_rightArm)

  GamepadInterface gamepad;

  MySystem(){
    if(mlr::getParameter<bool>("useRos", true)){
      new RosCom_Spinner();
      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
      new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame);
//      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/wide_stereo/left/image_rect_color", rgb_leftEye);
//      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/wide_stereo/right/image_rect_color", rgb_rightEye);
//      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/l_forearm_cam/image_rect_color", rgb_leftArm);
//      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/r_forearm_cam/image_rect_color", rgb_rightArm);
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new SubscriberConv<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr>("/ft_sensor/ft_compensated", wrenchL);

    }
    new KinectDepthPacking();
    new ImageViewer("kinect_rgb");
    new ImageViewer("kinect_depthRgb");
//    new ImageViewer("rgb_leftArm");
//    new ImageViewer("rgb_rightArm");
//    new ImageViewer("rgb_leftEye");
//    new ImageViewer("rgb_rightEye");
    new Kinect2PointCloud();
    new PointCloudViewer();
//    addModule<Pr2GamepadController>(NULL, /*Module::loopWithBeat,*/ .01);
    cout <<"SYSTEM=" <<registry() <<endl;

  }
};

void TEST(Sensors){

  MySystem S;

  mlr::KinematicWorld world("model.kvg");
  OpenGL gl;
  gl.setClearColors(1., 1., 1., 1.);
  gl.camera.setPosition(10., -15., 8.);
  gl.camera.focus(0, 0, 1.);
  gl.camera.upright();

//  gl.camera = kinectCam;
  gl.add(glStandardScene, NULL);
  gl.add(mlr::glDrawGraph, &world);
//  primitives.G.init("model.kvg");
//  mlr::Shape *kinShape = primitives.G.getShapeByName("endeffKinect");
//  mlr::Shape *wrenchDispL = primitives.G.getShapeByName("wrenchDispL");
//  mlr::Shape *wrenchDispR = primitives.G.getShapeByName("wrenchDispR");
//  gl.add(glDrawPrimitives, &primitives);
  gl.update();
  gl.lock.writeLock();
//  primitives.P.append(new ArrCloudView(S.kinect_points_world, S.kinect_pointColors));
  gl.lock.unlock();

  threadOpenModules(true);

  for(uint t=0;;t++){
    arr gamepadState = S.gamepadState.get();
    if(t>10 && stopButtons(gamepadState)) moduleShutdown().incrementValue();
    if(moduleShutdown().getValue()>0) break;
    S.gamepadState.var->waitForNextRevision();

    //-- update world
    gl.lock.writeLock();
    // joint sensors
    arr q_obs    = S.ctrl_obs.get()->q;
    arr qdot_obs = S.ctrl_obs.get()->qdot;
    if(q_obs.N==world.q.N){
      world.setJointState(q_obs, qdot_obs);
    }else{
      LOG(0) <<"joint dim unequal: " <<q_obs.N <<' ' <<world.q.N;
    }

    //kinect point cloud
    if(S.kinect_points.get()->N){
      mlr::Shape *s = world.getShapeByName("kinectCloud");
      if(!s){
        s = new mlr::Shape(world, NoBody);
        s->name="kinectCloud";
        s->type = mlr::meshST;
      }
      s->mesh.V = S.kinect_points.get();
      s->mesh.C = S.kinect_pointColors.get();
      s->X = s->rel = S.kinect_frame.get();
    }

    gl.lock.unlock();

    gl.lock.writeLock();
    gl.lock.unlock();
    if(!(t%10)) gl.update();

    // force sensors
    arr wL = S.wrenchL.get()();
    arr wR = S.wrenchR.get()();
    cout <<"WRENCHES= " <<wL <<wR  <<endl;
    if(wL.N==6 && wR.N==6){
      mlr::Quaternion rot, tmp;
      rot.setDeg(-90, 0, 1, 0);
      tmp.setDeg(90, 0, 0, 1);
      rot = rot*tmp;
//      wrenchDispR->rel.pos = mlr::Vector(.1,0,0) - .01 * (rot * mlr::Vector(wR.sub(0,2)));
//      wrenchDispR->rel.rot.setVec(-1.*(rot*mlr::Vector(wR.sub(3,-1))));;
//      wrenchDispL->rel.pos = mlr::Vector(.1,0,0) - .01 * (rot * mlr::Vector(wL.sub(0,2)));
//      wrenchDispL->rel.rot.setVec(-1.*(rot*mlr::Vector(wL.sub(3,-1))));;
    }
  }

  threadCloseModules();
  modulesReportCycleTimes();
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  rosCheckInit("pr2_sensors");
  testSensors();

  return 0;
}
