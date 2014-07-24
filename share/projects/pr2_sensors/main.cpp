#include <Motion/feedbackControl.h>
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

#include "../pcl_sprint_projections/dataStructures.h"

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(arr, joystickState)

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
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
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
    connect();
  }
};

void testSensors(){

  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  ors::Joint *trans=world.getJointByName("worldTranslationRotation");
  ors::Shape *ftL_shape=world.getShapeByName("endeffL");

  FeedbackMotionControl MP(world, true);
  MP.qitselfPD.y_ref = q;
  MP.H_rate_diag = pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);

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

  if(MT::getParameter<bool>("useRos", false)){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      S.ctrl_obs.var->waitForNextRevision();
      if(S.ctrl_obs.get()->q.N==MP.world.q.N
         && S.ctrl_obs.get()->qdot.N==MP.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S.ctrl_obs.get()->q;
    qdot = S.ctrl_obs.get()->qdot;
    //arr fL_base = S.fL_obs.get();
    MP.setState(q, qdot);
  }
  arr zero_qdot(qdot.N);
  zero_qdot.setZero();
  CtrlMsg refs;

  for(uint t=0;;t++){
    S.joystickState.var->waitForNextRevision();
    arr joypadState = S.joystickState.get();
    bool shutdown = j2t.updateTasks(joypadState);
    if(t>10 && shutdown) engine().shutdown.incrementValue();

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
    gl.update();

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

    //compute control
    arr a = MP.operationalSpaceControl();
    q += .01*qdot;
    qdot += .01*a;
//    MP.reportCurrentState();
    MP.setState(q, qdot);
    //MP.world.reportProxies();
    if(!(t%4))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    //-- force task
    uint mode = 0;
    if(joypadState.N) mode = uint(joypadState(0));
    if(mode==2){
      arr y_fL, J_fL;
      MP.world.kinematicsPos(y_fL, J_fL, ftL_shape->body, &ftL_shape->rel.pos);
      cout <<"FORCE TASK" <<endl;
      refs.fL = ARR(10., 0., 0.);
      J_fL = J_fL.sub(0,1,0,-1);
      arr gain = 10.*(~J_fL*J_fL) + .3*eye(q.N);
      cout <<J_fL <<gain <<endl;
//      refs.u_bias = 1.*(~J_fL * refs.fL);
      refs.Kq_gainFactor = gain;
//      refs.Kq_gainFactor = ARR(.3);
      refs.u_bias = zeros(q.N);
    }else{
      refs.fL = ARR(0., 0., 0.);
      refs.Kq_gainFactor = ARR(1.);
      refs.u_bias = zeros(q.N);
    }

    refs.Kd_gainFactor = ARR(1.);
    refs.q=q;
    refs.qdot=zero_qdot;
    if(trans && trans->qDim()==3){
      refs.qdot(trans->qIndex+0) = qdot(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot(trans->qIndex+2);
      if(true){ //no translations!
        refs.qdot(trans->qIndex+0) = 0.;
        refs.qdot(trans->qIndex+1) = 0.;
        refs.qdot(trans->qIndex+2) = 0.;
      }
    }
    S.ctrl_ref.set() = refs;

    if(engine().shutdown.getValue()/* || !rosOk()*/) break;
  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testSensors();

  return 0;
}
