#include <ros/ros.h>
#include <ros/service_client.h>
#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <marc_controller_pkg/SetPosTarget.h>
#include <marc_controller_pkg/GetJointState.h>
#include <Gui/opengl.h>
#include <marc_controller_pkg/JointState.h>
#include <Core/array-vector.h>
#include <Motion/pr2_heuristics.h>

struct Pr2Listener:Module{
  ACCESS(arr, q);
  ACCESS(arr, qdot);

  ros::Subscriber sub;

  Pr2Listener():Module("Pr2Listener"){
  }

  void pr2listener(const marc_controller_pkg::JointState::ConstPtr& msg){
    q.set() = ARRAY(msg->q);
    qdot.set() = ARRAY(msg->qd);
  }

  void open(){
    ros::NodeHandle nh;
    sub = nh.subscribe("/tree_rt_controller/jointState", 1, &Pr2Listener::pr2listener, this);
  }

  void step(){
    ros::spin();
  }

  void close(){
  }
};

void rosCheckInit(){
  ros::init(MT::argc, MT::argv, "pr2_module");

};

struct MySystem:System{
  ACCESS(arr, q);
  ACCESS(arr, qdot);
  ACCESS(arr, joystickState);
  MySystem(){
    addModule<JoystickInterface>(NULL, ModuleThread::loopWithBeat, .01);
    addModule<Pr2Listener>(NULL, ModuleThread::loopFull);
    connect();
  }
};


int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  rosCheckInit();

  ros::NodeHandle nh;

  ors::KinematicWorld world("scene");
  makeConvexHulls(world.shapes);

  FeedbackMotionControl MP(world, false);
  MP.nullSpacePD.active=false;
  MP.H_rate_diag = .001* pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);
  MySystem S;
  engine().open(S);

  marc_controller_pkg::SetPosTarget setPosTargetSrv;
  ros::ServiceClient setPosTargetClient;
  setPosTargetClient = nh.serviceClient<marc_controller_pkg::SetPosTarget>("/tree_rt_controller/set_pos_target",true);
  setPosTargetClient.waitForExistence();
  setPosTargetSrv.request.pos.resize(3);
  setPosTargetSrv.request.vel.resize(3);

  ros::Publisher jointReference_publisher;
  jointReference_publisher = nh.advertise<marc_controller_pkg::JointState>("/tree_rt_controller/jointReference", 1);

  for(;;){
    S.joystickState.var->waitForNextWriteAccess();
    arr joy = S.joystickState.get();
    arr q = S.q.get();
    arr qdot = S.qdot.get();
    if(q.N==7 && qdot.N==7){
      MP.setState(q, qdot);
      MP.world.gl().update();
    }

    //cout <<S.q.get()() <<endl;
    bool shutdown = j2t.updateTasks(joy,0.01);
    if(shutdown) engine().shutdown.incrementValue();

    if(q.N && qdot.N==q.N){
      for(uint tt=0;tt<10;tt++){
        arr a = MP.operationalSpaceControl();
        q += .001*qdot;
        qdot += .001*a;
      }
      marc_controller_pkg::JointState jointRef;
      jointRef.N = q.d0;
      jointRef.q = VECTOR(q);
      jointRef.qd = VECTOR(qdot);
      jointReference_publisher.publish(jointRef);
    }

//    if (MP.tasks(0)->active){
//      for(uint i=0;i<3;i++) setPosTargetSrv.request.pos[i] = MP.tasks(0)->y_ref(i);
//      for(uint i=0;i<3;i++) setPosTargetSrv.request.vel[i] = MP.tasks(0)->v_ref(i);
//      //cout << "y_ref: " << MP.tasks(0)->y_ref << endl;
//      //cout << "v_ref: " << MP.tasks(0)->v_ref << endl;
//      setPosTargetClient.call(setPosTargetSrv);
//    }

    if(engine().shutdown.getValue() || !ros::ok()) break;
  }

  engine().close(S);

  return 0;
}
