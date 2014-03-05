#include <ros/ros.h>
#include <ros/service_client.h>
#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <tree_controller_pkg/SetPosTarget.h>
#include <tree_controller_pkg/GetJointState.h>
#include <Gui/opengl.h>
#include <tree_controller_pkg/JointState.h>




struct Pr2Listener:Module{
  ACCESS(arr, q);
  ros::Subscriber sub;
  Pr2Listener():Module("Pr2Listener"){
  }
void pr2listener(const tree_controller_pkg::JointState::ConstPtr& msg){
  cout << msg->q[0] << endl;
  q.writeAccess();
  q().resize(1);
  q()(0)=msg->q[0];
  q.deAccess();
}
  void open(){
    ros::NodeHandle nh;
    sub = nh.subscribe("/tree_rt_controller/joint_state", 1, &Pr2Listener::pr2listener, this);
  }
  void step(){
    ros::spin();
  }
  void close(){
  }

};

struct MySystem:System{
  ACCESS(arr, q);
  ACCESS(arr, joystickState);
  MySystem(){
    addModule<JoystickInterface>(NULL, ModuleThread::loopWithBeat, .01);
    addModule<Pr2Listener>(NULL, ModuleThread::loopFull);
    connect();
  }
};


int main(int argc, char** argv)
{
  /// Init ros
  ros::init(argc, argv, "pr2_module");
  ros::NodeHandle nh;

  /// Init ors
  MT::initCmdLine(argc,argv);
  ors::KinematicWorld world("scene");
  //  world.init(STRING("scene"));
  makeConvexHulls(world.shapes);

  FeedbackMotionControl MP(world, false);
  Gamepad2Tasks j2t(MP);
  MySystem S;
  engine().open(S);



  tree_controller_pkg::SetPosTarget setPosTargetSrv;
  ros::ServiceClient setPosTargetClient;
  setPosTargetClient = nh.serviceClient<tree_controller_pkg::SetPosTarget>("/tree_rt_controller/set_pos_target",true);
  setPosTargetClient.waitForExistence();
  setPosTargetSrv.request.pos.resize(3);
  setPosTargetSrv.request.vel.resize(3);

  tree_controller_pkg::GetJointState getJointStateSrv;
  ros::ServiceClient getJointStateClient;
  getJointStateClient = nh.serviceClient<tree_controller_pkg::GetJointState>("/tree_rt_controller/get_joint_state",true);
  setPosTargetClient.waitForExistence();
  arr q(7);
  arr qd(7);


  for(;;){
    S.joystickState.var->waitForNextWriteAccess();
    arr joy = S.joystickState.get();
    ros::spinOnce();
    // Get current joint state
    getJointStateClient.call(getJointStateSrv);
    for(uint i=0;i<7;i++) q(i) = getJointStateSrv.response.q[i];
    for(uint i=0;i<7;i++) qd(i) = getJointStateSrv.response.qd[i];
    cout << "q: " << q << endl;
    MP.setState(q,qd);
//    MP.world.gl().update();

    cout <<S.q.get()() <<endl;
    bool shutdown = j2t.updateTasks(joy,0.01);
    if(shutdown) engine().shutdown.incrementValue();

    if (MP.tasks(0)->active){
      for(uint i=0;i<3;i++) setPosTargetSrv.request.pos[i] = MP.tasks(0)->y_ref(i);
      for(uint i=0;i<3;i++) setPosTargetSrv.request.vel[i] = MP.tasks(0)->v_ref(i);
      cout << "y_ref: " << MP.tasks(0)->y_ref << endl;
      cout << "v_ref: " << MP.tasks(0)->v_ref << endl;
      setPosTargetClient.call(setPosTargetSrv);
    }

    if(engine().shutdown.getValue() || !ros::ok()) break;
  }

  engine().close(S);

  return 0;
}
