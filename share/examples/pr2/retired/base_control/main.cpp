#include <Motion/gamepad2tasks.h>
#include <Control/taskController.h>
#include <Hardware/gamepad/gamepad.h>
//#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <RosCom/roscom.h>
#include <RosCom/rosmacro.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Algo/spline.h>


ROSSUB("/robot_pose_ekf/odom_combined", geometry_msgs::PoseWithCovarianceStamped , pr2_odom)

struct MySystem{
  ACCESS(CtrlMsg, ctrl_ref)
      ACCESS(CtrlMsg, ctrl_obs)
      ACCESS(geometry_msgs::PoseWithCovarianceStamped, pr2_odom)
      MySystem(){
    if(mlr::getParameter<bool>("useRos", false)){
      addModule<ROSSUB_pr2_odom>(NULL, /*Module::loopWithBeat,*/ 0.02);
      new RosCom_Spinner();
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
    }
    //connect();
  }
};

void changeColor(void*){  orsDrawAlpha = .5; glColor(.5,.0,.0); }
void changeColor2(void*){  orsDrawAlpha = 1.; }

void setOdom(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose){
  ors::Quaternion quat(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
  ors::Vector pos(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);

  double angle;
  ors::Vector rotvec;
  quat.getRad(angle, rotvec);
  q(qIndex+0) = pos(0);
  q(qIndex+1) = pos(1);
  q(qIndex+2) = mlr::sign(rotvec(2)) * angle;
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  bool useRos = mlr::getParameter<bool>("useRos", false);
  bool fixBase = mlr::getParameter<bool>("fixBase", false);

  MySystem S;
  threadOpenModules(true);

  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");
  ors::KinematicWorld world_pr2 = world;
  world.gl().add(changeColor);
  world.gl().add(ors::glDrawGraph, &world_pr2);
  world.gl().add(changeColor2);

  ors::Joint *trans=world.getJointByName("worldTranslationRotation");
  arr q, qdot;

  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    uint trials=0;
    for(;useRos;){
      S.ctrl_obs.var->waitForNextRevision();
      cout <<"REMOTE joint dimension=" <<S.ctrl_obs.get()->q.N <<endl;
      cout <<"LOCAL  joint dimension=" <<world.q.N <<endl;

      if(S.ctrl_obs.get()->q.N==world.q.N
         && S.ctrl_obs.get()->qdot.N==world.q.N)
        break;

      trials++;
      if(trials>20){
        HALT("sync'ing real PR2 with simulated failed - using useRos=false")
      }
    }

    q = S.ctrl_obs.get()->q;
    qdot = S.ctrl_obs.get()->qdot;

    setOdom(q, trans->qIndex, S.pr2_odom.get());
    world.setJointState(q,qdot);
    world_pr2.setJointState(q,qdot);
  }

  arr q0(3);
  q0(0) = q(trans->qIndex+0);
  q0(1) = q(trans->qIndex+1);
  q0(2) = q(trans->qIndex+2);
  arr target = q0 + ARR(-0.2*cos(q(trans->qIndex+2)),-0.2*sin(q(trans->qIndex+2)),-0.8);


  MotionProblem MP(world);
  Task *task;
  task = MP.addTask("transitions", new TransitionTaskMap(world));
  task->map.order=2;
  task->setCostSpecs(0, MP.T, ARR(0.), 1e0);
  task = MP.addTask("position", new TaskMap_qItself(world,"worldTranslationRotation"));
  task->setCostSpecs(MP.T-5, MP.T, target, 1e3);
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  optConstrained(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2.));
  MP.costReport();
  displayTrajectory(x, 1, world, "planned trajectory");



  /// execute trajectory on robot
  setOdom(q, trans->qIndex, S.pr2_odom.get());
  world.setJointState(q,qdot);
  world_pr2.setJointState(q,qdot);

  double duration = mlr::getParameter<double>("duration");
  double tau = duration/x.d0;
  arr xd;
  getVel(xd,x,tau);

  mlr::Spline xs(x.d0,x);
  mlr::Spline xds(x.d0,xd);
  CtrlMsg refs;
  arr zero_qdot(qdot.N);
  zero_qdot.setZero();
  arr q_real;
  double s = 0.;
  double t = 0.;
  mlr::timerStart(true);
  while(t<2*duration){
    refs.fL = zeros(6);
    refs.KiFT.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = 1.;
    refs.Kd = 1.;
    refs.Ki = 0.;
    refs.gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 0.;

    s = min(ARR(t/duration,1.));
    refs.q=xs.eval(s);
    q_real = S.ctrl_obs.get()->q;
    setOdom(q_real, trans->qIndex, S.pr2_odom.get());

//    cout << refs.q << endl;
//    cout << q_real << endl;

    double gain = 0.75;
    refs.qdot=zero_qdot;
    arr v = xds.eval(s);;
    if(!fixBase && trans && trans->qDim()==3){
      refs.qdot(trans->qIndex+0) = v(trans->qIndex+0) +gain*(refs.q(trans->qIndex+0)-q_real(trans->qIndex+0));
      refs.qdot(trans->qIndex+1) = v(trans->qIndex+1) +gain*(refs.q(trans->qIndex+1)-q_real(trans->qIndex+1));
      refs.qdot(trans->qIndex+2) = v(trans->qIndex+2) +gain*(refs.q(trans->qIndex+2)-q_real(trans->qIndex+2));
    }
    cout << v << endl;

    S.ctrl_ref.set() = refs;

    /// visualize pr2
    world.setJointState(xs.eval(s));
    world_pr2.setJointState(q_real);
    world.gl().update();

    t = t + mlr::timerRead(true);
  }

  cout << q_real(trans->qIndex+0) << " | " << target(trans->qIndex+0) << endl;
  cout << q_real(trans->qIndex+1) << " | " << target(trans->qIndex+1) << endl;
  cout << q_real(trans->qIndex+2) << " | " << target(trans->qIndex+2) << endl;
  threadCloseModules();

  cout <<"bye bye" <<endl;
  return 0;
}

