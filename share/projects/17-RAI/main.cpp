#include <Roopi/roopi.h>
#include <KOMO/komo.h>
#include <Control/taskControl.h>
//#include <RosCom/subscribeRosKinect.h>
//#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>
//#include <memory>
//#include <RosCom/roscom.h>
#include <Kin/frame.h>
#include <Kin/taskMap_InsideBox.h>

#include <Core/thread.h>

double shapeSize(const mlr::KinematicWorld& K, const char* name, uint i=2);

struct KOMO_fineManip : KOMO{
  KOMO_fineManip(const mlr::KinematicWorld& K) : KOMO(K){}

  void setFineGrasp(double time, const char *endeff, const char *object, const char* gripper){
    mlr::KinematicWorld& K = world;
    StringA joints = K.getJointNames();

    setKinematicSwitch(time, true, "JT_transX", endeff, object);
//    setKinematicSwitch(time, true, "insert_transX", NULL, object);

    //vertical
    setTask(time-.3, time, new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e1);
    //downward motion
    setTask(time-.3, time-.2, new TaskMap_Default(posTMT, K, endeff), OT_sumOfSqr, {0.,0.,-.3}, 1e0, 1);
    //anti-podal
//    setTask(time-.3, time, new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_x), OT_sumOfSqr, NoArr, 1e1);
//    setTask(time-.3, time, new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_z), OT_sumOfSqr, NoArr, 1e1);
    //insideBox
    setTask(time-.1, time, new TaskMap_InsideBox(K, endeff, NoVector, object, .05), OT_ineq, NoArr, 1e2);
    //open gripper
    setTask(time-.2, time-.1, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.04}, 1e1);
    setTask(time, time, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.01}, 1e1);
    //hold still
    joints.removeValue(gripper);
    setTask(time-.1, time+.05, new TaskMap_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
  }

  void setFinePlace(double time, const char *endeff, const char *object, const char* placeRef, const char* gripper){
    mlr::KinematicWorld& K = world;
    StringA joints = K.getJointNames();

    //connect object to placeRef
    mlr::Transformation rel = 0;
    rel.pos.set(0,0, .5*(shapeSize(world, object) + shapeSize(world, placeRef)));
    setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );

    //vertical
    setTask(time-.4, time, new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e2);
    //downward motion
    setTask(time-.35, time-.25, new TaskMap_Default(posTMT, K, endeff), OT_sumOfSqr, {0.,0.,-.3}, 1e0, 1);
    //insideBox
    setTask(time, time, new TaskMap_AboveBox(world, object, placeRef, .1), OT_ineq, NoArr, 1e2);
    //open gripper
    setTask(time-1., time-.1, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.01}, 1e1);
    setTask(time, time, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.04}, 1e1);

    //hold still
    joints.removeValue(gripper);
    setTask(time-.15, time+.05, new TaskMap_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);

  }

};

//===============================================================================

struct KinSim : Thread{
  mlr::KinematicWorld K;
  uint pathRev=0, switchesRev=0;
  mlr::Spline reference;
  double phase=0.;
  double dt;
  ofstream log;

  Access<arr> path;
  Access<arr> currentQ;
  Access<arr> nextQ;
  Access<StringA> switches;
  Access<mlr::KinematicWorld> world;

  KinSim(double dt=.01) : Thread("KinSim", dt), dt(dt),
    path(this, "path"),
    currentQ(this, "currentQ"),
    nextQ(this, "nextQ"),
    switches(this, "switches"),
    world(this, "world"){

    K = world.get();

    reference.points = K.q;
    reference.points.append( K.q );
    reference.points.append( K.q );
    reference.points.reshape(3, K.q.N );
    reference.degree = 2;
    reference.setUniformNonperiodicBasis();
    log.open("z.KinSim");
    this->K.gl().title = "KinSim";

    currentQ.set() = K.q;
    nextQ.set() = K.q;
  }
  ~KinSim(){
    log.close();
  }

  void open(){}
  void close(){}

  void step(){
    //check for path update
    uint rev=path.readAccess();
    if(pathRev != rev){
      pathRev = rev;
      reference.points = K.getJointState();
      reference.points.append(path());
      reference.points.reshape(path().d0+1, K.getJointStateDimension());
      reference.setUniformNonperiodicBasis();
      phase=0.;
      nextQ.set() = reference.points[-1];
    }
    path.deAccess();

    //check for switches update
    uint rev2=switches.readAccess();
    if(switchesRev != rev2){
      switchesRev = rev2;
      StringA cmd = switches.get();
      cout <<"CMD = " <<cmd <<endl;
      if(cmd(0)=="attach"){
        mlr::Frame *a = K.getFrameByName(cmd(1));
        mlr::Frame *b = K.getFrameByName(cmd(2));

        if(b->parent) b->unLink();
        b->linkFrom(a, true);
        (new mlr::Joint(*b)) -> type=mlr::JT_rigid;
        K.calc_q();
      }

    }
    switches.deAccess();

    arr q = reference.eval(phase);
    K.setJointState(q);
    currentQ.set() = q;
    world.set() = K;
    K.gl().update();

    log <<q <<endl;

    phase += dt;
    if(phase>1.) phase=1.;
  }

};

//===============================================================================

void planGrasp(){
  KOMO_fineManip komo(Access<mlr::KinematicWorld>("world").get());

  komo.world.setJointState(Access<arr>("nextQ").get());

  komo.setPathOpt(1., 60, 5.);
  komo.setFineGrasp(1., "endeff", "stick", "wsg_50_base_joint_gripper_left");

  komo.reset();
  komo.run();
  komo.getReport(true);

//  for(;;)
//  komo.displayTrajectory(.05, true);
//  R.wait();

  StringA joints = komo.world.getJointNames();
  arr x = komo.getPath(joints);
  Access<arr>("path").set() = x;
}

//===============================================================================

void planPlace(){
  KOMO_fineManip komo(Access<mlr::KinematicWorld>("world").get());

  komo.world.setJointState(Access<arr>("nextQ").get());

  komo.setPathOpt(1., 60, 5.);
  komo.setFinePlace(1., "endeff", "stick", "table1", "wsg_50_base_joint_gripper_left");

  komo.reset();
  komo.run();
  komo.getReport(true);

//  for(;;)
//  komo.displayTrajectory(.05, true);
//  R.wait();

  StringA joints = komo.world.getJointNames();
  arr x = komo.getPath(joints);
  Access<arr>("path").set() = x;
}

//===============================================================================

void planHome(){
  KOMO_fineManip komo(Access<mlr::KinematicWorld>("world").get());

  komo.world.setJointState(Access<arr>("nextQ").get());

  komo.setPathOpt(1., 60, 5.);

  komo.setHoming(.9, 1., 1e2);

  komo.reset();
  komo.run();
  komo.getReport(true);

//  for(;;)
//  komo.displayTrajectory(.05, true);
//  R.wait();

  StringA joints = komo.world.getJointNames();
  arr x = komo.getPath(joints);
  Access<arr>("path").set() = x;
}


//===============================================================================

void TEST(PickAndPlace2) {
  Roopi R;
  R.startTweets();

  Access<mlr::KinematicWorld>("world").set()->init("model.g");
  KinSim sim;
  sim.threadLoop();

  for(;;){
  planGrasp();
  R.wait();

  Access<StringA>("switches").set() = {"attach", "endeff", "nostick"};

  planHome();
  R.wait();

  planPlace();
  R.wait();

  Access<StringA>("switches").set() = {"attach", "table1", "nostick"};

  planHome();
  R.wait();
  }


}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testPickAndPlace2();

  return 0;
}

