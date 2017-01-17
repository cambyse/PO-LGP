#include <Motion/komo.h>
#include <string>
#include <map>

using namespace std;

//struct HandPositionMap:TaskMap{
//  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){
//    mlr::Shape *hand = G.getShapeByName("humanL");
//    arr posArm, Jarm;
//    G.kinematicsPos(posArm, Jarm, hand->body); // get function to minimize and its jacobian in state G for the hand
//    // posArm -= ARR(.5,.5,1.3);

//    // commit results
//    y = posArm;
//    J = Jarm;
//  }

//  virtual uint dim_phi(const mlr::KinematicWorld& G){
//    return 3;
//  }

//  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return mlr::String("HandPositionMap"); }

//};

struct HeadPoseMap:TaskMap{

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){
    mlr::Shape *head = G.getShapeByName("manhead");
    arr posHead, JposHead;
    G.kinematicsPos(posHead, JposHead, head->body);    // get function to minimize and its jacobian in state G

    arr quatHead, JquatHead;
    G.kinematicsQuat(quatHead, JquatHead, head->body); // get function to minimize and its jacobian in state G

    // concatenate y and J from position and orientation (quaternion)
    arr tmp_y=zeros(dim_);
    arr tmp_J=zeros(dim_, JposHead.dim(1));
    tmp_y.setVectorBlock(posHead, 0);
    tmp_y.setVectorBlock(quatHead, posHead.dim(0) - 1);

    tmp_J.setMatrixBlock(JposHead, 0, 0);
    tmp_J.setMatrixBlock(JquatHead, JposHead.dim(0) - 1, 0);

    // commit results
    y = tmp_y;
    J = tmp_J;
  }

  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return mlr::String("HeadPoseMap"); }

  static arr buildTarget( mlr::Vector const& position, double yaw_deg )
  {
    mlr::Quaternion target_quat;
    target_quat.setDeg(yaw_deg, 0.0, 0.0, 1.0);

    arr target_arr(dim_);
    target_arr.setVectorBlock( position.getArr(), 0 );
    target_arr.setVectorBlock( conv_quat2arr(target_quat), position.getArr().dim(0) - 1 );

    return target_arr;
  }

private:
  static const uint dim_ = 7;
};

//===========================================================================

void move(){
  
  KOMO komo;
  komo.setConfigFromFile();

  // mlr::Body *b = komo.world.getBodyByName("/human/base");
  // b->X.addRelativeTranslation(.3,0,0);

  //  komo.setHoming(-1., -1., 1e-1);
  //  komo.setSquaredQVelocities();
  komo.setSquaredFixJointVelocities();
  komo.setSquaredFixSwitchedObjects();
  komo.setSquaredQAccelerations();
#if 0
  komo.setPosition(1., 1., "humanL", "target", OT_sumOfSqr, NoArr, 1e2);
#else
  //komo.setPosition(1., 1.1, "humanL", "target", OT_sumOfSqr, NoArr, 1e2);

  //komo.setPosition(1., 2.0, "manhead", "target", OT_sumOfSqr, ARR(0,0,0), 1e2);  // objective to have the head on the target on this time slab
  //komo.setTask(startTime, endTime, new TaskMap_Default(posTMT, world, shape, NoVector, shapeRel, NoVector), type, target, prec);

  //komo.setTask(1.0, 2.0, new HeadPositionMap(), OT_sumOfSqr, ARR(0,-0.4,1.7), 1e2);

  arr target_arr = HeadPoseMap::buildTarget( mlr::Vector( 0, -0.3, 1.7 ), 80 );

  komo.setTask(1.0, 2.0, new HeadPoseMap(), OT_sumOfSqr, target_arr, 1e2);

  //komo.setTask(.3, .5, new HandPositionMap(), OT_sumOfSqr, ARR(.5,.5,1.3), 1e2);
  //komo.setTask(.8, 1., new HandPositionMap(), OT_sumOfSqr, ARR(.8,0.,1.3), 1e2);
  //komo.setTask(.8, 1., new TaskMap_Default(posDiffTMT, komo.world, "/human/humanR", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);

//  komo.setTask(.3, 1., new TaskMap_Default(gazeAtTMT, komo.world, "eyes", NoVector, "target", NoVector), OT_sumOfSqr, NoArr, 1e2);
#endif
  komo.setSlowAround(3., .1, 1e3);

  komo.reset();
  komo.run();
//  komo.checkGradients();

  Graph result = komo.getReport(true);

  for(;;) komo.displayTrajectory(.1, true);
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  move();

  return 0;
}
