#include "komo_fine.h"
#include <Kin/taskMaps.h>
#include <Kin/taskMap_InsideBox.h>
#include <Kin/frame.h>

double shapeSize(const mlr::KinematicWorld& K, const char* name, uint i=2);

void KOMO_fineManip::setFineLift(double time, const char *endeff){
    setTask(time+.1, time+.2, new TaskMap_Default(posTMT, world, endeff), OT_sumOfSqr, {0.,0.,.2}, 1e2, 1);
}

void KOMO_fineManip::setFineHoming(double time, const char *gripper){
  uintA bodies;
  mlr::Joint *j;
  for(mlr::Frame *f:world.frames){
      if(f->name!=gripper && (j=f->joint) && !j->constrainToZeroVel && j->qDim()>0) bodies.append(f->ID);
  }
  setTask(time-.1, time, new TaskMap_qItself(bodies, true), OT_sumOfSqr, NoArr, 1e2);
}


void KOMO_fineManip::setFineGrasp(double time, const char *endeff, const char *object, const char *gripper){
    mlr::KinematicWorld& K = world;
    StringA joints = K.getJointNames();

    setKinematicSwitch(time, true, "JT_trans3", endeff, object);
//    setKinematicSwitch(time, true, "ballZero", endeff, object);

    //    setKinematicSwitch(time, true, "insert_transX", NULL, object);

    //height to grasp
    double h = .5*shapeSize(world, object) -.02;
    setTask(time-.1, time, new TaskMap_Default(posDiffTMT, K, endeff, NoVector, object), OT_sumOfSqr, {0.,0.,h}, 1e2);

    //vertical
    setTask(time-.35, time, new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e1);
    //downward motion
    setTask(time-.3, time-.2, new TaskMap_Default(posTMT, K, endeff), OT_sumOfSqr, {0.,0.,-.2}, 1e2, 1);
    //anti-podal
    //    setTask(time-.3, time, new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_x), OT_sumOfSqr, NoArr, 1e1);
    //    setTask(time-.3, time, new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_z), OT_sumOfSqr, NoArr, 1e1);
    //insideBox
//    setTask(time-.1, time, new TaskMap_InsideBox(K, endeff, NoVector, object, .02), OT_ineq, NoArr, 1e2);
    //open gripper
    setTask(time-.5, time-.1, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.06}, 1e2);
    //close gripper
    setTask(time, time, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.0}, 1e2);

    //hold still
    joints.removeValue(gripper);
    setTask(time-.1, time+.05, new TaskMap_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
}

void KOMO_fineManip::setFinePlace(double time, const char *endeff, const char *object, const char *placeRef, const char *gripper){
    mlr::KinematicWorld& K = world;
    StringA joints = K.getJointNames();

    //connect object to placeRef
    mlr::Transformation rel = 0;
    rel.pos.set(0,0, .5*(shapeSize(world, object) + shapeSize(world, placeRef)));
    setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );

    //vertical
    setTask(time-.4, time, new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e2);
    //downward motion
    setTask(time-.35, time-.25, new TaskMap_Default(posTMT, K, endeff), OT_sumOfSqr, {0.,0.,-.2}, 1e2, 1);
    //insideBox
    setTask(time, time, new TaskMap_AboveBox(world, object, placeRef, .1), OT_ineq, NoArr, 1e2);
    //close gripper
    setTask(time-1., time-.15, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.0}, 1e2);
    //open gripper
    setTask(time-.05, time, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.06}, 1e2);

    //hold still
    joints.removeValue(gripper);
    setTask(time-.15, time+.05, new TaskMap_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
}

void KOMO_fineManip::setFinePlaceFixed(double time, const char *endeff, const char *object, const char *placeRef, const mlr::Transformation& relPose, const char* gripper){
    mlr::KinematicWorld& K = world;
    StringA joints = K.getJointNames();

    //connect object to table
    setKinematicSwitch(time, true, "rigidZero", placeRef, object, relPose );

    //vertical
    setTask(time-.4, time, new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e2);
    //downward motion
    setTask(time-.35, time-.25, new TaskMap_Default(posTMT, K, endeff), OT_sumOfSqr, {0.,0.,-.2}, 1e2, 1);
    //close gripper
    setTask(time-1., time-.15, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.0}, 1e2);
    //open gripper
    setTask(time-.05, time, new TaskMap_qItself(QIP_byJointNames, {gripper}, K), OT_sumOfSqr, {.06}, 1e2);

    //hold still
    joints.removeValue(gripper);
    setTask(time-.15, time+.05, new TaskMap_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
}

void KOMO_fineManip::setIKGrasp(const char *endeff, const char *object){
    mlr::KinematicWorld& K = world;

    //height to grasp
    double h = .5*shapeSize(world, object) -.02;
    setTask(1.,1., new TaskMap_Default(posDiffTMT, K, endeff, NoVector, object), OT_eq, {0.,0.,h}, 1e2);

    //anti-podal
    setTask(1.,1., new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_x), OT_eq, NoArr, 1e1);
    setTask(1.,1., new TaskMap_Default(vecAlignTMT, K, endeff, Vector_y, object, Vector_z), OT_eq, NoArr, 1e1);

    //insideBox
//    setTask(time-.1, time, new TaskMap_InsideBox(K, endeff, NoVector, object, .02), OT_ineq, NoArr, 1e2);

    //vertical
    setTask(1.,1., new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e0 );
}

void KOMO_fineManip::setIKPlaceFixed(const char *object, const mlr::Transformation& worldPose){
    mlr::KinematicWorld& K = world;

    setTask(1.,1., new TaskMap_Default(posDiffTMT, K, object), OT_eq, worldPose.pos.getArr(), 1e1);
    Task *t = setTask(1.,1., new TaskMap_Default(quatDiffTMT, K, object), OT_eq, {}, 1e1);
//    t->map->flipTargetSignOnNegScalarProduct=true;

    //vertical
//    setTask(time-.4, time, new TaskMap_Default(vecTMT, K, endeff, Vector_z), OT_sumOfSqr, {0.,0.,1.}, 1e2);

}

void KOMO_fineManip::setGoTo(const arr &q, const char* endeff, double up, double down){
  mlr::KinematicWorld& K = world;

  if(endeff){
    arr profile(T, 3);
    profile.setZero();

    if(up>0.){
      uint t0=up*T;
      for(uint t=0;t<t0;t++) profile[t] = ARR(0.,0., .3*sin(.5*(double(t)/t0)*MLR_PI));
      setTask(0., up, new TaskMap_Default(posDiffTMT, K, endeff), OT_eq, profile, 1e2, 1);
    }

    if(down>0.){
      uint t0=down*T;
      for(uint t=t0;t<T;t++) profile[t] = ARR(0.,0., -.3*sin((.5+.5*double(t-t0)/(T-t0))*MLR_PI));
      setTask(down,1., new TaskMap_Default(posDiffTMT, K, endeff), OT_eq, profile, 1e2, 1);
    }
  }

  setTask(1., 1., new TaskMap_qItself(), OT_eq, q, 1e1);
}
