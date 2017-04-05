#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include "../12_MBMF_LEARNING/task_manager.h"
#include "../12_MBMF_LEARNING/motion_interface.h"
#include "../src/traj_factory.h"
#include "../src/plotUtil.h"
#include <pr2/roscom.h>


#include <System/engine.h>

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  bool useRos = mlr::getParameter<bool>("useRos");
  mlr::KinematicWorld world("model.kvg");
  world.gl().resize(800,800);
  arr X;
  X << FILE("data/door1/Xact2.dat");


  arr Pdemo1,P1,Pdemo2,P2;
  TrajFactory tf;

  drawLine(world,X,Pdemo1,"endeffC1",0);
  drawLine(world,X,Pdemo2,"endeffC2",1);
  drawPoints(world,X,P1,"endeffC1",0);
  drawPoints(world,X,P2,"endeffC2",1);

  for (uint t=0;t<X.d0;t++) {
    world.setJointState(X[t]);
    mlr::Body *handle = world.getBodyByName("handle");
    mlr::Shape *ec1 = world.getShapeByName("endeffC1");
    mlr::Shape *ec2 = world.getShapeByName("endeffC2");

    handle->X.pos = (P1[t]+P2[t])/2.;
//    mlr::Vector x_axis(0.,1.,0.);
//    mlr::Vector y_axis(0.,1.,0.);
//    mlr::Vector z_axis(0.,.0,1.);
//    mlr::Vector v  = ec1->X.rot.getY();
//    handle->X.rot.setDiff(x_axis,v);
//    handle->X.rot.setDiff(y_axis,ec1->X.rot.getZ());

//    arr R;
//    R = ec1->X.rot.getArr();
//    R[0] =  ARR(ec1->X.rot.getX());
//    R[1] =  ARR(ec1->X.rot.getY());
//    R[2] =  ARR(ec1->X.rot.getZ());
//    R=~R;
//    handle->X.rot.setMatrix(R.p);

    handle->X.rot = ec1->X.rot;
    double d = length(ARR(ec1->X.pos-ec2->X.pos));
    double h = .029;
    double w = sqrt(d*d-h*h);
    double alpha = acos(w/d)*180./M_PI;
    double beta = asin(h/d)*180./M_PI;;
    handle->X.addRelativeRotationDeg(90.,0.,1.,0.);
    handle->X.addRelativeRotationDeg(-alpha-beta,1.,0.,0.);

    world.gl().update(STRING(t));
    world.watch(true);
  }

  return 0;



/*


//  world.setJointState(X[T(0)]);
  arr R1 = world.getShapeByName("endeffC1")->X.rot.getArr();
  arr R2 = world.getShapeByName("endeffC2")->X.rot.getArr();

  //  arr offset = ARR(0.05,0.,0.0);
  //  // test translation transformation
  //  for (uint t=0; t<T(0); t++){
  //    /// linear transition from trajectory to contact point
  //    P1[t] = P1[t] + t/T(0)*R1*offset;
  //    P2[t] = P2[t] + t/T(0)*R2*offset;
  //  }
  //  for (uint t=T(0); t<T(1); t++){
  //    /// linear transition from trajectory to contact point
  //    R1 = world.getShapeByName("endeffC1")->X.rot.getArr();
  //    P1[t] = P1[t] + R1*offset;
  //    R2 = world.getShapeByName("endeffC2")->X.rot.getArr();
  //    P2[t] = P2[t] + R2*offset;
  //  }

  // test gripper opening
  arr offset = ARR(0.0,0.02,0.0);
  for (uint t=0; t<T(0); t++){
    /// linear transition from trajectory to contact point
    P1[t] = P1[t] + t/T(0)*R1*offset;
    P2[t] = P2[t] - t/T(0)*R2*offset;
  }
  for (uint t=T(0); t<T(1); t++){
    world.setJointState(X[t]);
    /// linear transition from trajectory to contact point
    R1 = world.getShapeByName("endeffC1")->X.rot.getArr();
    P1[t] = P1[t] + R1*offset;
    R2 = world.getShapeByName("endeffC2")->X.rot.getArr();
    P2[t] = P2[t] - R2*offset;
  }


  cout << sum((P2-P1)%(P2-P1)) << endl;
  cout << sum((Pdemo2-Pdemo1)%(Pdemo2-Pdemo1)) << endl;

  MotionProblem MP(world,false);
  MP.T = T(1)-1;
  MP.tau = 0.01;
  MP.x0 = X[0];

  //--tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = world.getHmetric();


  t =MP.addTask("posC1", new DefaultTaskMap(posTMT,world,"endeffC1"));
  t->setCostSpecs(0,MP.T, P1, 1e2);
  t =MP.addTask("posC2", new DefaultTaskMap(posTMT,world,"endeffC2"));
  t->setCostSpecs(0,MP.T, P2, 1e2);

  MotionProblemFunction MPF(MP);
  arr Xt = X;
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0;
  optConstrainedMix(Xt, NoArr, Convert(MPF), o);

  for(;;){
    displayTrajectory(X,-1,world,"demo");
    displayTrajectory(Xt,-1,world,"trans");
    world.watch(true);
  }

  return 0;
*/
}
