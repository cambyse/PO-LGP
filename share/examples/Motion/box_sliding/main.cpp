#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>
#include <Kin/kin_swift.h>

void box1(arr &y){
  mlr::KinematicWorld world("box.ors");

  // set some visualization properties
  world.getJointByName("table_box")->A.pos = mlr::Vector(y.sub(0,2));
  world.getJointByName("table_box")->A.rot.setRad(y(3)*M_PI/180);
  world.getJointByName("table_boxTarget")->A.pos = mlr::Vector(y.sub(4,6));
  world.getJointByName("table_boxTarget")->A.rot.setRad(y(7)*M_PI/180);
  world.getJointByName("table_boxTargetVis")->A = world.getJointByName("table_boxTarget")->A;
  world.getJointByName("table_boxTargetVis2")->A = world.getJointByName("table_boxTarget")->A;
  world.calc_fwdPropagateFrames();

  arr q;
  world.getJointState(q);
//  q=q*0.;
//  q(4)=-1.5;
//  world.setJointState(q);

//  mlr::Vector dir= ( world.getBodyByName("endeffM")->X.pos - world.getBodyByName("endeffL")->X.pos);
//  mlr::Vector pos = world.getBodyByName("endeffL")->X.pos + dir*.5;
//  mlr::Vector dir2 = pos-world.getBodyByName("R_LOWER_WRIST")->X.pos;
//  cout << dir << endl;
//  cout << ~(world.getBodyByName("R_LOWER_WRIST")->X.rot.getArr())*ARRAY(dir2) << endl;

//  world.watch(true);
//  world.watch(true);
//  cout <<world.gl().camera.X->pos << endl;
//  cout <<world.gl().camera.X->rot << endl;

  MotionProblem MP(world,false);
  MP.useSwift=false;
  cout <<"joint dimensionality=" <<q.N <<endl;


  //cout <<   ((TransitionTaskMap*)&t->map)->H_rate_diag << endl;
  arr param = {0.374559, 7163.75, 3985.44, 3487.1, 3887.7, 302.976, 2330.48};
  uint pC = 0;

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(world), OT_sumOfSqr);
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.},param(pC));pC++;
  ((TaskMap_Transition*)&t->map)->H_rate_diag = 1.;

  cout << MP.T << endl;

  double conT = MP.T/2.;
  cout <<"conT " << conT << endl;

  // position task maps
  t = MP.addTask("posT", new TaskMap_Default(posTMT, world, "box", NoVector, "boxTarget",NoVector),OT_sumOfSqr);
  t->setCostSpecs(MP.T-1,MP.T, {0.}, param(pC));pC++;

  t = MP.addTask("vecT", new TaskMap_Default(vecAlignTMT, world, "box", mlr::Vector(0.,1.,0), "boxTarget",mlr::Vector(0.,1.,0)),OT_sumOfSqr);
  t->setCostSpecs(MP.T-1,MP.T, {1.}, param(pC));pC++;
  t = MP.addTask("posC1", new TaskMap_Default(posTMT, world, "endeffL", NoVector),OT_sumOfSqr);
  t->setCostSpecs(conT-5,conT, conv_vec2arr(world.getShapeByName("boxP1")->X.pos), param(pC));pC++;
  t = MP.addTask("posC2", new TaskMap_Default(posTMT, world, "endeffM", NoVector),OT_sumOfSqr);
  t->setCostSpecs(conT-5,conT, conv_vec2arr(world.getShapeByName("boxP2")->X.pos), param(pC));pC++;
  t = MP.addTask("posPre", new TaskMap_Default(posTMT, world, "endeffM", NoVector),OT_sumOfSqr);
  t->setCostSpecs(conT-70,conT-70, conv_vec2arr(world.getShapeByName("preContact")->X.pos), param(pC));pC++;
  t = MP.addTask("rotPre", new TaskMap_Default(vecAlignTMT, world, "endeffC", mlr::Vector(0.,0.,1.),"preContact",mlr::Vector(1.,0.,0.)),OT_sumOfSqr);
  t->setCostSpecs(conT-70,conT-70, {1.}, param(pC));pC++;

  // constraints
  t = MP.addTask("contact1", new PointEqualityConstraint(world, "endeffL",NoVector, "boxP1",NoVector),OT_eq);
  t->setCostSpecs(conT, MP.T, {0.}, 1.);
  t = MP.addTask("contact2", new PointEqualityConstraint(world, "endeffM",NoVector, "boxP2",NoVector),OT_eq);
  t->setCostSpecs(conT, MP.T, {0.}, 1.);

  t = MP.addTask("box_fixation0", new qItselfConstraint(world.getJointByName("table_box")->qIndex, world.getJointStateDimension()),OT_eq);
  t->setCostSpecs(0.,conT, {0.}, 1.);
  t = MP.addTask("box_fixation1", new qItselfConstraint(world.getJointByName("table_box")->qIndex+1, world.getJointStateDimension()),OT_eq);
  t->setCostSpecs(0.,conT, {0.}, 1.);
  t = MP.addTask("box_fixation2", new qItselfConstraint(world.getJointByName("table_box")->qIndex+2, world.getJointStateDimension()),OT_eq);
  t->setCostSpecs(0.,conT, {0.}, 1.);
  t = MP.addTask("velocity_dir2", new VelAlignConstraint(world, "endeffM",NoVector, "box", mlr::Vector(1,0,0),.99),OT_ineq);
  t->setCostSpecs(conT, MP.T, {0.}, 1.);

//  t = MP.addTask("collision", new ProxyConstraint(allPTMT,{} , 0.001));
//  t->setCostSpecs(0., MP.T/2-200, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
//  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  optConstrained(x, NoArr, Convert(MP), OPT(verbose=1, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=1.5,stopTolerance = 1e-5));

  MP.costReport(true);
  //  checkGradient(Convert(MF),x,1e-3);

//  MP.costReport();
  for(;;) {displayTrajectory(x, 1, world, "");}
  world.gl().resize(1000,1000);
  world.setJointState(x[0]);
  world.watch(true);
  for (uint i =1;i<x.d0;i++){
//    world.setJointState(x[i]);
//    world.gl().update();
//    world.watch(true);
  }
  world.watch(true);
}


int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  arr targets;
  targets << FILE("targets");
  cout << targets << endl;

  for (uint i =0;i<targets.d0;i++){
    arr param = targets[i];
    box1(param);
  }


  return 0;
}


