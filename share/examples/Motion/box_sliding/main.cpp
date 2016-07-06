#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>
#include <Ors/ors_swift.h>

void box1(arr &y){
  ors::KinematicWorld world("box.ors");
//  world.meldFixedJoints();
//  world.removeUselessBodies();
//  makeConvexHulls(world.shapes);
  // set some visualization properties
  world.getJointByName("table_box")->A.pos = ors::Vector(y.subRange(0,2));
  world.getJointByName("table_box")->A.rot.setRad(y(3)*M_PI/180);
  world.getJointByName("table_boxTarget")->A.pos = ors::Vector(y.subRange(4,6));
  world.getJointByName("table_boxTarget")->A.rot.setRad(y(7)*M_PI/180);
  world.getJointByName("table_boxTargetVis")->A = world.getJointByName("table_boxTarget")->A;
  world.getJointByName("table_boxTargetVis2")->A = world.getJointByName("table_boxTarget")->A;
  world.calc_fwdPropagateFrames();

  arr q;
  world.getJointState(q);
//  q=q*0.;
//  q(4)=-1.5;
//  world.setJointState(q);

//  ors::Vector dir= ( world.getBodyByName("endeffM")->X.pos - world.getBodyByName("endeffL")->X.pos);
//  ors::Vector pos = world.getBodyByName("endeffL")->X.pos + dir*.5;
//  ors::Vector dir2 = pos-world.getBodyByName("R_LOWER_WRIST")->X.pos;
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
  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, ARR(0.),param(pC));pC++;
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;


  double conT = MP.T/2.;
  cout <<"conT " << conT << endl;

  // position task maps
  t = MP.addTask("posT", new DefaultTaskMap(posTMT, world, "box", NoVector, "boxTarget",NoVector));
  t->setCostSpecs(MP.T,MP.T, {0.}, param(pC));pC++;
  t = MP.addTask("vecT", new DefaultTaskMap(vecAlignTMT, world, "box", ors::Vector(0.,1.,0), "boxTarget",ors::Vector(0.,1.,0)));
  t->setCostSpecs(MP.T,MP.T, {1.}, param(pC));pC++;
  t = MP.addTask("posC1", new DefaultTaskMap(posTMT, world, "endeffL", NoVector));
  t->setCostSpecs(conT-5,conT, conv_vec2arr(world.getShapeByName("boxP1")->X.pos), param(pC));pC++;
  t = MP.addTask("posC2", new DefaultTaskMap(posTMT, world, "endeffM", NoVector));
  t->setCostSpecs(conT-5,conT, conv_vec2arr(world.getShapeByName("boxP2")->X.pos), param(pC));pC++;
  t = MP.addTask("posPre", new DefaultTaskMap(posTMT, world, "endeffM", NoVector));
  t->setCostSpecs(conT-70,conT-70, conv_vec2arr(world.getShapeByName("preContact")->X.pos), param(pC));pC++;
  t = MP.addTask("rotPre", new DefaultTaskMap(vecAlignTMT, world, "endeffC", ors::Vector(0.,0.,1.),"preContact",ors::Vector(1.,0.,0.)));
  t->setCostSpecs(conT-70,conT-70, ARR(1.), param(pC));pC++;

  // constraints
  t = MP.addTask("contact1", new PointEqualityConstraint(world, "endeffL",NoVector, "boxP1",NoVector));
  t->setCostSpecs(conT, MP.T, {0.}, 1.);
  t = MP.addTask("contact2", new PointEqualityConstraint(world, "endeffM",NoVector, "boxP2",NoVector));
  t->setCostSpecs(conT, MP.T, {0.}, 1.);

  t = MP.addTask("box_fixation0", new qItselfConstraint(world.getJointByName("table_box")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(0.,conT, ARR(0.), 1.);
  t = MP.addTask("box_fixation1", new qItselfConstraint(world.getJointByName("table_box")->qIndex+1, world.getJointStateDimension()));
  t->setCostSpecs(0.,conT, ARR(0.), 1.);
  t = MP.addTask("box_fixation2", new qItselfConstraint(world.getJointByName("table_box")->qIndex+2, world.getJointStateDimension()));
  t->setCostSpecs(0.,conT, ARR(0.), 1.);
  t = MP.addTask("velocity_dir2", new VelAlignConstraint(world, "endeffM",NoVector, "box", ors::Vector(1,0,0),.99));
  t->setCostSpecs(conT, MP.T, {0.}, 1.);

//  t = MP.addTask("collision", new ProxyConstraint(allPTMT,{} , 0.001));
//  t->setCostSpecs(0., MP.T/2-200, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=1.5,stopTolerance = 1e-4));


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


