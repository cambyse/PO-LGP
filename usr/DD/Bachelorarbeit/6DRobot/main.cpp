#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Algo/gaussianProcess.h>
#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <Roopi/roopi.h>
#include <Control/TaskControllerModule.h>
#include <Ors/orsviewer.h>

#include "taskMapVariance.h"
#include "objectGenerator.h"

double noise=.1,priorVar=1.,kernelRange=.2;
void randomData(arr& X,arr& Y){
  X.setGrid(2,-4.,4.,5);
  Y=sin(X);
  Y.reshape(Y.N);
  rndGauss(Y,noise,true);
}

double kernel(void *P, const arr& x,const arr& y){
  if(&x==&y) return priorVar+noise*noise;
  GaussKernelParams a = *(GaussKernelParams*)P;
  double d=sqrDistance(x,y);
  return priorVar*::exp(-.5 * d/(kernelRange*kernelRange));
}

void testGP() {

  arr X,Y,Xp,Yp,Sp;
  GaussianProcess gp;
  GaussKernelParams gpp(1., 1., .1);
  gp.obsVar = 0.05;
  gp.setKernel(GaussKernel, &gpp);
  //gp.setKernel(kernel, &gpp);

  gp.dcov=dGaussKernel;
  gp.covF_D = GaussKernelF_D;
  gp.covD_D = GaussKernelD_D;

  randomData(X,Y);
  //cout << X << endl;
  //gp.recompute(X,Y);
  gp.appendObservation(~ARR(1.0,0.0),1.0);
  gp.recompute();
  arr y, s;
  gp.evaluate(~ARR(1.0,0.0), y, s);
  cout << y << endl;
  cout << s << endl;
  arr g;
  gp.gradient(g, ~ARR(2.0,0.0));
  cout << g << endl;
  //plotBelief(gp,-5.,5., true);
}

void tests() {
  ors::KinematicWorld world("model.ors");
  //world.joints.first()->type = ors::JT_free;
  //world.analyzeJointStateDimensions();
  //world.calc_fwdPropagateFrames();

  cout << world.getJointStateDimension() << endl;

  MotionProblem MP(world);

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), sumOfSqrTT);
  t->map.order=2; //acceleration task
  t->setCostSpecs(0, MP.T, {0.}, 1.0);

  //t = MP.addTask("collisions", new CollisionConstraint(0.11), ineqTT);
  //t->setCostSpecs(0., MP.T, {0.}, 1.0);

  t = MP.addTask("bla", new TaskMap_Default(posTMT, world, "endeff"), sumOfSqrTT);
  t->setCostSpecs(MP.T-5, MP.T, ARR(0.0,1.0,0.0), 5.0);

  t = MP.addTask("bla2", new TaskMap_Default(vecTMT, world, "endeff", ors::Vector(0.0,1.0,0.0)), sumOfSqrTT);
  t->setCostSpecs(MP.T-5, MP.T, ARR(0.0,.0,1.0), 5.0);

  arr x = MP.getInitialization();

  optConstrained(x , NoArr, Convert(MP), OPT(verbose=1));
  MP.costReport();
  x.reshape(MP.T, x.N/MP.T);
  displayTrajectory(x, x.d0, world, "blub", 0.1);
  world.watch(true);
  world.setJointState(x[x.d0-1]);
  world.watch(true);
}

void verruecktWennDasKlappt() {
  ors::KinematicWorld world("model.ors");

  MotionProblem MP(world);

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), sumOfSqrTT);
  t->map.order=2; //acceleration task
  t->setCostSpecs(0, MP.T, {0.}, .05);

  GaussianProcess gp;
  GaussKernelParams gpp(1.5, 1.5, .1);
  gp.obsVar = 0.05;
  gp.setKernel(GaussKernel, &gpp);
  //gp.setKernel(kernel, &gpp);
  gp.mu = 1.0;
  gp.dcov = dGaussKernel;
  gp.covF_D = GaussKernelF_D;
  gp.covD_D = GaussKernelD_D;
  gp.appendObservation(~ARR(0.0,0.0,0),1.0);
  gp.appendObservation(~ARR(1.0,0.0,0),1.0);
  gp.appendObservation(~ARR(1.0,1.0,0),1.0);
  gp.appendObservation(~ARR(0.0,1.0,0),1.0);
  gp.appendObservation(~ARR(0.0,0.0,-1.0),1.0);
  gp.appendObservation(~ARR(1.0,0.0,-1.0),1.0);
  gp.appendObservation(~ARR(1.0,1.0,-1.0),1.0);
  gp.appendObservation(~ARR(0.0,1.0,-1.0),1.0);


  gp.appendObservation(~ARR(0.1,0.1,-0.1),-1.0);
  gp.appendObservation(~ARR(0.9,0.0,-0.1),-1.0);
  gp.appendObservation(~ARR(0.9,0.9,-0.1),-1.0);
  gp.appendObservation(~ARR(0.0,0.9,-0.1),-1.0);
  gp.appendObservation(~ARR(0.1,0.1,-0.9),-1.0);
  gp.appendObservation(~ARR(0.9,0.0,-0.9),-1.0);
  gp.appendObservation(~ARR(0.9,0.9,-0.9),-1.0);
  gp.appendObservation(~ARR(0.0,0.9,-0.9),-1.0);


  //gp.appendObservation(~ARR(0.0,3.0,-1.0),1.0);
  //gp.appendObservation(~ARR(1.0,3.0,-1.0),1.0);
  //gp.appendObservation(~ARR(0.0,3.0,1.0),1.0);

  gp.recompute();
  arr y, v;
  gp.evaluate(~ARR(0.,0.,0.0), y,v);
  cout << v << endl;

  ScalarFunction blobby = [&gp](arr&,arr&, const arr& X){
    arr y, non;
    gp.evaluate(~X, y, non);
    return y.first();
  };

  ors::Mesh m;
  world.gl().add(m);
  m.setImplicitSurface(blobby,-1.5,1.5);
  cout << m << endl;

  world.gl().add(glDrawPlot, &plotModule);
  arr X;
  X.setGrid(3,-2,2,10);
  arr grad = zeros(X.d0,3);
  for(uint i = 0; i < X.d0; i++) {
    arr g;
    gp.gradient(g, X[i]);
    grad[i] = g;
  }
  //plotVectorField(X, grad);



  TaskMap* taskMap = new TaskMapVariance(gp, world, "endeff");

  //t = MP.addTask("super", taskMap, sumOfSqrTT);
  //t->map.order = 2;
  //t->setCostSpecs(0, MP.T, {-0.5}, 10.0);

  TaskMap* ori = new TaskMapGPGradient(gp, world, "endeff", ors::Vector(1.0,0.0,0.0));
  t = MP.addTask("orie", ori, sumOfSqrTT);
  t->setCostSpecs(0, MP.T, {0.0}, 10.0);

  //t = MP.addTask("bla", new TaskMap_Default(vecTMT, world, "endeff", ors::Vector(0.0,0.0,1.0)), sumOfSqrTT);
  //t->setCostSpecs(0, MP.T, ARR(0.0,0.0,1.0), 2.0);

  t = MP.addTask("bla", new TaskMap_Default(posTMT, world, "endeff"), sumOfSqrTT);
  t->setCostSpecs(MP.T-5, MP.T, ARR(0.0,2.0,0.0), 5.0);

  arr x = MP.getInitialization();

  //checkJacobianCP(Convert(MP), x, 1e-4);

  optConstrained(x , NoArr, Convert(MP), OPT(verbose=1));
  MP.costReport();
  x.reshape(MP.T, x.N/MP.T);
  //displayTrajectory(x, x.d0, world, "blub", 0.1);
  world.watch(true);
  for(uint i = 0; i < x.d0; i++) {
    world.setJointState(x[i]);
    world.watch(false);
    arr y, v;
    //gp.evaluate(~x[i], y, v);
    //cout << v << endl;
    cout << ori->phi(world) << endl;
    mlr::wait(0.1);
  }
  world.setJointState(x[x.d0-1]);
  world.watch(true);
}


void withRobot() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Object o(world);
  o.generateObject();

  GaussianProcess gp;
  GaussKernelParams gpp(1.0, .2, .001);
  gp.obsVar = 0.001;
  gp.setKernel(GaussKernel, &gpp);
  gp.mu = 1.0;
  gp.dcov = dGaussKernel;
  gp.covF_D = GaussKernelF_D;
  gp.covD_D = GaussKernelD_D;

  for(uint i = 0; i < 100; i++) {
    gp.appendObservation(~o.sampleFromObject(), 0.0);
  }
  //gp.appendObservation(~(conv_vec2arr(o.s->mesh.center()+conv_vec2arr(o.s->X.pos))), -1.0);
  gp.recompute();
  //FILE("X") << gp.X;
  //FILE("Y") << gp.Y;

  ScalarFunction impl = [&gp](arr&,arr&, const arr& X){
    arr y, non;
    gp.evaluate(~X, y, non);
    return y.first();
  };


  //world.watch(true);

  //makeConvexHulls(world.shapes);

  //OrsPoseViewer* viewer = getThread<OrsPoseViewer>("OrsPoseViewer");

  //ors::Mesh me;
  //viewer->gl.add(me);
  //me.setBox();
  //me.setRandom(10);
  //me.scale(1.5,1.5,1.5);
  world.gl().add(glDrawPlot, &plotModule);
  arr X;
  X.setGrid(3,-1,1,20);
  //X = gp.X;
  arr grad = zeros(X.d0,3);
  for(uint i = 0; i < X.d0; i++) {
    arr g;
    gp.gradient(g, X[i]);
    grad[i] = g;
  }
  //plotVectorField(X, grad);
  //world.watch(true);

  Roopi R(world);
  OrsPoseViewer* viewer = getThread<OrsPoseViewer>("OrsPoseViewer");
  viewer->gl.add(glDrawPlot, &plotModule);
  //plotVectorField(X, grad);
  //ors::Mesh m;
  //viewer->gl.add(m);
  //m.setImplicitSurface(impl,-1.5,1.5);
  /*
  GaussianProcess gp;
  GaussKernelParams gpp(1.5, 1.5, .1);
  gp.obsVar = 0.05;
  gp.setKernel(GaussKernel, &gpp);
  //gp.setKernel(kernel, &gpp);
  gp.mu = 1.0;
  gp.dcov = dGaussKernel;
  gp.covF_D = GaussKernelF_D;
  gp.covD_D = GaussKernelD_D;
  gp.appendObservation(~ARR(0.0,0.0,0),1.0);
  gp.appendObservation(~ARR(1.0,0.0,0),1.0);
  gp.appendObservation(~ARR(1.0,1.0,0),1.0);
  gp.appendObservation(~ARR(0.0,1.0,0),1.0);
  gp.appendObservation(~ARR(0.0,0.0,-1.0),1.0);
  gp.appendObservation(~ARR(1.0,0.0,-1.0),1.0);
  gp.appendObservation(~ARR(1.0,1.0,-1.0),1.0);
  gp.appendObservation(~ARR(0.0,1.0,-1.0),1.0);


  gp.appendObservation(~ARR(0.1,0.1,-0.1),-1.0);
  gp.appendObservation(~ARR(0.9,0.0,-0.1),-1.0);
  gp.appendObservation(~ARR(0.9,0.9,-0.1),-1.0);
  gp.appendObservation(~ARR(0.0,0.9,-0.1),-1.0);
  gp.appendObservation(~ARR(0.1,0.1,-0.9),-1.0);
  gp.appendObservation(~ARR(0.9,0.0,-0.9),-1.0);
  gp.appendObservation(~ARR(0.9,0.9,-0.9),-1.0);
  gp.appendObservation(~ARR(0.0,0.9,-0.9),-1.0);

  gp.recompute();*/

  /*

  ScalarFunction blobby = [&gp](arr&,arr&, const arr& X){
    arr y, non;
    gp.evaluate(~X, y, non);
    return y.first();
  };

  ors::Mesh m;
  //R.tcm()->modelWorld.set()->gl().add(m);
  //viewer->gl.add(m);
  //m.setImplicitSurface(blobby,-1.5,1.5);

*/
  R.releasePosition();

  CtrlTask* gr = R.createCtrlTask("gr", new TaskMapGP1D(gp, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(gr, ARR(00.0), ARR(5.0));
  R.modifyCtrlTaskReference(gr, ARR(0.85), ARR(-0.1));




  CtrlTask* t = R.createCtrlTask("GP", new TaskMapGPGradient(gp, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(-1.0,0.0,0.0)));
  R.modifyCtrlC(t, ARR(1000.0));
  R.modifyCtrlTaskGains(t, 10.0, 5.0);
  R.modifyCtrlTaskReference(t, ARR(0.0));
  //R.activateCtrlTask(t);


  CtrlTask* hold = R.createCtrlTask("hold", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(hold, 10.0, 5.0);
  //R.activateCtrlTask(hold);

  arr hoPos = R.getTaskValue(hold);
  //hoPos(0) -= 0.2;
  //hoPos(1) += 0.1;
  //R.modifyCtrlTaskReference(hold, hoPos);

  CtrlTask* mo = R.createCtrlTask("m", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(0.0,0.0,1.0)));
  R.modifyCtrlTaskGains(mo, 0.0, 1.0);
  R.modifyCtrlTaskReference(mo, ARR(0.0), ARR(-0.1));

  //R.waitForConv(t);

  //R.deactivateCtrlTask(hold);
  //R.deactivateCtrlTask(t);
  //R.activateCtrlTask(gr);

  arr V;
  TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(1.0,0.0,0.0));
  arr v;
  eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
  cout << v << endl;
  V.append(~v);
  eOri.ivec = ors::Vector(0.0,1.0,0.0);
  eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
  cout << v << endl;
  V.append(~v);
  eOri.ivec = ors::Vector(0.0,0.0,1.0);
  eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
  cout << v << endl;
  V.append(~v);
  V = ~V;
  arr Lambda = diag(ARR(0.0,10.0,10.0));
  arr A = V*Lambda*~V;
  arr B = V*Lambda/2.0*~V;
  cout << A << endl;
  R.modifyCtrlTaskGains(hold, A, B);

  R.activateCtrlTask(hold);
  hoPos(1) += 0.3;
  R.modifyCtrlTaskReference(hold, hoPos);

  mlr::wait(100.0);

  R.modifyCtrlTaskGains(hold, diag(ARR(10.0,10.0,0.0)), diag(ARR(5.0,5.0,1.0)));
  //R.activateCtrlTask(hold,false);

  CtrlTask* f = R.createCtrlTask("f", new TaskMapGP(gp, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(f, 10.0, 5.0);
  R.modifyCtrlTaskReference(f, ARR(0.0));
  R.activateCtrlTask(f);

  mlr::wait(10.0);

  //arr hoPos = R.getTaskValue(hold);
  hoPos(1) -= 0.1;
  R.modifyCtrlTaskReference(hold, hoPos);


  mlr::wait(10.0);


  CtrlTask* t1D = R.createCtrlTask("t1D", new TaskMap1DPosOrientation(R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(t1D, 0.0, 5.0);
  R.modifyCtrlTaskReference(t1D, ARR(0.0), ARR(0.1));
  R.activateCtrlTask(t1D);

  R.modifyCtrlTaskGains(hold, diag(ARR(10.0,0.0,10.0)), diag(ARR(5.0,0.0,5.0)));

  mlr::wait(100.0);

  //R.activateCtrlTask(mo);

  CtrlTask* move = R.createCtrlTask("move", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlC(move, ARR(1000.0));
  arr Kp = eye(3)*10.0;
  //Kp(2,2) = 0.0;
  arr Kd = Kp*0.5;
  R.modifyCtrlTaskGains(move, Kp, Kd ,0.03);
  arr pos;
  pos = R.getTaskValue(move);

  //pos(1) -= 0.4;
  pos(2) -= 0.2;

  //R.activateCtrlTask(move);
  R.modifyCtrlTaskReference(move, pos);
  //R.followTaskTrajectory(move, 10.0, traj);

  //while(true) {
  //cout << t->map.phi(R.tcm()->modelWorld.get()()) << endl;
  //mlr::wait(0.1);
  //}
  while(true) {
    bool b = false;
    R.tcm()->ctrlTasks.readAccess();
    if(t->isConverged()) b = true;
    R.tcm()->ctrlTasks.deAccess();
    if(b) break;
    mlr::wait(0.1);
  }
  //mlr::wait(10.0);
  cout << "conv" << endl;
  pos(1) -= 0.1;
  pos(2) -= 0.1;
  R.modifyCtrlTaskReference(move, pos);
  while(true) {
    cout << R.getTaskValue(t) << endl;
    mlr::wait(0.1);
  }

  mlr::wait(1000.0);
}

void forceTest() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  //ors::KinematicWorld world("model.ors");
  Object o(world);
  o.generateObject();
  //computeMeshNormals(world.shapes);
  makeConvexHulls(world.shapes);
  while(true) {
    //world.addForce(ors::Vector(1.0,0.0,0.0), world.getBodyByName("l_wrist_roll_link"));
    world.contactsToForces(100.0);
    arr M, F;
    world.equationOfMotion(M, F, false);
    //cout << F << endl;
    arr u = F;
    u(11) = -0.2;
    world.stepDynamics(u, 0.01, 0.0, false);
    world.stepSwift();
    mlr::wait(0.01);
    world.watch(false);
  }
  world.watch(true);

}

void forceSimulation() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Object o(world);
  o.generateObject();
  Roopi R(world);

  CtrlTask* task = R.createCtrlTask("orientEndeffR", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(task, 10.0, 5.0);
  R.modifyCtrlTaskReference(task, ARR(0.0,0.0,-1.0));
  R.releasePosition();
  //R.activateCtrlTask(task);

  //mlr::wait(10.0);

  CtrlTask* t = R.createCtrlTask("moveEndeffR", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", ors::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(t, .0, 1.0);
  R.modifyCtrlTaskReference(t, ARR(0.0), ARR(0.1));
  R.activateCtrlTask(t);

  mlr::wait(2.0);

  R.tcm()->ctrlTasks.writeAccess();
  dynamic_cast<TaskMap_Default&>(t->map).ivec = ors::Vector(1.0,0.0,0.0);
  R.tcm()->ctrlTasks.deAccess();

  mlr::wait(1000.0);
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  //tests();
  //testGP();
  //verruecktWennDasKlappt();
  withRobot();
  //forceTest();
  //forceSimulation();
  return 0;
}
