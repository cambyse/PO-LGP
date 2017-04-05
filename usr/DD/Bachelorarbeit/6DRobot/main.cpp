#include <Core/array.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Algo/gaussianProcess.h>
#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <Roopi/roopi.h>
#include <Control/TaskControlThread.h>
#include <Kin/kinViewer.h>
#include <Algo/ann.h>

#include <Roopi/surfaceModelAct.h>

#include "taskMapVariance.h"
#include "objectGenerator.h"
#include "surfaceVisualisationModule.h"

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
  mlr::KinematicWorld world("model.ors");
  //world.joints.first()->type = mlr::JT_free;
  //world.analyzeJointStateDimensions();
  //world.calc_fwdPropagateFrames();

  cout << world.getJointStateDimension() << endl;

  MotionProblem MP(world);

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), OT_sumOfSqr);
  t->map.order=2; //acceleration task
  t->setCostSpecs(0, MP.T, {0.}, 1.0);

  //t = MP.addTask("collisions", new CollisionConstraint(0.11), OT_ineq);
  //t->setCostSpecs(0., MP.T, {0.}, 1.0);

  t = MP.addTask("bla", new TaskMap_Default(posTMT, world, "endeff"), OT_sumOfSqr);
  t->setCostSpecs(MP.T-5, MP.T, ARR(0.0,1.0,0.0), 5.0);

  t = MP.addTask("bla2", new TaskMap_Default(vecTMT, world, "endeff", mlr::Vector(0.0,1.0,0.0)), OT_sumOfSqr);
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
  mlr::KinematicWorld world("model.ors");

  MotionProblem MP(world);

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), OT_sumOfSqr);
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

  mlr::Mesh m;
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

  //t = MP.addTask("super", taskMap, OT_sumOfSqr);
  //t->map.order = 2;
  //t->setCostSpecs(0, MP.T, {-0.5}, 10.0);

  TaskMap* ori = new TaskMapGPGradient(gp, world, "endeff", mlr::Vector(1.0,0.0,0.0));
  t = MP.addTask("orie", ori, OT_sumOfSqr);
  t->setCostSpecs(0, MP.T, {0.0}, 10.0);

  //t = MP.addTask("bla", new TaskMap_Default(vecTMT, world, "endeff", mlr::Vector(0.0,0.0,1.0)), OT_sumOfSqr);
  //t->setCostSpecs(0, MP.T, ARR(0.0,0.0,1.0), 2.0);

  t = MP.addTask("bla", new TaskMap_Default(posTMT, world, "endeff"), OT_sumOfSqr);
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
  mlr::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
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

  //mlr::Mesh me;
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
  //mlr::Mesh m;
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

  mlr::Mesh m;
  //R.tcm()->modelWorld.set()->gl().add(m);
  //viewer->gl.add(m);
  //m.setImplicitSurface(blobby,-1.5,1.5);

*/
  R.releasePosition();

  CtrlTask* gr = R.createCtrlTask("gr", new TaskMapGP1D(gp, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(gr, ARR(00.0), ARR(5.0));
  R.modifyCtrlTaskReference(gr, ARR(0.85), ARR(-0.1));




  CtrlTask* t = R.createCtrlTask("GP", new TaskMapGPGradient(gp, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(-1.0,0.0,0.0)));
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

  CtrlTask* mo = R.createCtrlTask("m", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,1.0)));
  R.modifyCtrlTaskGains(mo, 0.0, 1.0);
  R.modifyCtrlTaskReference(mo, ARR(0.0), ARR(-0.1));

  //R.waitForConv(t);

  //R.deactivateCtrlTask(hold);
  //R.deactivateCtrlTask(t);
  //R.activateCtrlTask(gr);

  arr V;
  TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0));
  arr v;
  eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
  cout << v << endl;
  V.append(~v);
  eOri.ivec = mlr::Vector(0.0,1.0,0.0);
  eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
  cout << v << endl;
  V.append(~v);
  eOri.ivec = mlr::Vector(0.0,0.0,1.0);
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


  CtrlTask* t1D = R.createCtrlTask("t1D", new TaskMap1DPosOrientation(R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
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
  mlr::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  //mlr::KinematicWorld world("model.ors");
  Object o(world);
  o.generateObject();
  //computeMeshNormals(world.shapes);
  makeConvexHulls(world.shapes);
  while(true) {
    //world.addForce(mlr::Vector(1.0,0.0,0.0), world.getBodyByName("l_wrist_roll_link"));
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
  mlr::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Object o(world);
  o.generateObject();
  Roopi R(world);

  CtrlTask* col = R.createCtrlTask("col", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(col, 40.0, 10.0);
  R.modifyCtrlTaskReference(col, ARR(0.0));
  R.activateCtrlTask(col);

  CtrlTask* posT = R.createCtrlTask("pos", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posT, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  R.activateCtrlTask(posT, true);
  CtrlTask* task = R.createCtrlTask("orientEndeffR", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(task, 10.0, 5.0);
  R.modifyCtrlTaskReference(task, ARR(0.0,0.0,-1.0));
  R.releasePosition();
  R.activateCtrlTask(task);

  mlr::wait(5.0);

  R.holdPosition();
  CtrlTask* holdLeftArm = R.createCtrlTask("holdLeftArm", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffL"));
  R.modifyCtrlTaskGains(holdLeftArm, 10.0,5.0);
  CtrlTask* ho = R.createCtrlTask("ho", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(ho, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.releasePosition();
  R.activateCtrlTask(col);
  R.activateCtrlTask(holdLeftArm);
  R.activateCtrlTask(ho);
  mlr::wait(1.0);
  CtrlTask* orientation = R.createCtrlTask("orientation", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(orientation, 10.0, 5.0);
  R.modifyCtrlTaskReference(orientation, ARR(0.0,0.0,-1.0));
  R.activateCtrlTask(orientation);
  CtrlTask* move1D = R.createCtrlTask("move1D", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(move1D, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(move1D, ARR(0.0), ARR(0.1));
  R.modifyForce(move1D, ARR(-4.0), 0.005, 0.999);
  R.activateCtrlTask(move1D);
  mlr::wait(3.0);
  //R.modifyCtrlTaskGains(ho, diag(ARR(20.0,20.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  arr pos = R.getTaskValue(ho);
  arr old = pos;
  pos(1) += 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(0) -= 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(1) -= 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(0) += 0.15;
  R.interpolateToReference(ho, 5.0, pos, old);
  mlr::wait(0.5);
  old = pos;
  pos(1) += 0.3;
  R.interpolateToReference(ho, 10.0, pos, old);
  R.holdPosition();
  mlr::wait(1000.0);


  /*CtrlTask* pos = R.createCtrlTask("pos", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(pos, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  R.activateCtrlTask(pos, true);

  CtrlTask* task = R.createCtrlTask("orientEndeffR", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(task, 10.0, 5.0);
  R.modifyCtrlTaskReference(task, ARR(0.0,0.0,-1.0));
  R.releasePosition();
  R.activateCtrlTask(task);

  mlr::wait(3.0);

  R.modifyCtrlTaskGains(pos, diag(ARR(20.0,20.0,0.0)), diag(ARR(5.0,5.0,0.0)));

  CtrlTask* t = R.createCtrlTask("moveEndeffR", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(t, .0, 10.0);
  R.modifyCtrlTaskReference(t, ARR(0.0), ARR(0.1));
  R.activateCtrlTask(t);

  mlr::wait(3.0);
  arr posRef = R.getTaskValue(pos);
  posRef(1) += 0.3;
  R.interpolateToReference(pos, 5.0, posRef);

  while(true) {
    cout << R.getTaskValue(col) << endl;
  }

  /*mlr::wait(2.0);

  R.tcm()->ctrlTasks.writeAccess();
  dynamic_cast<TaskMap_Default&>(t->map).ivec = mlr::Vector(1.0,0.0,0.0);
  R.tcm()->ctrlTasks.deAccess();
  */

  mlr::wait(1000.0);
}

void testThread() {
  mlr::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Object o(world);
  o.generateObject();
  Roopi R(world);

  CtrlTask* col = R.createCtrlTask("col", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(col, 40.0, 10.0);
  R.modifyCtrlTaskReference(col, ARR(0.0));
  R.activateCtrlTask(col);

  CtrlTask* posT = R.createCtrlTask("pos", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posT, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  R.activateCtrlTask(posT, true);
  CtrlTask* task = R.createCtrlTask("orientEndeffR", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(task, 10.0, 5.0);
  R.modifyCtrlTaskReference(task, ARR(0.0,0.0,-1.0));
  R.releasePosition();
  R.activateCtrlTask(task);

  mlr::wait(5.0);

  R.holdPosition();
  CtrlTask* holdLeftArm = R.createCtrlTask("holdLeftArm", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffL"));
  R.modifyCtrlTaskGains(holdLeftArm, 10.0,5.0);
  CtrlTask* ho = R.createCtrlTask("ho", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(ho, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.releasePosition();
  R.activateCtrlTask(col);
  R.activateCtrlTask(holdLeftArm);
  R.activateCtrlTask(ho);
  mlr::wait(1.0);
  CtrlTask* orientation = R.createCtrlTask("orientation", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(orientation, 10.0, 5.0);
  R.modifyCtrlTaskReference(orientation, ARR(0.0,0.0,-1.0));
  R.activateCtrlTask(orientation);
  CtrlTask* move1D = R.createCtrlTask("move1D", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(move1D, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(move1D, ARR(0.0), ARR(0.1));
  R.modifyForce(move1D, ARR(-4.0), 0.005, 0.999);
  R.activateCtrlTask(move1D);
  mlr::wait(3.0);
  //R.modifyCtrlTaskGains(ho, diag(ARR(20.0,20.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", ho);
  arr pos = R.getTaskValue(ho);
  arr old = pos;
  pos(1) += 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  //mlr::wait(100.0);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(0) -= 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(1) -= 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  old = pos;
  pos(0) += 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  old = pos;
  pos(1) += 0.4;
  R.interpolateToReference(th, 10.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  R.holdPosition();
  delete th;
  mlr::wait(1000.0);
}

void testExploreSurface() {
  mlr::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Object o(world);
  o.generateObject();
  Roopi R(world);

  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 40.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);
  surfaceModel.threadLoop();


  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  arr old = pos;
  pos(1) += 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  //mlr::wait(100.0);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(0) -= 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  //pos = R.getTaskValue(ho);
  old = pos;
  pos(1) -= 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  old = pos;
  pos(0) += 0.15;
  R.interpolateToReference(th, 5.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  old = pos;
  pos(1) += 0.4;
  R.interpolateToReference(th, 10.0, pos, old);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  R.holdPosition();

  surfaceModel.timer.report();
  surfaceModel.threadStop();
  surfaceModel.gpSurface.writeAccess();
  arr X = surfaceModel.gpSurface().X;
  /*double xMax = max(X.sub(0,-1,0,0));
  double xMin = min(X.sub(0,-1,0,0));
  double yMax = max(X.sub(0,-1,1,1));
  double yMin = min(X.sub(0,-1,1,1));
  double zMax = max(X.sub(0,-1,2,2));
  double zMin = min(X.sub(0,-1,2,2));

  arr xGrid, yGrid, zGrid;
  xGrid.setGrid(1, xMin, xMax, 20);
  yGrid.setGrid(1, yMin, yMax, 20);
  zGrid.setGrid(1, zMin, zMax, 20);
  xGrid.reshapeFlat();
  yGrid.reshapeFlat();
  zGrid.reshapeFlat();

  arr grid = zeros(xGrid.d0*yGrid.d0*zGrid.d0, 3);
  for(uint i = 0; i < xGrid.d0; i++) {
    for(uint j = 0; j < yGrid.d0; j++) {
      for(uint k = 0; k < zGrid.d0; k++) {
        grid(i*yGrid.d0*zGrid.d0+j*zGrid.d0+k, 0) = xGrid(i);
        grid(i*yGrid.d0*zGrid.d0+j*zGrid.d0+k, 1) = yGrid(j);
        grid(i*yGrid.d0*zGrid.d0+j*zGrid.d0+k, 2) = zGrid(k);
      }
    }
  }*/

  ScalarFunction levelFunc = [&surfaceModel](arr&,arr&, const arr& X) {
    double y, s;
    surfaceModel.gpSurface().evaluate(X, y, s);
    return y - 0.1;
  };

  mlr::Mesh me;
  me.setImplicitSurface(levelFunc, -1.5, 1.5, 100);
  cout << me.V << endl;

  /*ANN ann;
  ann.setX(grid);
  arr halfLevelSet, variance;
  for(uint i = 0; i < X.d0; i++) {
    arr dist;
    intA ind;
    ann.getkNN(dist, ind, X[i], 1000, 0.0, false);
    for(uint j = 0; j < ind.d0; j++) {
      double y, v;
      surfaceModel.gpSurface().evaluate(grid[ind(j)], y, v);
      //cout << y << endl;
      if(fabs(y-0.2) < 0.1) {
        halfLevelSet.append(~grid[ind(j)]);
        variance.append(v);
      }
    }
  }*/

  surfaceModel.gpSurface.deAccess();
/*
  OrsPoseViewer* viewer = getThread<OrsPoseViewer>("OrsPoseViewer");
  viewer->gl.add(glDrawPlot, &plotModule);
  viewer->gl.lock.writeLock();
  //plotPoints(grid);
  plotPoints(halfLevelSet);
  cout << variance << endl << endl;
  uint index = variance.maxIndex();
  cout << variance(index) << endl;
  cout << halfLevelSet[index] << endl;
  //cout << surfaceModel.X.d0 << endl;
  plotPoint(halfLevelSet[index]);
  viewer->gl.lock.unlock();
*/

  //grid.append(~xGrid);
  //grid.append(~yGrid);
  //grid.append(~zGrid);
  //grid = ~grid;
/*
  //arr grid;
  //grid.setGrid(3, -0.3, 1.0, 50);
  arr halfLevelSet, variance;
  for(uint i = 0; i < grid.d0; i++) {
    double y, v;
    surfaceModel.gpSurface().evaluate(grid[i], y, v);
    cout << y << endl;
    if(sign(y-0.1)*(y-0.1) < 0.1) {
      halfLevelSet.append(~grid[i]);
      variance.append(v);
    }
  }
  surfaceModel.gpSurface.deAccess();
  //cout << halfLevelSet << endl;
  OrsPoseViewer* viewer = getThread<OrsPoseViewer>("OrsPoseViewer");
  viewer->gl.add(glDrawPlot, &plotModule);
  viewer->gl.lock.writeLock();
  //plotPoints(grid);
  plotPoints(halfLevelSet);
  cout << variance << endl << endl;
  uint index = variance.maxIndex();
  cout << variance(index) << endl;
  cout << halfLevelSet[index] << endl;
  viewer->gl.lock.unlock();
  //cout << surfaceModel.X.d0 << endl;
  /*plotPoint(halfLevelSet[index]);

  old = pos;
  pos(2) += 0.3;
  R.interpolateToReference(th, 10.0, halfLevelSet[index]);
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();
  R.waitForFinishedTaskReferenceInterpolAct(th);
*/
  mlr::wait(1000.0);

  delete th;


/*

  GaussianProcess gp;
  GaussKernelParams gpp(1.0, .2, 0.2);
  gp.obsVar = 1.0;
  gp.setKernel(GaussKernel, &gpp);
  gp.mu = 1.0;
  gp.dcov = dGaussKernel;
  gp.covF_D = GaussKernelF_D;
  gp.covD_D = GaussKernelD_D;

  gp.X = surfaceModel.X;
  gp.Y = surfaceModel.Y;
  double ac = mlr::timerRead();
  gp.recompute();
  cout << mlr::timerRead() - ac << endl;
  cout << "bl" << endl;

  ScalarFunction impl = [&gp](arr&,arr&, const arr& X){
    arr y, non;
    gp.evaluate(~X, y, non);
    return y.first();
  };

  OrsPoseViewer* viewer = getThread<OrsPoseViewer>("OrsPoseViewer");

  mlr::Mesh me;
  viewer->gl.add(me);
  me.setImplicitSurface(impl,-1.5,1.5);

*/

  //mlr::wait(100.0);
}

void testExploreSurface2() {
  mlr::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Object o(world);
  o.generateObject();
  Roopi R(world);

  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 40.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);
  surfaceModel.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) += 0.05;
  //pos(0) -= 0.05;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  /*pos(0) -= 0.15;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);*/
  for(uint i = 0; i < 10; i++) {
    arr actPos = R.getTaskValue(posEndeff);
    surfaceModel.threadStop();
    arr gradGP, gradV;
    surfaceModel.gpSurface.writeAccess();
    surfaceModel.gpSurface().gradient(gradGP, actPos);
    surfaceModel.gpSurface().gradientV(gradV, actPos);
    gradV.reshapeFlat();
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);
    surfaceModel.gpSurface.deAccess();
    arr newPos = actPos + 0.05*gradVT;
    surfaceModel.threadLoop();
    R.interpolateToReference(th, 5.0, newPos);
    R.waitForFinishedTaskReferenceInterpolAct(th);
  }
}



void testExploreSurface3() {
  mlr::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors"));
  Object o(world);
  //o.generateObject("b", 0.08, 0.08, 0.1, 0.5, -0.2, 0.7);//mlr::Vector(0.6,-0.2,.65);
  o.generateObject("b", 0.2, 0.2, 0.1, 0.6, -0.1, 0.65);
  //Object table(world);
  //table.generateObject("h", 0.4, 0.4, 0.1, 0.6, -0.2, 0.6);
  Roopi R(world);

  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 50.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);
  /*arr tX, tY;
  for(uint i = 0; i < 100; i++) {
    tX.append(~table.sampleFromObject());
    tY.append(0.0);
  }
  surfaceModel.X = tX;
  surfaceModel.Y = tY;*/
  surfaceModel.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) += 0.01;
  //pos(0) -= 0.01;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  /*pos(0) -= 0.15;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);*/
  for(uint i = 0; i < 100000; i++) {
    arr actPos = R.getTaskValue(posEndeff);
    surfaceModel.threadStop();
    arr gradGP, gradV;
    surfaceModel.gpSurface.writeAccess();
    surfaceModel.gpSurface().gradient(gradGP, actPos);
    surfaceModel.gpSurface().gradientV(gradV, actPos);
    gradV.reshapeFlat();
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);
    surfaceModel.gpSurface.deAccess();
    arr newPos = actPos + 0.05*gradVT;// + 0.1*randn(3);
    R.tcm()->ctrlTasks.writeAccess();
    dynamic_cast<TaskMap_Default&>(moveTowardsSurface->map).ivec = mlr::Vector(-gradGP/length(gradGP));
    R.tcm()->ctrlTasks.deAccess();
    R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(5.0));
     R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.05));
    R.modifyCtrlTaskReference(oriEndeff, -gradGP/length(gradGP));

    arr V;
    TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0));
    arr v;
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,1.0,0.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,0.0,1.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    V = ~V;
    arr Lambda = diag(ARR(0.0,10.0,10.0));
    arr A = V*Lambda*~V;
    arr B = V*Lambda/1.0*~V;
    R.modifyCtrlTaskGains(posEndeff, A, B);
    bool mo = true;
    if(surfaceModel.Y.last() > 0.0) {
      mo = false;
    }
    surfaceModel.threadLoop();
    if(mo) {
      R.interpolateToReference(th, 0.5, newPos);
      R.waitForFinishedTaskReferenceInterpolAct(th);
    } else {
      //R.holdPosition();
    }
  }
}

void test1DRobot() {
  mlr::KinematicWorld world("model.ors");
  world.setJointState(ARR(0.6,-0.1,0.9, 0.0, 0.0, 0.0), zeros(6));
  Object o(world);
  //o.generateObject("b", 0.08, 0.08, 0.1, 0.5, -0.2, 0.7);//mlr::Vector(0.6,-0.2,.65);
  o.generateObject("b", 0.2, 0.2, 0.1, 0.6, -0.1, 0.65);
  //Object table(world);
  //table.generateObject("h", 0.4, 0.4, 0.1, 0.6, -0.2, 0.6);
  Roopi R(world);
  //mlr::wait(30.0);
  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 50.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  //R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);

  //SurfaceVisualisationModule surfaceVisualisation;

  /*arr tX, tY;
  for(uint i = 0; i < 100; i++) {
    tX.append(~table.sampleFromObject());
    tY.append(0.0);
  }
  surfaceModel.X = tX;
  surfaceModel.Y = tY;*/
  surfaceModel.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) -= 0.1;
  pos(0) += 0.1;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  /*pos(0) -= 0.15;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);*/

  //surfaceVisualisation.threadLoop();

  Access<uint> maxIt(NULL, "maxIt");
  maxIt.set()() = 1000000;

  for(uint i = 0; i < maxIt.get()(); i++) {
    arr actPos = R.getTaskValue(posEndeff);
    surfaceModel.threadStop();
    arr gradGP, gradV;
    surfaceModel.gpSurface.writeAccess();
    surfaceModel.gpSurface().gradient(gradGP, actPos);
    surfaceModel.gpSurface().gradientV(gradV, actPos);
    gradV.reshapeFlat();
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);
    surfaceModel.gpSurface.deAccess();
    #if 1
      arr newPos = actPos + 0.1*gradVT;// + 0.1*randn(3);
    #else
      arr randomDirection = randn(3);
      arr gradRandom = (randomDirection - (~gradGP*randomDirection).first()*gradGP/length(gradGP)/length(gradGP));
      gradRandom = gradRandom/length(gradRandom);
      arr newPos = actPos + 0.1*gradRandom;
    #endif
    R.tcm()->ctrlTasks.writeAccess();
    dynamic_cast<TaskMap_Default&>(moveTowardsSurface->map).ivec = mlr::Vector(-gradGP/length(gradGP));
    R.tcm()->ctrlTasks.deAccess();
    R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(5.0));
    R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.05));
    R.modifyCtrlTaskReference(oriEndeff, -gradGP/length(gradGP));

    arr V;
    TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0));
    arr v;
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,1.0,0.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,0.0,1.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    V = ~V;
    arr Lambda = diag(ARR(0.0,10.0,10.0));
    arr A = V*Lambda*~V;
    arr B = V*Lambda/1.0*~V;
    R.modifyCtrlTaskGains(posEndeff, A, B);
    bool mo = true;
    if(surfaceModel.Y.last() > 0.0) {
      mo = false;
    }
    //FILE("XData") << surfaceModel.X;
    //FILE("YData") << surfaceModel.Y;
    surfaceModel.threadLoop();
    if(mo) {
      R.interpolateToReference(th, 0.5, newPos, actPos);
      R.waitForFinishedTaskReferenceInterpolAct(th);
    } else {
      cout << "out" << endl;
      mlr::wait(0.5);
      //R.holdPosition();
    }
  }
  R.holdPosition();
  mlr::wait(1000.0);
}

void test1DRobot2() {
  mlr::KinematicWorld world("model.ors");
  world.setJointState(ARR(0.6,-0.1,0.9, 0.0, 0.0, 0.0), zeros(6));
  Object o(world);
  //o.generateObject("b", 0.08, 0.08, 0.1, 0.5, -0.2, 0.7);//mlr::Vector(0.6,-0.2,.65);
  o.generateObject("b", 0.2, 0.2, 0.1, 0.6, -0.1, 0.65);
  //Object table(world);
  //table.generateObject("h", 0.4, 0.4, 0.1, 0.6, -0.2, 0.6);
  Roopi R(world);
  //mlr::wait(30.0);
  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 50.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap1DPosOrientation(R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  //R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);
  /*arr tX, tY;
  for(uint i = 0; i < 100; i++) {
    tX.append(~table.sampleFromObject());
    tY.append(0.0);
  }
  surfaceModel.X = tX;
  surfaceModel.Y = tY;*/
  surfaceModel.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) -= 0.1;
  pos(0) += 0.1;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  /*pos(0) -= 0.15;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);*/
  for(uint i = 0; i < 1000000; i++) {
    arr actPos = R.getTaskValue(posEndeff);
    surfaceModel.threadStop();
    arr gradGP, gradV;
    surfaceModel.gpSurface.writeAccess();
    surfaceModel.gpSurface().gradient(gradGP, actPos);
    surfaceModel.gpSurface().gradientV(gradV, actPos);
    gradV.reshapeFlat();
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);
    surfaceModel.gpSurface.deAccess();
    arr newPos = actPos + 0.1*gradVT;// + 0.1*randn(3);
    //R.tcm()->ctrlTasks.writeAccess();
    //dynamic_cast<TaskMap_Default&>(moveTowardsSurface->map).ivec = mlr::Vector(-gradGP/length(gradGP));
    //R.tcm()->ctrlTasks.deAccess();
    R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(5.0));
    R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.05));
    R.modifyCtrlTaskReference(oriEndeff, -gradGP/length(gradGP));

    arr V;
    TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0));
    arr v;
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,1.0,0.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,0.0,1.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    V = ~V;
    arr Lambda = diag(ARR(0.0,10.0,10.0));
    arr A = V*Lambda*~V;
    arr B = V*Lambda/1.0*~V;
    R.modifyCtrlTaskGains(posEndeff, A, B);
    bool mo = true;
    if(surfaceModel.Y.last() > 0.0) {
      mo = false;
    }
    FILE(STRING("viel/XData"<<i)) << surfaceModel.X;
    FILE(STRING("viel/YData"<<i)) << surfaceModel.Y;
    surfaceModel.threadLoop();
    if(mo) {
      R.interpolateToReference(th, 0.5, newPos, actPos);
      R.waitForFinishedTaskReferenceInterpolAct(th);
    } else {
      cout << "out" << endl;
      mlr::wait(0.5);
      //R.holdPosition();
    }
  }
}

#if 0
void test1DRobot3() {
  mlr::KinematicWorld world("model.ors");
  world.setJointState(ARR(0.6,-0.1,0.9, 0.0, 0.0, 0.0), zeros(6));
  Object o(world);
  //o.generateObject("b", 0.08, 0.08, 0.1, 0.5, -0.2, 0.7);//mlr::Vector(0.6,-0.2,.65);
  o.generateObject("b", 0.2, 0.2, 0.1, 0.6, -0.1, 0.65);
  //Object table(world);
  //table.generateObject("h", 0.4, 0.4, 0.1, 0.6, -0.2, 0.6);
  Roopi R(world);
  //mlr::wait(30.0);
  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 50.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap1DPosOrientation(R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  //R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);
  /*arr tX, tY;
  for(uint i = 0; i < 100; i++) {
    tX.append(~table.sampleFromObject());
    tY.append(0.0);
  }
  surfaceModel.X = tX;
  surfaceModel.Y = tY;*/
  surfaceModel.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) -= 0.1;
  pos(0) += 0.1;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);
  CtrlTask* oriGP = R.createCtrlTask("oriGP", new TaskMapGPGradientThread(surfaceModel.gpSurface, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(-1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriGP, 60.0, 5.0);
  R.modifyCtrlTaskReference(oriGP, ARR(0.0));
  R.activateCtrlTask(oriGP);
  R.deactivateCtrlTask(oriEndeff);
  //R.tcm()->ctrlTasks.writeAccess();
  //moveTowardsSurface->map = *new TaskMapGP1DThread(surfaceModel.gpSurface, R.tcm()->modelWorld.get()(), "endeffR");
  //R.tcm()->ctrlTasks.deAccess();
  R.deactivateCtrlTask(moveTowardsSurface);
  CtrlTask* moveTowardsSurface2 = R.createCtrlTask("moveTowardsSurface", new TaskMapGP1DThread(surfaceModel.gpSurface, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(moveTowardsSurface2, ARR(0.0), ARR(5.0));
  R.modifyCtrlTaskReference(moveTowardsSurface2, ARR(0.0), ARR(-0.05));
  R.activateCtrlTask(moveTowardsSurface2);
  /*pos(0) -= 0.15;
  R.interpolateToReference(th, 5.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);*/
  for(uint i = 0; i < 1000000; i++) {
    arr actPos = R.getTaskValue(posEndeff);
    surfaceModel.threadStop();
    arr gradGP, gradV;
    surfaceModel.gpSurface.writeAccess();
    surfaceModel.gpSurface().gradient(gradGP, actPos);
    surfaceModel.gpSurface().gradientV(gradV, actPos);
    gradV.reshapeFlat();
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);
    surfaceModel.gpSurface.deAccess();
    arr newPos = actPos + 0.1*gradVT;// + 0.1*randn(3);
    //R.tcm()->ctrlTasks.writeAccess();
    //dynamic_cast<TaskMap_Default&>(moveTowardsSurface->map).ivec = mlr::Vector(-gradGP/length(gradGP));
    //R.tcm()->ctrlTasks.deAccess();
    R.modifyCtrlTaskGains(moveTowardsSurface2, ARR(0.0), ARR(5.0));
    R.modifyCtrlTaskReference(moveTowardsSurface2, ARR(0.0), ARR(-0.05));
    //R.modifyCtrlTaskReference(oriEndeff, -gradGP/length(gradGP));

    arr V;
    TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0));
    arr v;
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,1.0,0.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,0.0,1.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    V = ~V;
    arr Lambda = diag(ARR(0.0,10.0,10.0));
    arr A = V*Lambda*~V;
    arr B = V*Lambda/1.0*~V;
    R.modifyCtrlTaskGains(posEndeff, A, B);
    bool mo = true;
    if(surfaceModel.Y.last() > 0.0) {
      mo = false;
    }
    //FILE("XData") << surfaceModel.X;
    //FILE("YData") << surfaceModel.Y;
    surfaceModel.threadLoop();
    if(mo) {
      R.interpolateToReference(th, 0.5, newPos, actPos);
      R.waitForFinishedTaskReferenceInterpolAct(th);
    } else {
      cout << "out" << endl;
      mlr::wait(0.5);
      //R.holdPosition();
    }
  }
}
#endif

#if 0

void test1DRobot4() {
  mlr::KinematicWorld world("model.ors");
  world.setJointState(ARR(0.6,-0.1,0.9, 0.0, 0.0, 0.0), zeros(6));
  Object o(world);
  //o.generateObject("b", 0.08, 0.08, 0.1, 0.5, -0.2, 0.7);//mlr::Vector(0.6,-0.2,.65);
  o.generateObject("b", 0.2, 0.2, 0.1, 0.6, -0.1, 0.65);
  //Object table(world);
  //table.generateObject("h", 0.4, 0.4, 0.1, 0.6, -0.2, 0.6);
  Roopi R(world);
  //mlr::wait(30.0);
  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 50.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap1DPosOrientation(R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  //R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);
  surfaceModel.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) -= 0.1;
  pos(0) += 0.1;
  //R.interpolateToReference(th, 5.0, pos);
  //R.waitForFinishedTaskReferenceInterpolAct(th);
  mlr::wait(0.5);
  CtrlTask* oriGP = R.createCtrlTask("oriGP", new TaskMapGPGradientThread(surfaceModel.gpSurface, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(-1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriGP, 30.0, 5.0);
  R.modifyCtrlTaskReference(oriGP, ARR(0.0));
  R.activateCtrlTask(oriGP);
  R.deactivateCtrlTask(oriEndeff);
  R.deactivateCtrlTask(moveTowardsSurface);

  /*CtrlTask* moveTowardsSurface2 = R.createCtrlTask("moveTowardsSurface", new TaskMapGP1DThread(surfaceModel.gpSurface, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(moveTowardsSurface2, ARR(0.0), ARR(5.0));
  R.modifyCtrlTaskReference(moveTowardsSurface2, ARR(0.0), ARR(-0.05));
  R.activateCtrlTask(moveTowardsSurface2);*/

  CtrlTask* moveTowardsSurface2 = R.createCtrlTask("moveTowardsSurface", new TaskMap1DPosOrientation(R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface2, ARR(0.0), ARR(5.0));
  R.modifyCtrlTaskReference(moveTowardsSurface2, ARR(0.0), ARR(0.05));
  R.activateCtrlTask(moveTowardsSurface2);

  CtrlTask* moveVariance = R.createCtrlTask("moveVariance", new TaskMapGPVariance1DThread(surfaceModel.gpSurface, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(moveVariance, ARR(0.0), ARR(100.0));
  R.modifyCtrlTaskReference(moveVariance, ARR(0.0), ARR(0.1));
  R.activateCtrlTask(moveVariance);
  mlr::wait(10000.0);
  /*for(uint i = 0; i < 1000000; i++) {
    arr actPos = R.getTaskValue(posEndeff);
    surfaceModel.threadStop();
    arr gradGP, gradV;
    surfaceModel.gpSurface.writeAccess();
    surfaceModel.gpSurface().gradient(gradGP, actPos);
    surfaceModel.gpSurface().gradientV(gradV, actPos);
    gradV.reshapeFlat();
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);
    surfaceModel.gpSurface.deAccess();
    arr newPos = actPos + 0.1*gradVT;
    arr V;
    TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0));
    arr v;
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,1.0,0.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,0.0,1.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    V = ~V;
    arr Lambda = diag(ARR(0.0,10.0,10.0));
    arr A = V*Lambda*~V;
    arr B = V*Lambda/1.0*~V;
    R.modifyCtrlTaskGains(posEndeff, A, B);
    bool mo = true;
    if(surfaceModel.Y.last() > 0.0) {
      mo = false;
    }
    surfaceModel.threadLoop();
    if(mo) {
      R.interpolateToReference(th, 0.5, newPos, actPos);
      R.waitForFinishedTaskReferenceInterpolAct(th);
    } else {
      cout << "out" << endl;
      mlr::wait(0.5);
      //R.holdPosition();
    }
  }*/
}
#endif




void surfaceExploration_1() {
  mlr::KinematicWorld world("model.ors");
  world.setJointState(ARR(0.6,-0.1,0.9, 0.0, 0.0, 0.0), zeros(6));

  Object o(world);
  o.generateObject("b", 0.2, 0.2, 0.1, 0.6, -0.1, 0.65);
  Object ob(world);
  ob.generateObject("a", 0.22, 0.22, 0.12, 0.6, -0.1, 0.65, false);

  Roopi R(world);

  CtrlTask* contactSimulator = R.createCtrlTask("contactSimulator", new TaskMap_Proxy(allPTMT, {0u}, 0.01));
  R.modifyCtrlTaskGains(contactSimulator, 50.0, 10.0);
  R.modifyCtrlTaskReference(contactSimulator, ARR(0.0));
  R.activateCtrlTask(contactSimulator);

  CtrlTask* posEndeff = R.createCtrlTask("posEndeff", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"));
  R.modifyCtrlTaskGains(posEndeff, diag(ARR(10.0,10.0,10.0)), diag(ARR(5.0,5.0,5.0)));
  CtrlTask* oriEndeff = R.createCtrlTask("oriEndeff", new TaskMap_Default(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0)));
  R.modifyCtrlTaskGains(oriEndeff, 20.0, 5.0);
  R.modifyCtrlTaskReference(oriEndeff, ARR(0.0,0.0,-1.0));

  R.activateCtrlTask(posEndeff, true);
  R.activateCtrlTask(oriEndeff);
  R.releasePosition();
  mlr::wait(4.0);
  R.holdPosition();
  R.activateCtrlTask(contactSimulator);

  CtrlTask* moveTowardsSurface = R.createCtrlTask("moveTowardsSurface", new TaskMap_Default(pos1DTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(0.0,0.0,-1.0)));
  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(15.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.1));
  //R.modifyForce(moveTowardsSurface, ARR(-4.0), 0.005, 0.999);

  R.modifyCtrlTaskGains(posEndeff, diag(ARR(30.0,30.0,0.0)), diag(ARR(5.0,5.0,0.0)));
  R.activateCtrlTask(posEndeff);
  R.activateCtrlTask(oriEndeff);
  R.activateCtrlTask(moveTowardsSurface);
  R.releasePosition();

  SurfaceModelAct surfaceModel(R);

  //SurfaceVisualisationModule surfaceVisualisation;

  surfaceModel.threadLoop();

  mlr::wait(2.0);

  TaskReferenceInterpolAct* th = R.createTaskReferenceInterpolAct("interpol", posEndeff);
  arr pos = R.getTaskValue(posEndeff);
  pos(1) -= 0.01;
  pos(0) += 0.01;
  R.interpolateToReference(th, 1.0, pos);
  R.waitForFinishedTaskReferenceInterpolAct(th);

  R.modifyCtrlTaskGains(moveTowardsSurface, ARR(0.0), ARR(5.0));
  R.modifyCtrlTaskReference(moveTowardsSurface, ARR(0.0), ARR(0.05));

  //surfaceVisualisation.threadLoop();

  Access<uint> maxIt(NULL, "maxIt");
  maxIt.set()() = 1000000;

  TaskMap_Default eOri(vecTMT, R.tcm()->modelWorld.get()(), "endeffR", mlr::Vector(1.0,0.0,0.0));

  for(uint i = 0; i < maxIt.get()(); i++) {
    arr actPos = R.getTaskValue(posEndeff);
    //surfaceModel.threadStop();
    arr gradGP, gradV;
    surfaceModel.gpSurface.writeAccess();
    surfaceModel.gpSurface().gradient(gradGP, actPos);
    surfaceModel.gpSurface().gradientV(gradV, actPos);
    surfaceModel.gpSurface.deAccess();

    gradV.reshapeFlat();
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
    gradVT = gradVT/length(gradVT);

    #if 1
      arr newPos = actPos + 0.1*gradVT;
    #else
      arr randomDirection = randn(3);
      arr gradRandom = (randomDirection - (~gradGP*randomDirection).first()*gradGP/length(gradGP)/length(gradGP));
      gradRandom = gradRandom/length(gradRandom);
      arr newPos = actPos + 0.1*gradRandom;
    #endif

    R.tcm()->ctrlTasks.writeAccess();
    dynamic_cast<TaskMap_Default&>(moveTowardsSurface->map).ivec = mlr::Vector(-gradGP/length(gradGP));
    R.tcm()->ctrlTasks.deAccess();
    R.modifyCtrlTaskReference(oriEndeff, -gradGP/length(gradGP));

    arr V;
    arr v;
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,1.0,0.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    eOri.ivec = mlr::Vector(0.0,0.0,1.0);
    eOri.phi(v, NoArr, R.tcm()->modelWorld.get()());
    V.append(~v);
    V = ~V;
    arr Lambda = diag(ARR(0.0,10.0,10.0));
    arr A = V*Lambda*~V;
    arr B = V*Lambda/1.0*~V;
    R.modifyCtrlTaskGains(posEndeff, A, B);

    bool mo = true;
    if(surfaceModel.gpSurface.get()->Y.last() > 0.0) {
      mo = false;
    }
    //surfaceModel.threadLoop();
    if(mo) {
      R.interpolateToReference(th, 0.5, newPos, actPos);
      R.waitForFinishedTaskReferenceInterpolAct(th);
    } else {
      cout << "out" << endl;
      mlr::wait(0.5);
      //R.holdPosition();
    }
  }
  R.holdPosition();
  mlr::wait(1000.0);
}



int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  //tests();
  //testGP();
  //verruecktWennDasKlappt();
  //withRobot();
  //forceTest();
  //forceSimulation();
  //testThread();

  //testExploreSurface();

  //testExploreSurface2();
  //testExploreSurface3();

  //test1DRobot();
  //test1DRobot2();
  //test1DRobot4();
  surfaceExploration_1();

  return 0;
}
