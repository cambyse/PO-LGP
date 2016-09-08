#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Algo/gaussianProcess.h>
#include <Geo/mesh.h>
#include <Gui/opengl.h>

#include "taskMapVariance.h"

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

  gp.dcov = dGaussKernel;
  gp.covF_D = GaussKernelF_D;
  gp.covD_D = GaussKernelD_D;
  gp.appendObservation(~ARR(0.0,0.0,0),-1.0);
  gp.appendObservation(~ARR(1.0,0.0,0),-1.0);
  gp.appendObservation(~ARR(1.0,1.0,0),-1.0);
  gp.appendObservation(~ARR(0.0,1.0,0),-1.0);
  gp.appendObservation(~ARR(0.0,0.0,-1.0),-1.0);
  gp.appendObservation(~ARR(1.0,0.0,-1.0),-1.0);
  gp.appendObservation(~ARR(1.0,1.0,-1.0),-1.0);
  gp.appendObservation(~ARR(0.0,1.0,-1.0),-1.0);

  //gp.appendObservation(~ARR(0.0,3.0,-1.0),1.0);
  //gp.appendObservation(~ARR(1.0,3.0,-1.0),1.0);
  //gp.appendObservation(~ARR(0.0,3.0,1.0),1.0);

  gp.recompute();
  arr y, v;
  gp.evaluate(~ARR(0.,0.,0.0), y,v);
  cout << v << endl;

  /*ScalarFunction blobby = [&gp](arr&,arr&, const arr& X){
    arr y, non;
    gp.evaluate(~X, y, non);
    return y.first();
  };

  ors::Mesh m;
  world.gl().add(m);
  m.setImplicitSurface(blobby,-50,50);*/

  TaskMap* taskMap = new TaskMapVariance(gp, world, "endeff");

  t = MP.addTask("super", taskMap, sumOfSqrTT);
  t->map.order = 2;
  t->setCostSpecs(0, MP.T, {0.5}, 10.0);

  TaskMap* ori = new TaskMapGPGradient(gp, world, "endeff", ors::Vector(1.0,0.0,0.0));
  t = MP.addTask("orie", ori, sumOfSqrTT);
  t->setCostSpecs(0, MP.T, {0.0}, 1.0);

  //t = MP.addTask("bla", new TaskMap_Default(vecTMT, world, "endeff", ors::Vector(0.0,0.0,1.0)), sumOfSqrTT);
  //t->setCostSpecs(0, MP.T, ARR(0.0,0.0,1.0), 2.0);

  t = MP.addTask("bla", new TaskMap_Default(posTMT, world, "endeff"), sumOfSqrTT);
  t->setCostSpecs(MP.T-5, MP.T, ARR(-0.1,2.0,0.0), 5.0);

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
    mlr::wait(0.1);
  }
  world.setJointState(x[x.d0-1]);
  world.watch(true);
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  //tests();
  //testGP();
  verruecktWennDasKlappt();
  return 0;
}
