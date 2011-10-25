#include "roboticsCourse.h"
#include "ors.h"
#include "algos.h"
#include "opengl.h"
#include "plot.h"

void drawEnv(void*){ glStandardLight(NULL); glDrawFloor(10., .9, .9, .9); }
void drawBase(void*){ glDrawAxes(1.); }

struct sSimulator {
  ors::Graph G;
  OpenGL gl;
  SwiftInterface swift;
  double margin;
#ifdef MT_ODE
  OdeInterface ode;
#endif
  sSimulator(){ margin=.1; } //default margin = 10cm
};

void Simulator::anchorKinematicChainIn(const char* bodyName){
  s->G.reconfigureRoot(s->G.getBodyByName(bodyName));
  s->G.calcBodyFramesFromJoints();
  
  if(s->swift.isOpen){
    s->swift.close();
    s->swift.init(s->G, .5);
  }
  
#ifdef MT_ODE
  if(s->ode.isOpen){
    s->ode.clear();
    s->ode.createOde(s->G);
  }
#endif
}


Simulator::Simulator(const char* orsFile){
  s = new sSimulator;
  
  //ORS
  s->G.init(orsFile);
  if(s->G.getBodyByName("rfoot")){
    s->G.reconfigureRoot(s->G.getBodyByName("rfoot"));
    s->G.calcBodyFramesFromJoints();
  }
  
  //G.makeLinkTree();
  
  //OPENGL
  s->gl.add(drawEnv, 0);
  s->gl.add(ors::glDrawGraph, &s->G);
  s->gl.setClearColors(1., 1., 1., 1.);
  s->gl.camera.setPosition(10., -15., 8.);
  s->gl.camera.focus(0, 0, 1.);
  s->gl.camera.upright();
  s->gl.update();
  s->gl.add(glDrawPlot, &plotModule);
  
  //SWIFT
  s->swift.init(s->G, .5);
  
  //ODE
#ifdef MT_ODE
  s->ode.createOde(s->G);
#endif
  
  n=s->G.getJointStateDimension();
  
  
}

Simulator::~Simulator(){
  delete s;
}


void Simulator::watch(){
  s->gl.watch();
}

void Simulator::getJointAngles(arr& q){
  s->G.getJointState(q);
}

uint Simulator::getJointDimension(){
  return s->G.getJointStateDimension();
}

void Simulator::setJointAngles(const arr& q, bool updateDisplay){
  s->G.setJointState(q);
  s->G.calcBodyFramesFromJoints();
  listDelete(s->G.proxies);
  s->swift.computeProxies(s->G, false);
  //s->G.sortProxies(true);
  if(updateDisplay) s->gl.update();
}

void Simulator::setJointAnglesAndVels(const arr& q, const arr& qdot){
  s->G.setJointState(q, qdot);
  s->G.calcBodyFramesFromJoints();
  s->swift.computeProxies(s->G, false);
  s->G.sortProxies(true);
  s->gl.update();
}

void Simulator::kinematicsPos(arr& y, const char* bodyName, const arr* rel){
  if(rel){
    ors::Vector v;  v.set(rel->p);
    s->G.kinematics(y, s->G.getBodyByName(bodyName)->index, &v);
  }else{
    s->G.kinematics(y, s->G.getBodyByName(bodyName)->index, NULL);
  }
}

void Simulator::kinematicsVec(arr& y, const char* bodyName, const arr* vec){
  if(vec){
    ors::Vector v;  v.set(vec->p);
    s->G.kinematicsVec(y, s->G.getBodyByName(bodyName)->index, &v);
  }else{
    s->G.kinematicsVec(y, s->G.getBodyByName(bodyName)->index, NULL);
  }
}

void Simulator::jacobianPos(arr& J, const char* bodyName, const arr* rel){
  if(rel){
    ors::Vector v;  v.set(rel->p);
    s->G.jacobian(J, s->G.getBodyByName(bodyName)->index, &v);
  }else{
    s->G.jacobian(J, s->G.getBodyByName(bodyName)->index, NULL);
  }
}

void Simulator::jacobianVec(arr& J, const char* bodyName, const arr* vec){
  if(vec){
    ors::Vector v;  v.set(vec->p);
    s->G.jacobianVec(J, s->G.getBodyByName(bodyName)->index, &v);
  }else{
    s->G.jacobianVec(J, s->G.getBodyByName(bodyName)->index, NULL);
  }
}

void Simulator::kinematicsCOM(arr& y){
  s->G.getCenterOfMass(y);
  y.resizeCopy(2);
}

void Simulator::jacobianCOM(arr& J){
  s->G.getComGradient(J);
  J.resizeCopy(2, J.d1);
}

void Simulator::reportProxies(){
  s->G.reportProxies();
}

void Simulator::setContactMargin(double margin){
  s->margin = margin;
}

double Simulator::kinematicsContacts(){
  arr tmp(1);
  s->G.getContactMeasure(tmp, s->margin);
  return tmp(0);
}

void Simulator::jacobianContacts(arr& grad){
  s->G.getContactGradient(grad, s->margin);
}

void Simulator::getDynamics(arr& M, arr& F, const arr& qdot, bool gravity){
  s->G.clearForces();
  if(gravity) s->G.gravityToForces();
  s->G.equationOfMotion(M, F, qdot);
  F *= -1.; //different convention!!
}

double Simulator::getEnergy(){
  return s->G.getEnergy();
}

void Simulator::stepOde(const arr& qdot, bool updateDisplay){
#ifdef MT_ODE
  s->ode.setMotorVel(s->G, qdot, 100.);
  s->ode.step(0.01);
  s->ode.importStateFromOde(s->G);
#endif
  if(updateDisplay) s->gl.update();
}

struct sVisionSimulator {
  OpenGL gl;
  arr P;
  sVisionSimulator(){ }
};



VisionSimulator::VisionSimulator(){
  s = new sVisionSimulator;
  
  s->P.resize(3, 4);
  
  //OPENGL
  s->gl.add(drawEnv, 0);
  s->gl.add(drawBase, 0);
  s->gl.setClearColors(1., 1., 1., 1.);
  s->gl.camera.setPosition(10., -15., 8.);
  s->gl.camera.focus(0, 0, 0);
  s->gl.update();
  s->gl.add(glDrawPlot, &plotModule);
  
}

VisionSimulator::~VisionSimulator(){
  delete s;
}

void VisionSimulator::watch(){
  s->gl.watch();
}

void VisionSimulator::getRandomWorldPoints(arr& X, uint N){
  //generate N random 3D world points
  X.resize(N, 4);
  rndUniform(X, -1., 1., false);  //each point is random in [-1, 1]^4
  for(uint i=0; i<N; i++){
    X(i, 3)=1.;                 //initialize 4th coordinate to 1
  }
}

arr VisionSimulator::getCameraTranslation(){
  arr t;
  t.setCarray(s->gl.camera.X->pos.p, 3);
  return t;
}

void VisionSimulator::projectWorldPointsToImagePoints(arr& x, const arr& X, double noiseInPixel){
  uint N=X.d0;
  x.resize(N, 3);
  
  //*
  arr y(3);
  arr Mmodel(4, 4), Mproj(4, 4); intA Mview(4);
  glGetDoublev(GL_MODELVIEW_MATRIX, Mmodel.p);
  glGetDoublev(GL_PROJECTION_MATRIX, Mproj.p);
  glGetIntegerv(GL_VIEWPORT, Mview.p);
  //cout <<Mview <<endl;
  //cout <<Mmodel <<endl;
  //cout <<Mproj <<s->P <<endl;
  //*/
  intA view(4);
  glGetIntegerv(GL_VIEWPORT, view.p);
  
  //project the points using the OpenGL matrix
  s->P = s->gl.P;
  s->P /= s->P(0, 0);
  cout <<"VisionSimulator:"
       <<"\n  projection matrix used: " <<s->P
       <<"\n  camera position and quaternion: " <<s->gl.camera.X->pos <<"  " <<s->gl.camera.X->rot
       <<"\n  camera f=" <<.5*view(2) <<" x0=" <<view(0)+.5*view(2) <<" y0=" <<view(1)+.5*view(2)
       <<endl;
  for(uint i=0; i<N; i++){
    x[i] = s->P*X[i];
    x[i]() /= x(i, 2);
    //gluProject(X(i, 0), X(i, 1), X(i, 2), Mmodel.p, Mproj.p, Mview.p, &y(0), &y(1), &y(2));
    //cout <<"y="<< y <<" x=" <<x[i] <<endl;
  }
  rndGauss(x, noiseInPixel, true); //add Gaussian noise
  for(uint i=0; i<N; i++) x(i, 2)=1.;
  
  plotPoints(X);
  //s->gl.watch();
}
