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
  double dynamicNoise;
  bool gravity;
  
  //state
  arr q,qdot,qddot;

#ifdef MT_ODE
  OdeInterface ode;
#endif
  sSimulator(){ margin=.1; dynamicNoise=0.; gravity=true; } //default margin = 10cm
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

  s->G.getJointState(s->q, s->qdot);

  n=s->G.getJointStateDimension();
}

Simulator::~Simulator(){
  delete s;
}


void Simulator::watch(bool pause){
  if(pause) s->gl.watch();
  else s->gl.update();
}

void Simulator::getJointAngles(arr& q){
  s->G.getJointState(q);
}

void Simulator::getJointAnglesAndVels(arr& q, arr& qdot){
  s->G.getJointState(q, qdot);
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
  if(&q!=&s->q) s->q = q;
  s->qdot.setZero();
}

void Simulator::setJointAnglesAndVels(const arr& q, const arr& qdot){
  s->G.setJointState(q, qdot);
  s->G.calcBodyFramesFromJoints();
  s->swift.computeProxies(s->G, false);
  s->G.sortProxies(true);
  s->gl.update();
  if(&q!=&s->q) s->q = q;
  if(&qdot!=&s->qdot) s->qdot = qdot;
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

void Simulator::kinematicsContacts(arr& y){
  s->G.getContactMeasure(y, s->margin);
}

void Simulator::jacobianContacts(arr& J){
  s->G.getContactGradient(J, s->margin);
}

void Simulator::getDynamics(arr& M, arr& F){
  s->G.clearForces();
  if(s->gravity) s->G.gravityToForces();
  s->G.equationOfMotion(M, F, s->qdot);
  F *= -1.; //different convention!!
}

double Simulator::getEnergy(){
  return s->G.getEnergy();
}


void Simulator::stepDynamic(const arr& u_control, double tau){
  arr M,Minv,F;

  getDynamics(M, F);

  inverse(Minv,M);
  s->qddot = Minv * (u_control - F);
    
  if(s->dynamicNoise) rndGauss(s->qddot, s->dynamicNoise, true);

  //Euler integration (Runge-Kutte4 would be much more precise...)
  s->q    += tau * s->qdot;
  s->qdot += tau * s->qddot;
  setJointAnglesAndVels(s->q, s->qdot);
}
  
void Simulator::setDynamicSimulationNoise(double noise){
  s->dynamicNoise = noise;
}

void Simulator::setDynamicGravity(bool gravity){
  s->gravity = gravity;
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
  s->gl.camera.upright();
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
    //cout <<"y=" <<y <<" x=" <<x[i] <<endl;
  }
  rndGauss(x, noiseInPixel, true); //add Gaussian noise
  for(uint i=0; i<N; i++) x(i, 2)=1.;
  
  plotPoints(X);
  //s->gl.watch();
}

void glDrawCarSimulator(void *classP);

CarSimulator::CarSimulator(){
  //car parameters
  x=y=theta=0;
  tau=1.; //one second time steps
  L=2.; //2 meters between the wheels
  dynamicsNoise = .03;
  observationNoise = .5;

  //landmarks
  landmarks.resize(2,2);
  rndGauss(landmarks, 10.);
  //landmarks=ARR(10,0); landmarks.reshape(1,2);
  
  gl=new OpenGL;
  gl->add(drawEnv, this);
  gl->add(glDrawCarSimulator, this);
  gl->add(glDrawPlot,&plotModule);

  gl->camera.setPosition(10., -50., 100.);
  gl->camera.focus(0, 0, .5);
  gl->camera.upright();
  gl->update();
}

void CarSimulator::step(const arr& u){
  double v=u(0), phi=u(1);
  x += tau*v*cos(theta);
  y += tau*v*sin(theta);
  theta += tau*(v/L)*tan(phi);

  if(dynamicsNoise){
    x += dynamicsNoise*rnd.gauss();
    y += dynamicsNoise*rnd.gauss();
    theta += dynamicsNoise*rnd.gauss();
  }
  
  plotClear();
  for(uint i=0;i<gaussiansToDraw.N;i++) plotCovariance(gaussiansToDraw(i).a, gaussiansToDraw(i).A);
  gl->update();
}

void CarSimulator::getRealNoisyObservation(arr& Y){
  getMeanObservationAtState(Y, ARR(x,y,theta));
  rndGauss(Y,observationNoise,true);
}

void CarSimulator::getMeanObservationAtState(arr& Y, const arr& X){
  Y=landmarks;
  arr R = ARR(cos(X(2)), -sin(X(2)), sin(X(2)), cos(X(2)));
  R.reshape(2,2);
  arr p = ones(landmarks.d0,1)*~ARR(X(0),X(1));
  Y -= p;
  Y = Y*R;
  Y.reshape(Y.N);
}

void CarSimulator::getLinearObservationModelAtState(arr& C, arr& c, const arr& X){
  uint N=landmarks.d0;
  arr R = ARR(cos(X(2)), sin(X(2)), -sin(X(2)), cos(X(2)));
  R.reshape(2,2);
  C.resize(2*N,2*N);  C.setZero();
  for(uint i=0;i<N;i++) C.setMatrixBlock(R, 2*i, 2*i);
  cout <<C <<endl;
  c.resize(2*N);
  for(uint i=0;i<N;i++) c.setVectorBlock(ARR(X(0),X(1)), 2*i);
  c = - C * c;
}

void CarSimulator::getObservationJacobianAtState(arr& dy_dx, const arr& X){
  uint N=landmarks.d0;
  dy_dx = arr(2*N,3); dy_dx.setZero();
  for (uint i=0; i<N; i++){
    arr J(2,3);J.setZero();
    //by x
    J(0,0) = -cos(X(2));
    J(1,0) = sin(X(2));
    //by y
    J(0,1) = -sin(X(2));
    J(1,1) = -cos(X(2));
    //by theta
    J(0,2) = -sin(X(2))*(landmarks(i,0)-X(0)) + cos(X(2))*(landmarks(i,1)-X(1));
    J(1,2) = -cos(X(2))*(landmarks(i,0)-X(0)) - sin(X(2))*(landmarks(i,1)-X(1));
    dy_dx[i*2] = J[0];//copy in big J
    dy_dx[i*2+1] = J[1];
  }
}

void glDrawCarSimulator(void *classP){
  CarSimulator *s=(CarSimulator*)classP;
  ors::Transformation f;
  f.addRelativeTranslation(s->x,s->y,.3);
  f.addRelativeRotationRad(s->theta, 0., 0., 1.);
  f.addRelativeTranslation(1.,0.,0.);

  double GLmatrix[16];
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8,.2,.2);
  glDrawBox(3., 1.5, .5);
  
  for(uint l=0;l<s->landmarks.d0;l++){
    f.setZero();
    f.addRelativeTranslation(s->landmarks(l,0),s->landmarks(l,1), .5);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(.2,.8,.2);
    glDrawCylinder(.1,1.);
  }
  
  glLoadIdentity();
  glColor(.2,.2,.8);
  for(uint l=0;l<s->particlesToDraw.d0;l++){
    glPushMatrix();
    glTranslatef(s->particlesToDraw(l,0), s->particlesToDraw(l,1), .6);
    glDrawDiamond(.1, .1, .1);
    glPopMatrix();
  }

  for(uint l=0;l<s->particlesToDraw.d0;l++){
  }
}

template MT::Array<Gaussian>& MT::Array<Gaussian>::resize(uint);