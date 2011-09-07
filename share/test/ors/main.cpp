#include<MT/ors.h>
#include<MT/algos.h>
#include<MT/opengl.h>
#include<MT/plot.h>
#include <GL/gl.h>

void drawEnv(void*){  glStandardLight(NULL); }

void init(ors::Graph& G,OpenGL& gl,const char* orsFile){
  G.init(orsFile);
  //G.makeLinkTree();
  gl.add(drawEnv,0);
  gl.add(ors::glDrawGraph,&G);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.update();
}


//===========================================================================
//
// Jacobian test
//

namespace T1{
  uint i,j;
  ors::Graph *G;
  ors::Transformation rel;
  ors::Vector axis;
  static void f  (arr &y,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->kinematics(y,i,&rel); }
  static void df (arr &J,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->jacobian(J,i,&rel); }
  static void ddf(arr &H,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->hessian(H,i,&rel); }
  static void f2 (arr &y,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->kinematicsZ(y,i,&rel); }
  static void df2(arr &J,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->jacobianZ(J,i,&rel); }
  //static void f3 (arr &y,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->kinematicsOri2(y,i,axis); }
  //static void df3(arr &J,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->jacobianOri2(J,i,axis); }
}
void testKinematics(){
  ors::Graph G;
  OpenGL gl;
  init(G,gl,"arm3.ors");
  uint n=G.getJointStateDimension();
  arr x(n);
  T1::axis.set(1,0,0);
  T1::G = &G;
  MT::timerStart();
  for(uint k=0;k<100;k++){
    T1::i=rnd.num(0,G.bodies.N-1);
    T1::rel.setRandom();
    rndUniform(x,-.5,.5,false);
    gl.text.clr() <<"k=" <<k <<"  gradient checks of kinematics on random postures";
    //gl.update();
    MT::checkGradient(T1::f ,T1::df ,NULL,x,1e-5);
    MT::checkGradient(T1::df,T1::ddf,NULL,x,1e-5);
    MT::checkGradient(T1::f2,T1::df2,NULL,x,1e-5);
    //MT::checkGradient(T1::f3,T1::df3,NULL,x,1e-5);
  }
  cout <<"kinematics timing: "<< MT::timerRead() <<"sec" <<endl;
}

//===========================================================================
//
// SWIFT and contacts test
//

namespace Ctest{
  SwiftInterface *swift;
  ors::Graph *G;
  void f(arr& c,const arr &x,void*){     G->setJointState(x); G->calcBodyFramesFromJoints();  swift->computeProxies(*G,false); G->sortProxies(true); G->getContactMeasure(c,.1); }
  void df(arr& dfdx,const arr &x,void*){ G->setJointState(x); G->calcBodyFramesFromJoints();  swift->computeProxies(*G,false); G->sortProxies(true); G->getContactGradient(dfdx,.1); }
}

void testContacts(){
  ors::Graph G;
  OpenGL gl;
  init(G,gl,"arm7.ors");
  
  arr x,v,con,grad;
  double c;
  uint t;

  SwiftInterface swift;
  swift.init(G,.5);

  Ctest::G=&G;  Ctest::swift=&swift;

  G.getJointState(x,v);
  gl.text <<"testing the contact gradient";
  for(t=0;t<100;t++){
    G.setJointState(x);
    G.calcBodyFramesFromJoints();
    swift.computeProxies(G,false);
    G.sortProxies(true);

    G.reportProxies();

    G.getContactMeasure(con,.2);
    c=G.getContactGradient(grad,.2); //generate a gradient pushing away to 20cm distance
    cout <<"contact meassure = " <<con(0) <<' ' <<c <<endl;
    gl.text.clr() <<"t=" <<t <<"  movement along negative contact gradient (using SWIFT to get contacts)";
    //gl.watch();
    gl.update();
    //x += inverse(grad)*(-.1*c);
    x -= 1e-4*grad; //.1 * (invJ * grad);

    MT::checkGradient(Ctest::f,Ctest::df,NULL,x,1e10);
  }
}


//===========================================================================
//
// set state test
//

void generateSequence(arr &X,arr &V,uint n){
  uint i;
  rnd.seed(0);
  arr P(10,n);
  switch(0){
  case 0:
    //a random spline
    //a set of random via points with zero start and end:
    rndUniform(P,-1.,1.,false); P[0]=0.; P[P.d0-1]=0.; 
    break;
  case 1:
    //a sinus sequence
    for(i=0;i<P.d0;i++) P[i] = -5.*sin(4*i);
    break;
  }
  
  //convert into a smooth spline (1/0.03 points per via point):
  MT::makeSpline(X,V,P,(int)(1/0.03));
}

void testPlayStateSequence(){
  ors::Graph G;
  OpenGL gl;
  init(G,gl,"arm7.ors");
  uint n=G.getJointStateDimension();
  arr X,V;
  generateSequence(X,V,n);
  arr v(X.d1); v=0.;
  for(uint t=0;t<X.d0;t++){
    G.setJointState(X[t](),v);
    G.calcBodyFramesFromJoints();
    gl.text.clr() <<"replay of a state sequence -- time " <<t;
    gl.timedupdate(0.01);
  }
}

//===========================================================================
//
// ODE test
//

#ifdef MT_ODE
void testPlayTorqueSequenceInOde(){
  ors::Graph G;
  OpenGL gl;
  init(G,gl,"arm7.ors");
  OdeInterface ode;
  ode.createOde(G);
  uint n=G.getJointStateDimension();
  arr F,dF,Xt,Vt;
  generateSequence(F,dF,n);
  F *= .01;
  Xt.resizeAs(F); Vt.resizeAs(F);
  for(uint t=0;t<F.d0;t++){
    ode.addJointForce(G,F[t]());
    //G.clearJointErrors(); exportStateToOde(C,); //try doing this without clearing joint errors...!
    ode.step(0.03);
    ode.importStateFromOde(G);
    G.getJointState(Xt[t](),Vt[t]());
    gl.text.clr() <<"play a random torque sequence [using ODE] -- time " <<t;
    gl.timedupdate(.01);
  }
}
#endif


//===========================================================================
//
// standard IK test
//

void testFollowRedundantSequence(){  
  ors::Graph G;
  OpenGL gl;
  init(G,gl,"arm7.ors");
  //init();
  uint N=G.bodies.N-2;

  uint t,T,n=G.getJointStateDimension();
  arr x(n),v,z,J,invJ;
  x=.8;     //initialize with intermediate joint positions (non-singular positions)
  ors::Transformation rel;
  rel.pos.set(0,0,.3); //this frame describes the relative position of the endeffector wrt. 7th body

  //-- generate a random endeffector trajectory
  arr Z,Zt; //desired and true endeffector trajectories
  arr V;
  generateSequence(Z,V,3); //3D random sequence with limits [-1,1]
  T=Z.d0;
  G.setJointState(x);
  G.calcBodyFramesFromJoints();
  G.kinematics(z,N,&rel);
  for(t=0;t<T;t++) Z[t]() += z; //adjust coordinates to be inside the arm range
  plotLine(Z);
  gl.add(glDrawPlot,&plotModule);
  gl.update();
  //-- follow the trajectory kinematically
  for(t=0;t<T;t++){
    //Z[t] is the desired endeffector trajectory
    //x is the full joint state, z the endeffector position, J the Jacobian
    G.jacobian(J,N,&rel);  //get the Jacobian wrt. the 7th body (endeffector)
    invJ = inverse(J);       //pseudo inverse
    v = invJ * (Z[t]-z);     //multiply endeffector velocity with inverse jacobian
    x += v;                  //simulate a time step (only kinematically)
    G.setJointState(x);
    G.calcBodyFramesFromJoints();
    G.kinematics(z,N,&rel);  //get the new endeffector position
    //cout <<J * invJ <<invJ <<v <<endl <<x <<endl <<"tracking error = " <<maxDiff(Z[t],z) <<endl;
    gl.text.clr() <<"follow redundant trajectory -- time " <<t;
    gl.update();
    //gl.timedupdate(.01);
  }
}


//===========================================================================
//
// dynamics test
//

namespace T2{
  bool friction;
  arr tau;
  //static arr conswit;
  //bool hasContact=false;
  bool addContactsToDynamics=false;
  ors::Graph *G;
}

void ddf_joints(arr& xdd,const arr& x,const arr& v){
  T2::G->setJointState(x,v);
  T2::G->calcBodyFramesFromJoints();
  if(!T2::tau.N){ T2::tau.resize(x.N); T2::tau=0.; }
  if(T2::friction) T2::tau = -.01 * v;
  xdd.resize(x.N);
  T2::G->clearForces();
  T2::G->gravityToForces();
  if(T2::addContactsToDynamics){
    T2::G->contactsToForces(100.,10.);
  }
  T2::G->dynamics(xdd,v,T2::tau);
}

//---------- test standard dynamic control
void testDynamics(){
  ors::Graph G;
  OpenGL gl;
  init(G,gl,"arm7.ors");
  T2::G=&G;
  
  uint t,T=1200,n=G.getJointStateDimension();
  arr q,qd,qdd(n),qdd_(n);
  G.getJointState(q,qd);
  qdd.setZero();
  
  double dt=.01;

  ofstream z("z.dyn");
  G.clearForces();

  for(t=0;t<T;t++){
    if(false && !(t%1)){
      G.setJointState(q,qd);
      G.calcBodyFramesFromJoints();
      G.zeroGaugeJoints();
      G.calcBodyFramesFromJoints();
      G.getJointState(q,qd);
    }
    if(t>=1000){ //hold steady
      qdd_ = -1. * qd;
      G.inverseDynamics(T2::tau,qd,qdd_);
      //tau.resize(n); tau.setZero();
      //G.clearForces();
      //G.gravityToForces();
      G.dynamics(qdd,qd,T2::tau);
      CHECK(maxDiff(qdd,qdd_,0)<1e-5,"dynamics and inverse dynamics inconsistent");
      //cout <<q <<qd <<qdd <<endl;
      cout <<"test dynamics: fwd-inv error =" <<maxDiff(qdd,qdd_,0) <<endl;
      q  += .5*dt*qd;
      qd +=    dt*qdd;
      q  += .5*dt*qd;
      G.setJointState(q,qd);
      G.calcBodyFramesFromJoints();
      //cout <<q <<qd <<qdd <<endl;
      gl.text.clr() <<"t=" <<t <<"  torque controlled damping (acc = - vel)\n(checking consistency of forward and inverse dynamics),  energy=" <<G.getEnergy();
    }else{
      //cout <<q <<qd <<qdd <<' ' <<G.getEnergy() <<endl;
      MT::rk4dd(q,qd,q,qd,ddf_joints,dt);
      if(t>500){
        T2::friction=true;
        gl.text.clr() <<"t=" <<t <<"  friction swing using RK4,  energy=" <<G.getEnergy();
      }else{
        T2::friction=false;
        gl.text.clr() <<"t=" <<t <<"  free swing using RK4,  energy=" <<G.getEnergy();
      }
    }
    gl.update();
  }
}

/*void switchfunction(arr& s,const arr& x,const arr& v){
  G.setJointState(x,v);
  G.calcBodyFramesFromJoints();
  slGetProxies(C,ode);
  s.resize(G.bodies.N); s=.01;
  boolA c; c.resize(G.bodies.N);  c=false;
  uint i;
  int a,b;
  hasContact=false;
  for(i=0;i<G.proxies.N;i++) if(!G.proxies(i)->age){
    a=G.proxies(i)->a; b=G.proxies(i)->b;
    if(a>=0){ s(a) += G.proxies(i)->d; c(a) = true; }
    if(b>=0){ s(b) += G.proxies(i)->d; c(b) = true; }
    hasContact=true;
  }
  //for(i=0;i<s.N;i++) if(!c(i)) s(i)=.1;
}

bool checkContacts(const arr& s){
  uint i;
  for(i=0;i<s.N;i++) if(s(i)<0.) return true;
  return false;
}

//---------- test standard redundant control
void testContactDynamics(){
  init();

  uint t,T=1000,n=G.getJointStateDimension();
  arr q,qd,qdd(n),qdd_(n),s;
  G.getJointState(q,qd);
  switchfunction(s,q,qd);

  double dt=.001;
  bool cross;

  ofstream z("z.dyn");
  G.clearForces();

  for(t=0;t<T;t++){
    if(!(t%1)){
      G.setJointState(q,qd);
      G.calcBodyFramesFromJoints();
      G.zeroGaugeJoints();
      G.calcBodyFramesFromJoints();
      G.getJointState(q,qd);
    }
    z <<q <<qd <<qdd <<endl;
    conswit=s;
    if(false && checkContacts(s)){
      dt=.001;
      addContactsToDynamics=true;
      //gl.watch();
    }else{
      dt=.01;
      addContactsToDynamics=false;
    }
    cross=MT::rk4dd_switch(q,qd,s,q,qd,s,ddf_joints,switchfunction,dt,1e-4);
    //G.reportProxies();
    cout <<"*** s = " <<s <<endl;
    gl.text.clr() <<"t=" <<t <<"  using RK4_switch,  energy=" <<G.getEnergy();
    //if(cross) gl.watch();
    gl.update();
  }
}*/

//===========================================================================
//
// blender import test
//

static void drawTrimesh(void* _mesh){
  ors::Mesh *mesh=(ors::Mesh*)_mesh;
  glPushMatrix();
  ors::glDraw(*mesh);
  glPopMatrix();
}

void testBlenderImport(){
  MT::timerStart();
  ors::Mesh mesh;
  ors::Graph bl;
  readBlender("blender-export",mesh,bl);
  cout <<"loading time =" <<MT::timerRead() <<"sec" <<endl;
  OpenGL gl;
  gl.add(drawEnv,0);
  gl.add(drawTrimesh,&mesh);
  gl.watch("mesh only");
  gl.add(ors::glDrawGraph,&bl);
  gl.text="testing blender import";
  animateConfiguration(bl,gl);
}

int main(int argc,char **argv){

  testBlenderImport();

  testPlayStateSequence();
  testKinematics();
  testFollowRedundantSequence();
  testDynamics();
  testContacts();
#ifdef MT_ODE
  testPlayTorqueSequenceInOde();
#endif
  testBlenderImport();

  return 0;
}