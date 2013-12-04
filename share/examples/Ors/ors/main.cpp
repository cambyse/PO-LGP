#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Algo/spline.h>
#include <Algo/algos.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <GL/gl.h>
#include <Optim/optimization.h>

//===========================================================================
//
// test laod save
//

void TEST(LoadSave){
  ors::Graph G;
  ifstream fil("arm7.ors");
  fil >>G;
  G.calcBodyFramesFromJoints();
  cout <<G <<endl;

  for(uint i=0;i<0;i++){
    OpenGL gl("arm3.ors");
    G.gl().watch();
//    MT::wait(.1)
  }
}


//===========================================================================
//
// Jacobian test
//

namespace T1{
  uint i,j;
  ors::Graph *G;
  ors::Vector rel;
  ors::Vector axis;
  static void f  (arr &y, arr *J, const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->kinematicsPos(y,*J,i,&rel); }
  //static void f_hess (arr &J, arr *H, const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->jacobianPos(J,i,&rel);  if(H) G->hessianPos (*H,i,&rel); }
  static void f_vec (arr &y, arr *J, const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->kinematicsVec(y,*J,i,&axis); }
  //static void f3 (arr &y,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->kinematicsOri2(y,i,axis); }
  //static void df3(arr &J,const arr &x,void*){  G->setJointState(x);  G->calcBodyFramesFromJoints();  G->jacobianOri2(J,i,axis); }
}

void TEST(Kinematics){
  ors::Graph G("arm3.ors");
  uint n=G.getJointStateDimension();
  arr x(n);
  T1::axis.set(1,0,0);
  T1::G = &G;
  for(uint k=0;k<100;k++){
    T1::i=rnd.num(0,G.bodies.N-1);
    T1::rel.setRandom();
    rndUniform(x,-.5,.5,false);
    G.gl().text.clear() <<"k=" <<k <<"  gradient checks of kinematics on random postures";
    //G.gl().update();
    checkJacobian(Convert(T1::f, NULL), x, 1e-5);
    //checkJacobian(Convert(T1::f_hess, NULL), x, 1e-5);
    checkJacobian(Convert(T1::f_vec, NULL), x, 1e-5);
  }
}

//===========================================================================
//
// Kinematic speed test
//

void TEST(KinematicSpeed){
#define NUM 10000
#if 1
  ors::Graph G("arm7.ors");
  G.makeLinkTree();
  uint n=G.getJointStateDimension();
  arr x(n);
  MT::timerStart();
  for(uint k=0;k<NUM;k++){
    rndUniform(x,-.5,.5,false);
    G.setJointState(x);
    G.calcBodyFramesFromJoints();
  }
  cout <<"kinematics timing: "<< MT::timerRead() <<"sec" <<endl;
#endif

  ors::Transformation t,s; t.setRandom(); s.setRandom();
  MT::timerStart();
  for(uint k=0;k<NUM;k++){
    t.appendTransformation(s);
  }
  cout <<"transformation appending: "<< MT::timerRead() <<"sec" <<endl;

  ors::Matrix A,B,Y; A.setRandom(); B.setRandom();
  ors::Vector a,b,y; a.setRandom(); b.setRandom();
  MT::timerStart();
  for(uint k=0;k<NUM;k++){
    Y=A*B;
    y=a+A*b;
    a=y;
    A=Y;
  }
  cout <<"matrix timing: "<< MT::timerRead() <<"sec" <<endl;
}

//===========================================================================
//
// SWIFT and contacts test
//

namespace Ctest{
  ors::Graph *G;
  void f(arr& c, arr *dfdx, const arr &x,void*){
    G->setJointState(x); G->calcBodyFramesFromJoints();
    G->swift().computeProxies(*G,false);
    G->kinematicsProxyCost(c, (dfdx?*dfdx:NoArr), .2);
  }
}

void TEST(Contacts){
  ors::Graph G("arm7.ors");
  
  arr x,v,con,grad;
  uint t;

  G.swift().setCutoff(.5);

  Ctest::G=&G;

  G.getJointState(x,v);
  G.gl().text <<"testing the contact gradient";
  for(t=0;t<100;t++){
    G.setJointState(x);
    G.calcBodyFramesFromJoints();
    G.swift().computeProxies(G,false);

    G.reportProxies();

    G.kinematicsProxyCost(con, grad, .2);
    cout <<"contact meassure = " <<con(0) <<endl;
    G.gl().text.clear() <<"t=" <<t <<"  movement along negative contact gradient (using SWIFT to get contacts)";
    //G.gl().watch();
    G.gl().update();
    //x += inverse(grad)*(-.1*c);
    x -= 1e-3*grad; //.1 * (invJ * grad);

    checkJacobian(Convert(Ctest::f, NULL), x, 1e10);
  }
}


//===========================================================================
//
// set state test
//

void generateSequence(arr &X, uint T, uint n){
  rnd.seed(0);
  arr P(10,n);

  //a random spline
  //a set of random via points with zero start and end:
  rndUniform(P,-1.,1.,false); P[0]=0.; P[P.d0-1]=0.;
  
  //convert into a smooth spline (1/0.03 points per via point):
  X = MT::Spline(T,P).eval();
}

void TEST(PlayStateSequence){
  ors::Graph G("arm7.ors");
  uint n=G.getJointStateDimension();
  arr X;
  generateSequence(X, 200, n);
  arr v(X.d1); v=0.;
  for(uint t=0;t<X.d0;t++){
    G.setJointState(X[t](),v);
    G.calcBodyFramesFromJoints();
    G.gl().text.clear() <<"replay of a state sequence -- time " <<t;
    G.gl().timedupdate(0.01);
  }
}

//===========================================================================
//
// ODE test
//

#ifdef MT_ODE
void TEST(PlayTorqueSequenceInOde){
  ors::Graph G("arm7.ors");
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
    G.gl().text.clear() <<"play a random torque sequence [using ODE] -- time " <<t;
    G.gl().timedupdate(.01);
  }
}

void TEST(MeshShapesInOde){
  ors::Graph G("testOdeMesh.ors");
  OdeInterface ode;
  ode.createOde(G);
  for(uint t=0;t<1000;t++){
    //G.clearJointErrors(); exportStateToOde(C,); //try doing this without clearing joint errors...!
    ode.step(0.03);
    ode.importStateFromOde(G);
    G.gl().timedupdate(.01);
  }
}
#endif


//===========================================================================
//
// standard IK test
//

void TEST(FollowRedundantSequence){  
  ors::Graph G("arm7.ors");
  //init();
  uint N=G.bodies.N-2;

  uint t,T,n=G.getJointStateDimension();
  arr x(n),v,z,J,invJ;
  x=.8;     //initialize with intermediate joint positions (non-singular positions)
  ors::Vector rel(0,0,.3); //this frame describes the relative position of the endeffector wrt. 7th body

  //-- generate a random endeffector trajectory
  arr Z,Zt; //desired and true endeffector trajectories
  generateSequence(Z, 200, 3); //3D random sequence with limits [-1,1]
  Z *= .8;
  T=Z.d0;
  G.setJointState(x);
  G.calcBodyFramesFromJoints();
  G.kinematicsPos(z, NoArr, N, &rel);
  for(t=0;t<T;t++) Z[t]() += z; //adjust coordinates to be inside the arm range
  plotLine(Z);
  G.gl().add(glDrawPlot,&plotModule);
  G.gl().update();
  //-- follow the trajectory kinematically
  for(t=0;t<T;t++){
    //Z[t] is the desired endeffector trajectory
    //x is the full joint state, z the endeffector position, J the Jacobian
    G.kinematicsPos(z, J, N, &rel);  //get the new endeffector position
    invJ = inverse(J);       //pseudo inverse
    v = invJ * (Z[t]-z);     //multiply endeffector velocity with inverse jacobian
    x += v;                  //simulate a time step (only kinematically)
    G.setJointState(x);
    G.calcBodyFramesFromJoints();
    //cout <<J * invJ <<invJ <<v <<endl <<x <<endl <<"tracking error = " <<maxDiff(Z[t],z) <<endl;
    G.gl().text.clear() <<"follow redundant trajectory -- time " <<t;
    G.gl().update();
    //G.gl().timedupdate(.01);
  }
}


//===========================================================================
//
// dynamics test
//

//namespace T2{
//  bool friction;
//  arr tau;
//  //static arr conswit;
//  //bool hasContact=false;
//  bool addContactsToDynamics=false;
//  ors::Graph *G;
//}


//---------- test standard dynamic control
void TEST(Dynamics){
  ors::Graph G("arm7.ors");
  //G.makeLinkTree();
  cout <<G <<endl;

  struct DiffEqn:VectorFunction{
    ors::Graph& G;
    arr u;
    bool friction;
    DiffEqn(ors::Graph& _G):G(_G),friction(false){}
    void fv(arr& y,arr&,const arr& x){
      G.setJointState(x[0],x[1]);
      G.calcBodyFramesFromJoints();
      if(!u.N) u.resize(x.d1).setZero();
      if(friction) u = -10. * x[1];
      G.clearForces();
      G.gravityToForces();
      /*if(T2::addContactsToDynamics){
        G.contactsToForces(100.,10.);
      }*/
      G.dynamics(y, x[1], u);
    }
  } diffEqn(G);

  
  uint t,T=720,n=G.getJointStateDimension();
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
    if(t>=500){ //hold steady
      qdd_ = -1. * qd;
      G.inverseDynamics(diffEqn.u, qd, qdd_);
      //tau.resize(n); tau.setZero();
      //G.clearForces();
      //G.gravityToForces();
      G.dynamics(qdd, qd, diffEqn.u);
      CHECK(maxDiff(qdd,qdd_,0)<1e-5,"dynamics and inverse dynamics inconsistent");
      //cout <<q <<qd <<qdd <<endl;
      cout <<"test dynamics: fwd-inv error =" <<maxDiff(qdd,qdd_,0) <<endl;
      q  += .5*dt*qd;
      qd +=    dt*qdd;
      q  += .5*dt*qd;
      G.setJointState(q,qd);
      G.calcBodyFramesFromJoints();
      //cout <<q <<qd <<qdd <<endl;
      G.gl().text.clear() <<"t=" <<t <<"  torque controlled damping (acc = - vel)\n(checking consistency of forward and inverse dynamics),  energy=" <<G.getEnergy();
    }else{
      //cout <<q <<qd <<qdd <<' ' <<G.getEnergy() <<endl;
      arr x=cat(q,qd).reshape(2,q.N);
      MT::rk4_2ndOrder(x, x, diffEqn, dt);
      q=x[0]; qd=x[1];
      if(t>300){
        diffEqn.friction=true;
        G.gl().text.clear() <<"t=" <<t <<"  friction swing using RK4,  energy=" <<G.getEnergy();
      }else{
        diffEqn.friction=false;
        G.gl().text.clear() <<"t=" <<t <<"  free swing using RK4,  energy=" <<G.getEnergy();
      }
    }
    G.gl().update();
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
void TEST(ContactDynamics){
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
      //G.gl().watch();
    }else{
      dt=.01;
      addContactsToDynamics=false;
    }
    cross=MT::rk4dd_switch(q,qd,s,q,qd,s,ddf_joints,switchfunction,dt,1e-4);
    //G.reportProxies();
    cout <<"*** s = " <<s <<endl;
    G.gl().text.clear() <<"t=" <<t <<"  using RK4_switch,  energy=" <<G.getEnergy();
    //if(cross) G.gl().watch();
    G.gl().update();
  }
}*/

//===========================================================================
//
// blender import test
//

#if 0
static void drawTrimesh(void* _mesh){
#if MT_GL
  ors::Mesh *mesh=(ors::Mesh*)_mesh;
  glPushMatrix();
  mesh->glDraw();
  glPopMatrix();
#endif
}

void TEST(BlenderImport){
  MT::timerStart();
  ors::Mesh mesh;
  ors::Graph bl;
  readBlender("blender-export",mesh,bl);
  cout <<"loading time =" <<MT::timerRead() <<"sec" <<endl;
  OpenGL gl;
  G.gl().add(glStandardScene, NULL);
  G.gl().add(drawTrimesh,&mesh);
  G.gl().watch("mesh only");
  G.gl().add(ors::glDrawGraph,&bl);
  G.gl().text="testing blender import";
  animateConfiguration(bl,gl);
}
#endif

int MAIN(int argc,char **argv){
  testLoadSave();
  testPlayStateSequence();
  testKinematics();
  testKinematicSpeed();
  testFollowRedundantSequence();
  testDynamics();
  testContacts();
#ifdef MT_ODE
  testMeshShapesInOde();
  testPlayTorqueSequenceInOde();
#endif
  //testBlenderImport();

  return 0;
}
