#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Ors/ors_ode.h>
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
  ors::KinematicWorld G;
  FILE("arm7.ors") >>G;
  cout <<G <<endl;

  for(uint i=0;i<0;i++){
    G.watch(true);
//    MT::wait(.1)
  }
}


//===========================================================================
//
// Jacobian test
//

void TEST(Kinematics){
  struct MyFct:VectorFunction{
    enum Mode {Pos, Vec, Quat} mode;
    ors::KinematicWorld& W;
    ors::Body *b;
    ors::Vector& vec;
    MyFct(Mode _mode, ors::KinematicWorld &_W, ors::Body *_b, ors::Vector &_vec): mode(_mode), W(_W), b(_b), vec(_vec){}
    virtual void fv(arr& y, arr& J, const arr& x){
      W.setJointState(x);
      switch(mode){
        case Pos:  W.kinematicsPos(y,J,b,&vec); break;
        case Vec:  W.kinematicsVec(y,J,b,&vec); break;
        case Quat: W.kinematicsQuat(y,J,b); break;
      }
    }
    VectorFunction& operator()(){ return *this; }
  };

  //ors::KinematicWorld G("test.ors");
  ors::KinematicWorld G("kinematicTests.kvg");
  //ors::KinematicWorld G("../../../data/pr2_model/pr2_model.ors");

  for(uint k=0;k<100;k++){
    ors::Body *b = G.bodies.rndElem();
    ors::Vector vec;
    vec.setRandom();
    arr x(G.getJointStateDimension());
    rndUniform(x,-.5,.5,false);
//    x/=sqrt(sumOfSqr(x.subRange(0,3)));

    cout <<"kinematicsPos:  "; checkJacobian(MyFct(MyFct::Pos , G, b, vec)(), x, 1e-5);
    cout <<"kinematicsVec:  "; checkJacobian(MyFct(MyFct::Vec , G, b, vec)(), x, 1e-5);
    cout <<"kinematicsQuat: "; checkJacobian(MyFct(MyFct::Quat, G, b, vec)(), x, 1e-5);

    //checkJacobian(Convert(T1::f_hess, NULL), x, 1e-5);
  }
}

//===========================================================================
//
// Jacobian test
//

void TEST(QuaternionKinematics){
  ors::KinematicWorld G("kinematicTestQuat.kvg");
  orsDrawJoints=false;

  for(uint k=0;k<3;k++){
    ors::Quaternion target;
    target.setRandom();
    G.getShapeByName("ref")->rel.rot = target;
    G.getShapeByName("marker")->rel.rot = target;
    arr x;
    G.getJointState(x);
    for(uint t=0;t<100;t++){
      arr y,J;
      G.kinematicsQuat(y, J, G.bodies.last());  //get the new endeffector position
      arr Jinv = pseudoInverse(J, NoArr, 1e-4); //~J*inverse_SymPosDef(J*~J);
      if(scalarProduct(ARRAY(target),y)<0.) target.flipSign();
      x += 0.05 * Jinv * (ARRAY(target)-y);                  //simulate a time step (only kinematically)
      G.setJointState(x);
      G.watch(false, STRING("test quaternion task spaces -- time " <<t));
    }
  }

  orsDrawJoints=true;
}

//===========================================================================
//
// copy operator test
//

void TEST(Copy){
  ors::KinematicWorld G1("kinematicTests.kvg");
  //ors::KinematicWorld G1("../../../data/pr2_model/pr2_model.ors");
  ors::KinematicWorld G2(G1);

  G1.checkConsistency();
  G2.checkConsistency();

  G1 >>FILE("z.1");
  G2 >>FILE("z.2");

  charA g1,g2;
  g1.readRaw(FILE("z.1"));
  g2.readRaw(FILE("z.2"));

  CHECK(g1==g2,"copy operator failed!")
  cout <<"** copy operator success" <<endl;
}

//===========================================================================
//
// Kinematic speed test
//

void TEST(KinematicSpeed){
#define NUM 10000
#if 1
  ors::KinematicWorld G("arm7.ors");
  G.makeLinkTree();
  uint n=G.getJointStateDimension();
  arr x(n);
  MT::timerStart();
  for(uint k=0;k<NUM;k++){
    rndUniform(x,-.5,.5,false);
    G.setJointState(x);
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
  ors::KinematicWorld *G;
  void f(arr& c, arr *dfdx, const arr &x,void*){
    G->setJointState(x);
    G->stepSwift();
    G->kinematicsProxyCost(c, (dfdx?*dfdx:NoArr), .2);
  }
}

void TEST(Contacts){
  ors::KinematicWorld G("arm7.ors");
  
  arr x,con,grad;
  uint t;

  G.swift().setCutoff(.5);

  Ctest::G=&G;

  x = G.q;
  for(t=0;t<100;t++){
    G.setJointState(x);
    G.stepSwift();

    G.reportProxies();

    G.kinematicsProxyCost(con, grad, .2);
    cout <<"contact meassure = " <<con(0) <<endl;
    //G.watch(true);
    G.watch(false, STRING("t=" <<t <<"  movement along negative contact gradient (using SWIFT to get contacts)"));
    //x += inverse(grad)*(-.1*c);
    x -= 3e-3*grad; //.1 * (invJ * grad);

    checkJacobian(Convert(Ctest::f, NULL), x, 1e10);
  }
}

//===========================================================================

void TEST(Limits){
  ors::KinematicWorld G("arm7.ors");
  struct MyFct:VectorFunction{
    ors::KinematicWorld& world;
    arr limits;
    MyFct(ors::KinematicWorld& _world):world(_world),limits(world.getLimits()){}
    void fv(arr& y, arr& J, const arr& x){
      world.setJointState(x);
      world.kinematicsLimitsCost(y,J,limits);
    }
  } F(G);

  uint n=G.getJointStateDimension();
  arr x(n),y,J;
  for(uint k=0;k<10;k++){
    rndUniform(x,-2.,2.,false);
    checkJacobian(F,x,1e-4);
    for(uint t=0;t<10;t++){
      F.fv(y,J,x);
      cout <<"y=" <<y <<"  " <<flush;
      x -= .1 * J.reshape(n);
      checkJacobian(F,x,1e-4);
      G.setJointState(x);
      G.gl().update();
    }
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
  ors::KinematicWorld G("arm7.ors");
  uint n=G.getJointStateDimension();
  arr X;
  generateSequence(X, 200, n);
  arr v(X.d1); v=0.;
  for(uint t=0;t<X.d0;t++){
    G.setJointState(X[t](),v);
    G.watch(false, STRING("replay of a state sequence -- time " <<t));
  }
}

//===========================================================================
//
// ODE test
//

#ifdef MT_ODE
void TEST(PlayTorqueSequenceInOde){
  ors::KinematicWorld G("arm7.ors");
  G.ode();
  uint n=G.getJointStateDimension();
  arr F,Xt,Vt;
  generateSequence(F, 200, n);
  F *= .01;
  Xt.resizeAs(F); Vt.resizeAs(F);
  for(uint t=0;t<F.d0;t++){
    G.ode().addJointForce(F[t]());
    //G.clearJointErrors(); exportStateToOde(C,); //try doing this without clearing joint errors...!
    G.ode().step(0.03);
    G.ode().importStateFromOde();
    G.getJointState(Xt[t](),Vt[t]());
    G.gl().text.clear() <<"play a random torque sequence [using ODE] -- time " <<t;
    G.gl().timedupdate(.01);
  }
}

void TEST(MeshShapesInOde){
  ors::KinematicWorld G("testOdeMesh.ors");
  for(uint t=0;t<1000;t++){
    //G.clearJointErrors(); exportStateToOde(C,); //try doing this without clearing joint errors...!
    G.ode().step(0.03);
    G.ode().importStateFromOde();
    G.gl().timedupdate(.01);
  }
}
#endif


//===========================================================================
//
// standard IK test
//

void TEST(FollowRedundantSequence){  
  ors::KinematicWorld G("arm7.ors");

  uint t,T,n=G.getJointStateDimension();
  arr x(n),y,J,invJ;
  x=.8;     //initialize with intermediate joint positions (non-singular positions)
  ors::Vector rel = G.getShapeByName("endeff")->rel.pos; //this frame describes the relative position of the endeffector wrt. 7th body

  //-- generate a random endeffector trajectory
  arr Z,Zt; //desired and true endeffector trajectories
  generateSequence(Z, 200, 3); //3D random sequence with limits [-1,1]
  Z *= .8;
  T=Z.d0;
  G.setJointState(x);
  G.kinematicsPos(y, NoArr, G.bodies.last(), &rel);
  for(t=0;t<T;t++) Z[t]() += y; //adjust coordinates to be inside the arm range
  plotLine(Z);
  G.gl().add(glDrawPlot,&plotModule);
  G.watch(false);
  //-- follow the trajectory kinematically
  for(t=0;t<T;t++){
    //Z[t] is the desired endeffector trajectory
    //x is the full joint state, z the endeffector position, J the Jacobian
    G.kinematicsPos(y, J, G.bodies.last(), &rel);  //get the new endeffector position
    invJ = ~J*inverse_SymPosDef(J*~J);
    x += invJ * (Z[t]-y);                  //simulate a time step (only kinematically)
    G.setJointState(x);
//    cout <<J * invJ <<endl <<x <<endl <<"tracking error = " <<maxDiff(Z[t],y) <<endl;
    G.watch(false, STRING("follow redundant trajectory -- time " <<t));
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
//  ors::KinematicWorld *G;
//}


//---------- test standard dynamic control
void TEST(Dynamics){
  ors::KinematicWorld G("arm7.ors");
//  G.makeLinkTree();
  cout <<G <<endl;

  struct DiffEqn:VectorFunction{
    ors::KinematicWorld& G;
    arr u;
    bool friction;
    DiffEqn(ors::KinematicWorld& _G):G(_G),friction(false){}
    void fv(arr& y,arr&,const arr& x){
      G.setJointState(x[0], x[1], true);
      if(!u.N) u.resize(x.d1).setZero();
      if(friction) u = -10. * x[1];
      G.clearForces();
      G.gravityToForces();
      /*if(T2::addContactsToDynamics){
        G.contactsToForces(100.,10.);
      }*/
      G.fwdDynamics(y, x[1], u);
    }
  } diffEqn(G);

  
  uint t,T=720,n=G.getJointStateDimension();
  arr q,qd,qdd(n),qdd_(n);
  G.getJointState(q, qd);
  qdd.setZero();
  
  double dt=.01;

  ofstream z("z.dyn");
  G.clearForces();
  G.watch();
//  for(ors::Body *b:G.bodies){ b->mass=1.; b->inertia.setZero(); }

  for(t=0;t<T;t++){
    if(t>=500){ //hold steady
      qdd_ = -1. * qd;
      G.inverseDynamics(diffEqn.u, qd, qdd_);
      //tau.resize(n); tau.setZero();
      //G.clearForces();
      //G.gravityToForces();
      G.fwdDynamics(qdd, qd, diffEqn.u);
      CHECK(maxDiff(qdd,qdd_,0)<1e-5,"dynamics and inverse dynamics inconsistent");
      //cout <<q <<qd <<qdd <<endl;
      cout <<"test dynamics: fwd-inv error =" <<maxDiff(qdd,qdd_,0) <<endl;
      q  += .5*dt*qd;
      qd +=    dt*qdd;
      q  += .5*dt*qd;
      G.setJointState(q,qd);
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
    G.watch(false);
  }
}

/*void switchfunction(arr& s,const arr& x,const arr& v){
  G.setJointState(x,v);
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
      //G.zeroGaugeJoints();
      G.getJointState(q,qd);
    }
    z <<q <<qd <<qdd <<endl;
    conswit=s;
    if(false && checkContacts(s)){
      dt=.001;
      addContactsToDynamics=true;
      //G.watch(true);
    }else{
      dt=.01;
      addContactsToDynamics=false;
    }
    cross=MT::rk4dd_switch(q,qd,s,q,qd,s,ddf_joints,switchfunction,dt,1e-4);
    //G.reportProxies();
    cout <<"*** s = " <<s <<endl;
    G.gl().text.clear() <<"t=" <<t <<"  using RK4_switch,  energy=" <<G.getEnergy();
    //if(cross) G.watch(true);
    G.watch(false);
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
  ors::KinematicWorld bl;
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
  testCopy();
  testPlayStateSequence();
  testKinematics();
  testQuaternionKinematics();
  testKinematicSpeed();
  testFollowRedundantSequence();
  testDynamics();
  testContacts();
  testLimits();
#ifdef MT_ODE
//  testMeshShapesInOde();
  testPlayTorqueSequenceInOde();
#endif
  //testBlenderImport();

  return 0;
}
