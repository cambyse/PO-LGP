#include <Core/util.h>
#include <Kin/kin.h>

#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

#include <GL/glu.h>
#include <Gui/opengl.h>
#include <stdlib.h>
#include <vector>
#include "mpc.h"
#include <Motion/mobject.h>

const double PI = 3.1415926535897932384626;
//** Model Predictive Control **//
uint figID = 0;



void drawTraj(uint color, arr& p, uint lineStyle) {
  glColor(color);
  glPointSize(4.0f);
  glLineWidth(2);
  if (lineStyle == 1) {
    glBegin(GL_POINTS);
  } else {
    glBegin(GL_LINES);
  }
  glVertex3f(p(0,0),p(0,1),p(0,2));
  uint i;
  for (i = 1; i<p.d0-1; i++) {
    glVertex3f(p(i,0),p(i,1),p(i,2));
    glVertex3f(p(i,0),p(i,1),p(i,2));
  }
  glVertex3f(p(i,0),p(i,1),p(i,2));
  glEnd();
  glLineWidth(1);
}

void drawActTraj(void* classP){
  arr *p = (arr*)classP;
  drawTraj(5,*p,1);
}

void plotTraj(arr &x, double dt) {
  // compute velocity
  arr xd(x.d0,x.d1);
  uint i,j;
  for (i=0; i<x.d0; i++) {
    for (j=0; j<x.d1; j++) {
      if (i == 0) {
        xd(i,j) = (x(i+1,j) - x(i,j))/(dt);
      } else if(i == x.d0-1) {
        xd(i,j) = (x(i,j) - x(i-1,j))/(dt);
      } else {
        xd(i,j) = (x(i+1,j) - x(i-1,j))/(2*dt);
      }
    }
  }

  // compute acceleration
  arr xdd(x.d0,x.d1);
  for (i=0; i<xd.d0; i++) {
    for (j=0; j<xd.d1; j++) {
      if (i == 0) {
        xdd(i,j) = (xd(i+1,j) - xd(i,j))/(dt);
      } else if(i == xd.d0-1) {
        xdd(i,j) = (xd(i,j) - xd(i-1,j))/(dt);
      } else {
        xdd(i,j) = (xd(i+1,j) - xd(i-1,j))/(2*dt);
      }
    }
  }

  cout << "x[0]" << x[0] << endl;
  cout << "xd[0]" << xd[0] << endl;
  cout << "xdd[0]" << xdd[0] << endl;

  // Plot optimal trajectory
  // Plot position
  write(LIST<arr>(x),"out/x.output");
  mlr::String n;
  n<<"set term wxt "<<++figID<<" title 'position x'";
  gnuplot(n);
  gnuplot("plot 'out/x.output'using 1");
  gnuplot("replot 'out/x.output'using 2");
  gnuplot("replot 'out/x.output'using 3");
  gnuplot("replot 'out/x.output'using 4");
  gnuplot("replot 'out/x.output'using 5");
  gnuplot("replot 'out/x.output'using 6");
  gnuplot("replot 'out/x.output'using 7");

  // Plot velocity
  write(LIST<arr>(xd),"out/xd.output");
  n.clear();
  n<<"set term wxt "<<++figID<<" title 'velocity xd'";
  gnuplot(n);
  gnuplot("plot 'out/xd.output'using 1");
  gnuplot("replot 'out/xd.output'using 2");
  gnuplot("replot 'out/xd.output'using 3");
  gnuplot("replot 'out/xd.output'using 4");
  gnuplot("replot 'out/xd.output'using 5");
  gnuplot("replot 'out/xd.output'using 6");
  gnuplot("replot 'out/xd.output'using 7");

  // Plot acceleration
  write(LIST<arr>(xdd),"out/xdd.output");
  n.clear();
  n<<"set term wxt "<<++figID<<" title 'acceleration xdd'";
  gnuplot(n);
  gnuplot("plot 'out/xdd.output'using 1");
  gnuplot("replot 'out/xdd.output'using 2  ");
  gnuplot("replot 'out/xdd.output'using 3");
  gnuplot("replot 'out/xdd.output'using 4");
  gnuplot("replot 'out/xdd.output'using 5");
  gnuplot("replot 'out/xdd.output'using 6");
  gnuplot("replot 'out/xdd.output'using 7");
}

void scenario1() {

  // Create Trajectory with start velocity using PREFIX
  mlr::KinematicWorld G("scenes/scene1");
  makeConvexHulls(G.shapes);

  KOMO P(G);


  cout << "Loaded scene: " << endl;

  Task *c;
  c = P.addTask("transition", 	new TaskMap_Transition(G));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, P.T, {0.},1e-2);

  c = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", mlr::Vector(0., 0., 0.)));
  c->setCostSpecs(P.T, P.T,  conv_vec2arr(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                             {0.,0.,0.}, 1e-3);
  c = P.addTask("position_vel", new TaskMap_Default(posTMT,G,"endeff", mlr::Vector(0., 0., 0.)));
  c->map.order=1;
  c->setCostSpecs(P.T, P.T,  {0.,0.,0.}, 1e3,
                             {0.,0.,0.}, 0.);

  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt=P.tau;
  cout <<"Problem parameters:"<<" T="<<T<<" k="<<k<<" n="<<n<<"dt="<<dt<<" # joints=" <<G.getJointStateDimension()<<endl;

  double v0 = 0.03;

  arr x0 = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};
  arr prefix(2,n);
  prefix[0] = x0-2*dt*v0*{1.,1.,1.,1.,1.,1.,1.};
  prefix[1] = x0-dt*v0*{1.,1.,1.,1.,1.,1.,1.};


  P.prefix = prefix;
  cout << "Prefix: " << F.get_prefix() << endl;
  //-- mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));

  plotTraj(x,dt);

  displayTrajectory(x, 1, G, "planned trajectory");
  G.gl().watch();

}

void scenario2() {
  mlr::KinematicWorld G("scenes/scene1");
  makeConvexHulls(G.shapes);

  KOMO P(G);

  cout << "Loaded scene: " << endl;

  Task *c;
  c = P.addTask("transition", 	new TaskMap_Transition(G));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, P.T, {0.},1e-2);

  c = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", mlr::Vector(0., 0., 0.)));
  c->setCostSpecs(P.T, P.T,  conv_vec2arr(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                             {0.,0.,0.}, 1e-3);
  c = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", mlr::Vector(0., 0., 0.)));
  c->map.order=1;
  c->setCostSpecs(P.T, P.T,  {0.,0.,0.}, 1e3,
                             {0.,0.,0.}, 0.);

  MotionProblemFunction F(P);
  uint T=F.get_T(); uint k=F.get_k(); uint n=F.dim_x(); double dt=P.tau;
  cout <<"Problem parameters:"<<" T="<<T<<" k="<<k<<" n="<<n<<"dt="<<dt<<" # joints=" <<G.getJointStateDimension()<<endl;


  //-- mini evaluation test:
  arr xRef(T+1,n);
  xRef.setZero();

  //-- optimize
  optNewton(xRef, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));
  //  P.costReport();
  plotTraj(xRef,dt);
  mlr::wait(2);
  //  displayTrajectory(xRef, 1, G, gl,"planned trajectory");



  //** Replan from index i **//
  uint i = 2;
  arr x0 = xRef[i];
  arr v0 = (xRef[i+1]-xRef[i])/dt;
  arr a0 = (xRef[i+1]+xRef[i-1]-2.*xRef[i])/(dt*dt);
  G.setJointState(x0,v0);
  P.T = T-i;
  cout << "P.T: " << P.T << endl;

  // reset costs
  mlr::timerStart();
  Task *c2;
  c2 = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", mlr::Vector(0., 0., 0.)));

  c2->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          {0.,0.,0.}, 1e-3);
  c2 = P.addTask("position", new TaskMap_Default(posTMT,G,"endeff", mlr::Vector(0., 0., 0.)));
  c2->map.order=1;
  c2->setCostSpecs(P.T, P.T,
                             {0.,0.,0.}, 1e3,
                             {0.,0.,0.}, 0);

  arr prefix(2,n);
  prefix[1] = xRef[i-1];
  prefix[0] = xRef[i-2];
  P.prefix = prefix;

  // execute trajectory and continiously replan in each step
  arr x = xRef.rows(i,xRef.d0);
  x.setZero();

  optNewton(x, Convert(F), OPT(verbose=1, stopIters=20, damping=1e-3, maxStep=1., stopTolerance=1e-2));
  cout <<"Optimization time: " <<mlr::timerRead() <<"sec" <<endl;

  plotTraj(x,dt);

  cout << "\nx[0] before: " << x0 << endl;
  cout << "x[0] after: " << x[0] << endl;

  cout << "\nv[0] before: " << v0 << endl;
  cout << "v[0] after: " << (x[1]-x[0])/(dt) << endl;

  cout << "\na[0] before: " << a0 << endl;
  cout << "a[0] after: " << (x[1]+prefix[1]-(2.*x[0]))/(dt*dt) << endl;

  cout << sum((x- xRef({i,xRef.d0-1}))%(x- xRef({i,xRef.d0-1}))) << endl;

  displayTrajectory(x, 1, G, "planned trajectory");

}

void scenario3() {
  mlr::KinematicWorld world("scenes/scene1");
  arr q, qdot;
  world.getJointState(q, qdot);
  /*
  ** Plan Trajectory
  */

  KOMO P(world);



  arr goalRef = conv_vec2arr(P.world.getBodyByName("goalRef")->X.pos);

  //-- create an optimal trajectory to trainTarget
  Task *c;
  c = P.addTask("transition", 	new TaskMap_Transition(world));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, P.T, {0.},1e-2);

  c = P.addTask("position", new TaskMap_Default(posTMT,world,"endeff", mlr::Vector(0., 0., 0.)));
  c->setCostSpecs(P.T, P.T, goalRef, 1e4);
  c = P.addTask("position", new TaskMap_Default(posTMT,world,"endeff", mlr::Vector(0., 0., 0.)));
  c->map.order=1;
  c->setCostSpecs(P.T, P.T, {0.,0.,0.}, 1e3);

  c = P.addTask("orientation", new TaskMap_Default(vecTMT,world,"endeff",mlr::Vector(0., 0., 1.)));
  c->setCostSpecs(P.T, P.T, {1.,0.,0.}, 1e4);
  c = P.addTask("orientation", new TaskMap_Default(vecTMT,world,"endeff",mlr::Vector(0., 0., 1.)));
  c->map.order=1;
  c->setCostSpecs(P.T, P.T, {0.,0.,0.}, 1e3);

  P.x0 = 0.1;

  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt = P.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n);
  x.setZero();
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));
  //  P.costReport();
  //  displayTrajectory(x, 1, world, "planned trajectory", 0.01);


  /*
  ** Execute Trajectory
  */
  arr q0 = x[0];
  arr y_target, yPos_target,Phi,PhiJ,yVec_target,qd,W,yPos,yVec,JPos,JVec;

  q = P.x0;
  double fPos_deviation = 1e-2;
  double fVec_deviation = 1e-3;
  double w_reg = 100.;

  W.setDiag(1.,world.getJointStateDimension());  // W is equal the Id_n matrix
  W = W*w_reg;

  double tau_plan = P.tau;
  double t = 0.;
  double t_final = T*dt;

  MObject goalMO(&world, mlr::String("goal"), MObject::GOAL , 0.01, {0.,0.,1.});
  MPC mpc(P,x);
  world.setJointState(q);
  world.getJointState(q);

  // gl visualization
  world.gl().add(drawActTraj,&(mpc.x_cart));

  // RUN //
  while ((world.getBodyByName("endeff")->X.pos - goalMO.position).length() >1e-1) {

//    goalMO.move();
    mpc.replan(goalMO.position, q);

    q += (mpc.x[2]-mpc.x[1]);

    world.setJointState(q);

    t += tau_plan;
    world.watch(false, STRING(t));
  }
  world.watch(true,STRING(t));
}



int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  switch(mlr::getParameter<int>("mode",3)){
  case 1:  scenario1();  break;
  case 2:  scenario2();  break;
  case 3:  scenario3();  break;
  }

  return 0;
}

