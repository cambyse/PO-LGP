#include "traj_optimizer.h"
#include <GL/glu.h>
#include <Gui/opengl.h>


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


TrajOptimizer::TrajOptimizer(ors::KinematicWorld &_world)
{
  world = _world;
}

void TrajOptimizer::optimizeTrajectory(arr &_goal, arr &_q0, arr &x) {
  // Create Motion Problem
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  //-- create tasks for optimization problem
  TaskCost *c;

  c = MP.addTask("final_vel", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c,MotionProblem::finalOnly,ARRAY(0.),1e3);
  c->map.order=1;

  c = MP.addTask("position_right_hand", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, _goal, 1e5);

  //  c = MP.addTask("limits", new DefaultTaskMap(qLimitsTMT,world));
  //  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(0.), 1e5);
  //  c->map.order=1;


  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = _q0;


  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  x = repmat(~MP.x0,T+1,1);
  optNewton(x, Convert(MPF), OPT(verbose=0, allowOverstep=true));
  MP.costReport(true);


  cout << "goal: " << _goal << endl;
  displayTrajectory(x,100,world,"reference trajectory");
}

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void TrajOptimizer::sampleGoal(arr &_goal,const arr &_q0)
{
  double qLowerLimit[7] = {-2.2854, -0.5236, -3.9, -1.7, _q0(4)-M_PI, -2. ,_q0(6)-M_PI};
  double qUpperLimit[7] = {0.554602, 0.6, 0.66, 0., _q0(4)+M_PI, 0., _q0(6)+M_PI};

  //  arr limits = world.getLimits();
  //  cout << limits << endl;
  _goal.resizeAs(_q0);
  //  limits(4,0) = ;
  //  limits(6,0) = _q0(6)-M_PI;
  //  limits(4,1) = _q0(4)+M_PI;
  //  limits(6,1) = _q0(6)+M_PI;


  for (uint i = 0;i<_q0.N;i++) {
    //    _goal(i) = fRand( max(ARR(limits(i,0),_q0(i)-M_PI_2)) , min(ARR(limits(i,1),_q0(i)+M_PI_2)));
    _goal(i) = fRand( qLowerLimit[i], qUpperLimit[i] );
  }
}

void TrajOptimizer::optimizeBenchmarkMotion(BM_TYPE type, arr &_q0, arr &x)
{
  // Create Motion Problem
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  //-- create tasks for optimization problem
  TaskCost *c;

  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = _q0;

  //-- get current position in endeffector space
  arr traj0;
  world.setJointState(_q0,_q0*0.);
  world.kinematicsPos(traj0,NoArr,world.getBodyByName("endeffR"));

  switch (type){
    case BM_TYPE::STAR:
    {
      double l = 0.2;

      MP.T = 800;
      arr l1 = linspace(0,l,MP.T/16.-1);
      arr l2 = linspace(0,2.*l,MP.T/8.-1);

      traj = catCol(traj0(0)+0.*l1,traj0(1)+0.*l1,traj0(2)+l1);
      traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)+0.*l2,traj(traj.d0-1,2)-l2));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+0.*l1,traj(traj.d0-1,2)+l1));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+l1,traj(traj.d0-1,2)-l1*0.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)-l2,traj(traj.d0-1,2)-l2*0.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+l1,traj(traj.d0-1,2)+l1*0.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)-sqrt(2.)*l2/2.,traj(traj.d0-1,2)-sqrt(2.)*l2/2.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)-sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)+sqrt(2.)*l2/2.,traj(traj.d0-1,2)-sqrt(2.)*l2/2.));
      traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)-sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));
      traj.append(traj[traj.d0-1]);

      MP.tau = MT::getParameter<double>("duration") / double(MP.T);

      break;
    }
    case BM_TYPE::EIGHT:
    {
      double l = 0.1;

      double off = 0.1;
      double radius = 0.2;
      MP.T = round(MP.T/2.);
      arr t = linspace(0,.75,MP.T);
      arr x_traj = sin((t+.125)*2.*M_PI)*radius;
      arr y_traj = cos((t+.125)*2.*M_PI)*radius;

      // circles
      arr c1 = catCol(traj0(0)+0.*x_traj,traj0(1)+y_traj-radius-off,traj0(2)+x_traj);
      arr c2 = catCol(traj0(0)+0.*x_traj,traj0(1)-y_traj+radius+off,traj0(2)+x_traj);

      // lines
      double m = (c1(0,2) - c2(c2.d0-1,2))/(c1(0,1) - c2(c2.d0-1,1));
      arr center = (c2[0] - c1[c1.d0-1])*.5+c1[c1.d0-1];
      arr xlin = linspace(c1(0,1),c2(0,1),MP.T/3) -center(1);
      arr ylin = xlin*m;

      arr l1 = catCol(traj0(0)+0.*xlin,center(1)+xlin,center(2)-ylin);
      xlin.reverseRows();
      arr l2 = catCol(traj0(0)+0.*xlin,center(1)+xlin,center(2)-ylin);

      // circle 2
      traj.append(c1);
      traj.append(l1);
      traj.append(c2);
      traj.append(l2);

      traj.shift(round(l1.d0*0.5)*3);

      traj.append(traj);
//      traj.append(traj);

      MP.T = traj.d0-1;
      MP.tau = MT::getParameter<double>("duration") / double(MP.T);
      break;
    }
    case BM_TYPE::CIRCLE:
    {
      double turns = 5;

      // compute circle traj
      double radius = 0.18;
      arr t = linspace(0,1.,MP.T);
      arr x_traj = sin(t*2.*turns*M_PI)*radius;
      arr y_traj = cos(t*2.*turns*M_PI)*radius;
      traj = catCol(traj0(0)+0.*x_traj,traj0(1)+y_traj-radius,traj0(2)+x_traj);

      break;
    }
  }

  c = MP.addTask("pos", new DefaultTaskMap(posTMT,world,"endeffR"));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(1.,1.,1.), 1e4);
  c->target = traj; // set traj
  c->prec.subRange(0,20)=0.;

  c = MP.addTask("q_limit",new DefaultTaskMap(qLimitsTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(0.), 1e4);


  world.gl().add(drawActTraj,&(traj));

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  x = repmat(~MP.x0,T+1,1);
  optNewton(x, Convert(MPF), OPT(verbose=0,stopTolerance=1e-5,allowOverstep=true));
  MP.costReport(true);

  for (uint i = 0;i<x.d0;i=i+10) {
    world.setJointState(x[i]);
    world.gl().update();
  }
}

