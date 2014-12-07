#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motionHeuristics.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <GL/glu.h>

#include <Ors/ors_swift.h>


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

void circle_BM(){
  ors::KinematicWorld world("pr2_my_arm.ors");

  // Create Motion Problem
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  //-- create tasks for optimization problem
  TaskCost *c;

  arr traj0;
  world.kinematicsPos(traj0,NoArr,world.getBodyByName("endeffR"));

  arr q0;
  q0 = world.getJointState();

  double turns = 2;

  // compute circle traj
  double radius = 0.3;
  arr t = linspace(0,1.,MP.T);
  arr x_traj = sin(t*2.*turns*M_PI)*radius;
  arr y_traj = cos(t*2.*turns*M_PI)*radius;
  arr traj = catCol(traj0(0)+0.*x_traj,traj0(1)+y_traj-radius,traj0(2)+x_traj);


  c = MP.addTask("circle_pos", new DefaultTaskMap(posTMT,world,"endeffR"));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(1.,1.,1.), 1e4);
  c->target = traj;
  c->prec.subRange(0,20)=0.;

  c = MP.addTask("q_limit",new DefaultTaskMap(qLimitsTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(0.), 1e4);

  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = q0;
  world.gl().add(drawActTraj,&(traj));

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x = repmat(~MP.x0,T+1,1);
  optNewton(x, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,allowOverstep=true));

  MP.costReport(true);

  arr xRef;
  for (uint i = 0;i<x.d0;i++) {
    world.setJointState(x[i]);
    world.kinematicsPos(xRef,NoArr,world.getBodyByName("endeffR"));
    world.gl().update();
  }

}

void star_BM(){
  ors::KinematicWorld world("pr2_my_arm.ors");

  // Create Motion Problem
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  //-- create tasks for optimization problem
  TaskCost *c;


  arr traj0;
  world.kinematicsPos(traj0,NoArr,world.getBodyByName("endeffR"));

  cout << "traj0: " <<traj0 << endl;

  double l = 0.15;

  arr l1 = linspace(0,l,MP.T/16.-1);
  arr l2 = linspace(0,2.*l,MP.T/8.-1);

  arr q0;
  q0 = world.getJointState();

  cout << l1.d0  << endl;
  cout << catCol(l1,l1,l1) << endl;

  // 0.5 up
  arr traj = catCol(traj0(0)+0.*l1,traj0(1)+0.*l1,traj0(2)+l1);

  // 1 down
  traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)+0.*l2,traj(traj.d0-1,2)-l2));

  // 0.5 up
  traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+0.*l1,traj(traj.d0-1,2)+l1));

  // 0.5 left
  traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+l1,traj(traj.d0-1,2)-l1*0.));

  // 1 right
  traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)-l2,traj(traj.d0-1,2)-l2*0.));

  // 0.5 left
  traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+l1,traj(traj.d0-1,2)+l1*0.));

  // 0.5 lu
  traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));

  // 1 rd
  traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)-sqrt(2.)*l2/2.,traj(traj.d0-1,2)-sqrt(2.)*l2/2.));

  // 0.5 lu
  traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)+sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));

  // 0.5 lu
  traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)-sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));

  // 1 rd
  traj.append(catCol(traj(traj.d0-1,0)+0.*l2,traj(traj.d0-1,1)+sqrt(2.)*l2/2.,traj(traj.d0-1,2)-sqrt(2.)*l2/2.));

  // 0.5 lu
  traj.append(catCol(traj(traj.d0-1,0)+0.*l1,traj(traj.d0-1,1)-sqrt(2.)*l1/2.,traj(traj.d0-1,2)+sqrt(2.)*l1/2.));

  traj.append(traj[traj.d0-1]);


  c = MP.addTask("circle_pos", new DefaultTaskMap(posTMT,world,"endeffR"));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(1.,1.,1.), 1e5);
  c->target = traj;
//  c->prec.subRange(0,20)=0.;

  c = MP.addTask("q_limit",new DefaultTaskMap(qLimitsTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(0.), 1e4);

  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = q0;
  world.gl().add(drawActTraj,&(traj));


  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x = repmat(~MP.x0,T+1,1);
  optNewton(x, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,allowOverstep=true));
  MP.costReport(true);

  double TRef = dt*T;

  arr lim = world.getLimits();
  for (uint i = 0;i<x.d1;i++) {
    cout << i << " min: "<< min(x.col(i)) << " max: "<<max(x.col(i)) << endl;
    cout << i << " lim: "<< lim(i,0) << " lim: "<<lim(i,1) << endl;

  }

  arr xRef;
  for (uint i = 0;i<x.d0;i++) {
    world.setJointState(x[i]);
    world.kinematicsPos(xRef,NoArr,world.getBodyByName("endeffR"));
//    cout << "x[t] " << xRef << " traj[t]: " << traj[i] << endl;
    world.gl().update();
  }

}

void eight_BM(){
  ors::KinematicWorld world("pr2_my_arm.ors");

  // Create Motion Problem
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  //-- create tasks for optimization problem
  TaskCost *c;


  arr traj0;
  world.kinematicsPos(traj0,NoArr,world.getBodyByName("endeffR"));

  cout << "traj0: " <<traj0 << endl;

  double l = 0.15;

  double off = 0.1;
  double radius = 0.2;
  arr t = linspace(0,.75,MP.T);
  arr x_traj = sin((t+.125)*2.*M_PI)*radius;
  arr y_traj = cos((t+.125)*2.*M_PI)*radius;

  arr q0;
  q0 = world.getJointState();

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
  arr traj;
  traj.append(c1);
  traj.append(l1);
  traj.append(c2);
  traj.append(l2);

  cout << traj << endl;
  traj.shift(l1.d0*0.5*3);

  cout << traj[0] << endl;
  cout << traj0 << endl;

  MP.T = traj.d0-1;

  c = MP.addTask("circle_pos", new DefaultTaskMap(posTMT,world,"endeffR"));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(1.,1.,1.), 1e5);
  c->target = traj;
//  c->prec.subRange(0,20)=0.;

  c = MP.addTask("q_limit",new DefaultTaskMap(qLimitsTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(0.), 1e4);

  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = q0;
  world.gl().add(drawActTraj,&(traj));


  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x = repmat(~MP.x0,T+1,1);
  optNewton(x, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,allowOverstep=true));

  cout << q0 << endl;
  cout << x[0] << endl;

  MP.costReport(true);

  double TRef = dt*T;

  arr lim = world.getLimits();
  for (uint i = 0;i<x.d1;i++) {
    cout << i << " min: "<< min(x.col(i)) << " max: "<<max(x.col(i)) << endl;
    cout << i << " lim: "<< lim(i,0) << " lim: "<<lim(i,1) << endl;

  }

  arr xRef;
  for (uint i = 0;i<x.d0;i++) {
    world.setJointState(x[i]);
    world.kinematicsPos(xRef,NoArr,world.getBodyByName("endeffR"));
//    cout << "x[t] " << xRef << " traj[t]: " << traj[i] << endl;
    world.gl().update();
  }

}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

//  circle_BM();
//  star_BM();
  eight_BM();

  return 0;
}
