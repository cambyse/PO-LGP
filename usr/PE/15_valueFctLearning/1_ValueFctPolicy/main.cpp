#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>


void run() {
  ors::KinematicWorld world("scene0");
  MotionProblem MP(world);

  Task *t;
  Task *t2;
  t = MP.addTask("pos", new DefaultTaskMap(posTMT, world, "endeff", NoVector));
//  t2 = MP.addTask("rot", new DefaultTaskMap(vecTMT, world, "endeff", ors::Vector(0.,0.,1.), NULL, ors::Vector(0.,0.,1.)));

  arr y,J;
  arr x0,y0;
  world.getJointState(x0);
  MP.taskCosts.last()->map.phi(y0,J,world);

  arr W = eye(3);
  arr C = eye(4)*2.;
  arr w;
  w = ARR(world.getBodyByName("target")->X.pos);
  w=-2.*w;
  cout<< "x0 :" <<x0 << endl;
  cout <<"y0: "<< y0 << endl;
  cout <<"J: "<< J << endl;
  cout << "~y*W*y: "<< ~y0*W*y0 << endl;
  cout << "~x0*C*x0: "<< ~x0*C*x0 << endl;

  arr x;
  y =y0;

  while (length(y)>1e-3) {
    y.setZero(); J.setZero();
    for (uint i=0;i<MP.taskCosts.d0;i++) {
      arr yi,Ji;
      MP.taskCosts(i)->map.phi(yi,Ji,world);
      y = y + yi;
      J = J + Ji;
    }

    x = x0-inverse_SymPosDef(~J*W*J + C)*~J*(0.5*w + W*y);
    x0 = x;
    cout << "value: " << ~y*W*y+~w*y << endl;
    cout << "value: " << ~y*W*y << endl;
    cout << "distance to goal: " <<length(y) << endl;
    cout << "distance to goal: " <<y << endl;
    world.setJointState(x);
    world.gl().update();
    mlr::wait(0.01);
  }


  world.watch(true);
  return;
}

void run2() {
  ors::KinematicWorld world("scene0");
  MotionProblem MP(world);

  Task *t;
  Task *t2;
  t = MP.addTask("pos", new DefaultTaskMap(posTMT, world, "endeff", NoVector));

  arr y,J;
  arr x0,y0;
  world.getJointState(x0);
  MP.taskCosts.last()->map.phi(y0,J,world);

  arr W = eye(3);

  arr w;
  w = ARR(world.getBodyByName("target")->X.pos);
  w=-w;
  cout<< "x0 :" <<x0 << endl;
  cout <<"y0: "<< y0 << endl;
  cout <<"J: "<< J << endl;
  cout << "~y*W*y: "<< ~y0*W*y0 << endl;

  arr x;
  y =y0;
  double alpha = .01;
  while (length(y)>1e-3) {
    y.setZero(); J.setZero();
    for (uint i=0;i<MP.taskCosts.d0;i++) {
      arr yi,Ji;
      MP.taskCosts(i)->map.phi(yi,Ji,world);
      y = y + yi;
      J = J + Ji;
    }
    arr dir =2.*(~y*W*J + ~w*J);
    dir = dir/length(dir);
    x = x0 - alpha*dir;
    x0 = x;
    cout << "value: " << ~y*W*y+~w*y << endl;
    cout << "value: " << ~y*W*y << endl;
    cout << "distance to goal: " <<length(y) << endl;
    cout << "y: " <<y << endl;
    world.setJointState(x);
    world.gl().update();
    mlr::wait(0.01);
  }


  world.watch(true);
  return;
}



int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  run(); // value fct downhill with regularization
//  run2(); // value fct downhill with fixed stepsize
  return 0;
}
