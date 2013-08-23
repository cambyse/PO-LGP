#include <Ors/ors_physx.h>
#include <Gui/opengl.h>
#include <Core/array.h>
#include <Core/util.h>

/*----------------------------------------------------------------------------*/
int main(int argc, char** argv) {
  ors::Graph ors;
  ors.init("doorComplex.ors");
  ors::Body* robot = ors.getBodyByName("robot");
  ors.calcBodyFramesFromJoints();

  OpenGL glMy;
  OpenGL glPh("PhysX");
  PhysXInterface physx;
  bindOrsToOpenGL(ors, glMy);
  bindOrsToPhysX(ors, glPh, physx);

  SwiftInterface swift;
  swift.init(ors,0.3);

  double wGoal=1.5;
  double wObs=1;
  double dt = 0.01;
  double eps=1;

  arr x, goal;
  arr dirObs(3,1);
  arr dir(3,1);
  ors::Proxy *p;

  x = ARRAY(robot->X.pos);

  for (uint k=0; k<6; k++) {
    // set current goal
    MT::String bname;
    bname << "door" << k+1;
    goal = ARRAY(ors.getBodyByName(bname)->X.pos);
    goal(2) = 1;

    // remove collision avoidance for current door
    (ors.getBodyByName(bname)->shapes(0))->cont = false;

    while (norm(x - goal) > 1e-1) {
      x = ARRAY(robot->X.pos);

      // direction towards goal
      dir = wGoal*(goal - x)/norm(goal - x);

      // obstacle avoidance
      swift.computeProxies(ors,false);
      dirObs.setZero();
      for (uint j = 0; j < ors.proxies.N; j++) {
        p = ors.proxies(j);
        MT::String objA(ors.shapes(p->a)->body->name.p);
        MT::String objB(ors.shapes(p->b)->body->name.p);

        if ((p->d < eps) && (objA == "robot")) {
          ors.reportProxies();
          dirObs = dirObs + ARRAY(p->normal);
        }

        if ((p->d < eps) && (objB == "robot")) {
          ors.reportProxies();
          dirObs = dirObs - ARRAY(p->normal)/p->d;
        }
      }

      // add direction pointing away of obstacles
      if (norm(dirObs) > 0) {
        dir += wObs*dirObs/norm(dirObs);
      }

      // compute next position
      robot->X.pos = robot->X.pos + dt*dir/norm(dir);

      // update sim
      physx.step();
      glMy.update();
      glPh.update();

    }

    // add collision avoidance for current door again
    (ors.getBodyByName(bname)->shapes(0))->cont = true;
    swift.init(ors,1);
  }

  return 0;
}


// vim: ts=2:sw=2:expandtab