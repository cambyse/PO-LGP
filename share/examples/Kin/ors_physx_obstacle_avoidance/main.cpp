#include <Kin/kin_physx.h>
#include <Gui/opengl.h>
#include <Core/array.h>
#include <Core/util.h>
#include <Kin/kin_swift.h>

/*----------------------------------------------------------------------------*/

void TEST(PhysxObstacleAvoidance) {
  mlr::KinematicWorld ors;
  ors.init("doorComplex.ors");
  mlr::Body* robot = ors.getBodyByName("robot");

  OpenGL glPh("PhysX");
  //bindOrsToPhysX(ors, glPh, physx);

  ors.swift().setCutoff(0.3);

  double wGoal=1.5;
  double wObs=1;
  double dt = 0.01;
  double eps=1;

  arr x, goal;
  arr dirObs(3,1);
  arr dir(3,1);
  mlr::Proxy *p;

  x = conv_vec2arr(robot->X.pos);

  for (uint k=0; k<6; k++) {
    // set current goal
    mlr::String bname;
    bname << "door" << k+1;
    goal = conv_vec2arr(ors.getBodyByName(bname)->X.pos);
    goal(2) = 1;

    // remove collision avoidance for current door
    (ors.getBodyByName(bname)->shapes(0))->cont = false;

    while (length(x - goal) > 1e-1) {
      x = conv_vec2arr(robot->X.pos);

      // direction towards goal
      dir = wGoal*(goal - x)/length(goal - x);

      // obstacle avoidance
      ors.stepSwift();
      dirObs.setZero();
      for (uint j = 0; j < ors.proxies.N; j++) {
        p = ors.proxies(j);
        mlr::String objA(ors.shapes(p->a)->body->name.p);
        mlr::String objB(ors.shapes(p->b)->body->name.p);

        if ((p->d < eps) && (objA == "robot")) {
          ors.reportProxies();
          dirObs = dirObs + conv_vec2arr(p->normal);
        }

        if ((p->d < eps) && (objB == "robot")) {
          ors.reportProxies();
          dirObs = dirObs - conv_vec2arr(p->normal)/p->d;
        }
      }

      // add direction pointing away of obstacles
      if (length(dirObs) > 0) {
        dir += wObs*dirObs/length(dirObs);
      }

      // compute next position
      robot->X.pos = robot->X.pos + dt*dir/length(dir);

      // update sim
      ors.stepPhysx(0.01);
      ors.gl().update();
      //glPh.update();

    }

    // add collision avoidance for current door again
    (ors.getBodyByName(bname)->shapes(0))->cont = true;
    ors.swift().initActivations(ors);
    ors.swift().setCutoff(1.);
  }
}

int MAIN(int argc, char** argv) {
  testPhysxObstacleAvoidance();

  return 0;
}
