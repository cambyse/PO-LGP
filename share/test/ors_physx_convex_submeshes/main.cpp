// Use ors and physx with objects wich consist of convex submeshes.

#include <MT/ors_physx.h>
#include <MT/opengl.h>


/*----------------------------------------------------------------------------*/
int main(int argc, char** argv) {
  ors::Graph graph;
  graph.init("doorSimple.ors");
  ors::Body* robot = graph.getBodyByName("robot");
  graph.calcBodyFramesFromJoints();

  OpenGL glMy;
  OpenGL glPh("PhysX");
  PhysXInterface physx;
  bindOrsToOpenGL(graph, glMy);
  bindOrsToPhysX(graph, glPh, physx);

  ors::Vector control = ors::Vector(0.0, 0.01, 0.0);
  for (uint i = 0; i < 1000; i++) {

    // move robot
    robot->X.pos += control;
    graph.calcBodyFramesFromJoints();

    // update sim
    physx.step();
    glMy.update();
    glPh.update();
  }
  return 0;
}

// vim: ts=2:sw=2:expandtab
