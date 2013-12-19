#include <Ors/ors_physx.h>
#include <Gui/opengl.h>

void TEST(PhysxConvexSubmeshes) {
  ors::KinematicWorld graph;
  graph.init("doorSimple.ors");
  ors::Body* robot = graph.getBodyByName("robot");
  graph.calcBodyFramesFromJoints();

  OpenGL glPh("PhysX");
  //  bindOrsToPhysX(graph, glPh, physx);

  ors::Vector control = ors::Vector(0.0, 0.01, 0.0);
  for (uint i = 0; i < 1000; i++) {

    // move robot
    robot->X.pos += control;
    graph.calcBodyFramesFromJoints();

    // update sim
    graph.stepPhysx(0.01);
    graph.gl().update();
    //glPh.update();
  }
}

int MAIN(int argc, char** argv) {
  testPhysxConvexSubmeshes();

  return 0;
}
