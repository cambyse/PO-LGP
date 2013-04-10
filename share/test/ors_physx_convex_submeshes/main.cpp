// Use ors and physx with objects wich consist of convex submeshes.

#include <MT/ors_physx.h>
#include <MT/opengl.h>


/*----------------------------------------------------------------------------*/
/**
 * @brief Create a simple spherical robot.
 *
 * @param graph add the robot to the graph.
 *
 * @return pointer to the robot body.
 */
ors::Body* createRobot(ors::Graph& graph) {
  ors::Body* robot = new ors::Body(graph);
  robot->X.setRandom();
  robot->X.pos.x -= .5;
  robot->X.pos.y -= .5;
  robot->X.pos.z += 1.;
  robot->name << "robot";
  robot->type = ors::kinematicBT;

  ors::Shape* robotShape = new ors::Shape(graph, robot);
  robotShape->type = ors::sphereST;
  robotShape->size[0] = .05;
  robotShape->size[1] = .05;
  robotShape->size[2] = .05;
  robotShape->size[3] = .05;

  return robot;
}

/*----------------------------------------------------------------------------*/
int main(int argc, char** argv) {
  ors::Graph graph;
  graph.init("doorSimple.ors");
  ors::Body* robot = createRobot(graph);
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
