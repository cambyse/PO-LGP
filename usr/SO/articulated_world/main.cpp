/**
 * @file main.cpp
 * @brief A test scenario for a curious robot in an actuated world.
 *
 * The robot should expore the world and "understand" its DoFs.
 *
 * @author stefan otte
 * @version 0.0.1
 * @date 2013-01-09
 */


/*----------------------------------------------------------------------------*/
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
  robot->X.pos.z += 1.;
  robot->name << "robot";
  robot->type = ors::kinematicBT;
  /* cout << "robot: " << robot->X << endl; */
  /* cout << "robot.pos.pos: " << robot->X.pos << endl; */

  ors::Shape* robotShape = new ors::Shape(graph, robot);
  robotShape->type = ors::sphereST;
  robotShape->size[0] = .1;
  robotShape->size[1] = .1;
  robotShape->size[2] = .1;
  robotShape->size[3] = .1;
  return robot;
}

/*----------------------------------------------------------------------------*/
void bindOrsToPhysX(ors::Graph& graph, OpenGL& gl, PhysXInterface& physx) {
  physx.G = &graph;
  physx.create();

  gl.add(glStandardScene, NULL);
  gl.add(glPhysXInterface, &physx);
  gl.setClearColors(1., 1., 1., 1.);

  ors::Body* glCamera = graph.getBodyByName("glCamera");
  if (glCamera) {
    *(gl.camera.X) = glCamera->X;
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.watch();
  /* gl.update(); */
}

/*----------------------------------------------------------------------------*/
int main(int argc, char** argv) {
  // SETUP
  // load ors file
  std::string file("test.ors");
  if (argc == 2) {
    file = std::string(argv[1]);
  }
  cout << "Loading " << file << endl;
  ors::Graph graph;
  graph.init(file.c_str());

  // add simple robot to graph
  ors::Body* robot = createRobot(graph);
  graph.calcBodyFramesFromJoints();

  OpenGL glMy;
  OpenGL glPh("PhysX");
  PhysXInterface physx;

  /* init(graph, glMy, file.c_str()); */
  bindOrsToOpenGL(graph, glMy);
  bindOrsToPhysX(graph, glPh, physx);

  /* physx.G = &graph; */
  /* physx.create(); */
  /* glPh.add(glStandardScene, NULL); */
  /* glPh.add(glPhysXInterface, &physx); */
  /* glPh.setClearColors(1., 1., 1., 1.); */
  /* glPh.camera.setPosition(10., -15., 8.); */
  /* glPh.camera.focus(0, 0, 1.); */
  /* glPh.camera.upright(); */
  /* glPh.watch(); */

  // WORK
  double robotSpeedX = 0.002;
  double robotSpeedY = 0.005;
  for (uint t = 0; t < 500; t++) {
    cout << "\r t=" << t << std::flush;

    // move the robot
    robot->X.pos.y += robotSpeedY;
    robot->X.pos.x -= robotSpeedX;

    // update sim
    physx.step();
    glPh.update();
    glMy.update();
  }
  return 0;
}






















// vim: ts=2:sw=2:noexpandtab
