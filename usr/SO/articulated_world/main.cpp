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
#include "Variables.h"
#include "Processes.h"

#include <biros/biros.h>
#include <biros/biros_views.h>

#include <MT/ors_physx.h>
#include <MT/opengl.h>

#include <motion/motion.h>


/*----------------------------------------------------------------------------*/
/**
 * @brief Create a simple spherical robot.
 *
 * @param graph add the robot to the graph.
 *
 * @return pointer to the robot body.
 */
ors::Body* createRobot(ors::KinematicWorld& graph) {
  ors::Body* robot = new ors::Body(graph);
  robot->X.setRandom();
  robot->X.pos.x -= .5;
  robot->X.pos.y -= .5;
  robot->X.pos.z += 1.;
  robot->name << "robot";
  robot->type = ors::kinematicBT;
  /* cout << "robot: " << robot->X << endl; */
  /* cout << "robot.pos.pos: " << robot->X.pos << endl; */

  ors::Shape* robotShape = new ors::Shape(graph, robot);
  robotShape->type = ors::sphereST;
  robotShape->size[0] = .05;
  robotShape->size[1] = .05;
  robotShape->size[2] = .05;
  robotShape->size[3] = .05;

  return robot;
}

/*----------------------------------------------------------------------------*/
void bindOrsToPhysX(ors::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx) {
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
  MT::initCmdLine(argc, argv);

  // Load ORS into a varbiable
  GeometricState geometricState;
  ors::KinematicWorld& graph = geometricState.get_ors();

  // add simple robot to graph
  ors::Body* robot = createRobot(graph);
  /* createRobot(graph); */
  // graph.calcBodyFramesFromJoints();

  OpenGL glMy;
  OpenGL glPh("PhysX");
  PhysXInterface physx;
  bindOrsToOpenGL(graph, glMy);
  bindOrsToPhysX(graph, glPh, physx);

  // SETUP BIROS
  biros().dump();
  InsideOut insideOutView();
  cout << " view cereated" << endl;


  // VARBIABLES
  // GeometricState was already created. See above.
  PerceptsVar perceptsVar;
  RobotPosVar robotPosVar;
  WorldStateVar worldStateVar;
  MovementRequestVar movementRequstVar;

  // PROCESSES
  FakePerceptionP fakePerceptionP;
  CognitionP cognitionP;
  WorldStateProvider worldStateProviderP;

  // CONNECT Processes with Variables
  fakePerceptionP.geometricState = &geometricState;
  fakePerceptionP.percepts = &perceptsVar;
  fakePerceptionP.robot = &robotPosVar;

  cognitionP.percepts = &perceptsVar;
  cognitionP.robotPos = &robotPosVar;
  cognitionP.worldState = &worldStateVar;

  cognitionP.movementRequest = &movementRequstVar;

  worldStateProviderP.geometricState = &geometricState;
  worldStateProviderP.worldState = &worldStateVar;


  ProcessL processes;
  processes.append(&fakePerceptionP);
  processes.append(&cognitionP);
  processes.append(&worldStateProviderP);

  // WORK
  Metronome ticcer("ticcer", 100);
  for (uint i = 0; i < 10000; i++) {
    stepInSequenceThreaded(processes);

    robot->X.pos += movementRequstVar.control_u;

    /* graph.calcBodyFramesFromJoints(); */
    graph.calcShapeFramesFromBodies();
    /* graph.calcJointsFromBodies(); */

    // update sim
    physx.step();
    glMy.update();
    glPh.update();

    /* ticcer.waitForTic(); */
  }

  return 0;
}






















// vim: ts=2:sw=2:expandtab
