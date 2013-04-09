/**
 * @file articulated_world.cpp
 * A simple test of the articulation package in conjunction with the
 * biros articulated world exploration.
 *
 * Use `roslaunch articulated_world.launch` to fire up a complete running
 * example.
 */

// ============================================================================
//
// MLR
//
#include "Processes.h"
#include "Variables.h"

#include "MT/util.h"
#include "MT/array.h"
#include "biros/biros.h"
#include <biros/biros_views.h>

#include <MT/ors_physx.h>
#include <MT/opengl.h>

#include <motion/motion.h>


// ============================================================================
//
// ROS
//
#include "ros/ros.h"

// msg & srv
#include "articulation_msgs/TrackModelSrv.h"
#include "articulation_msgs/TrackMsg.h"
#include "geometry_msgs/Point.h"

// ============================================================================
//
// MISC
//
#include <iostream>


// ============================================================================
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


// ============================================================================
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
  // gl.update();
}

// ============================================================================
int ors_test(int argc, char** argv) {

  argv[argc] = "-ors_file";
  argc++;
  argv[argc] = "kitchen.ors";
  argc++;

  cout << argc << endl;
  for (int i = 0; i < argc; ++i) {
    cout << argv[i] << endl;
  }

  MT::initCmdLine(argc, argv);

  // Load ORS into a varbiable
  GeometricState geometricState;
  ors::Graph& mainGraph = geometricState.get_ors();

  // add simple robot to mainGraph
  ors::Body* robot = createRobot(mainGraph);
  // createRobot(mainGraph);
  // mainGraph.calcBodyFramesFromJoints();

  OpenGL glMy;
  OpenGL glPh("PhysX");
  OpenGL glBelief("Belief");

  PhysXInterface physx;
  bindOrsToOpenGL(mainGraph, glMy);
  bindOrsToPhysX(mainGraph, glPh, physx);

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
  WorldStateHypothesis worldStateHypothesis;

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
  worldStateProviderP.worldHypothesis = &worldStateHypothesis;
  worldStateProviderP.worldState = &worldStateVar;

  bindOrsToOpenGL(worldStateHypothesis.graph, glBelief);

  // collect processes...
  ProcessL processes;
  processes.append(&fakePerceptionP);
  processes.append(&cognitionP);
  processes.append(&worldStateProviderP);

  // WORK
  Metronome ticcer("ticcer", 100);
  for (uint i = 0; i < 10000; i++) {
    stepInSequenceThreaded(processes);

    // move (kinematic) robot
    robot->X.pos += movementRequstVar.control_u;

    // ors stuff
    // mainGraph.calcBodyFramesFromJoints();
    mainGraph.calcShapeFramesFromBodies();
    // mainGraph.calcJointsFromBodies();

    // update sim
    physx.step();
    glMy.update();
    glPh.update();
    glBelief.update();

    // ticcer.waitForTic();
    cout << "--------------------------------------------------------" << endl;
  }

  return 0;
}


// ============================================================================
/**
 * Create a trajectory for the given `model` with `n` samples and
 * the noise of `sigma_position`.
 */
articulation_msgs::TrackMsg getSampleTrack(size_t n = 100, double sigma_position = 0.02) {
  articulation_msgs::TrackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/";
  msg.id = 0;

  for (size_t i = 0; i < n; ++i) {
    double q = i / double(n);
    // WTF!?! no proper c'tor?
    // The code in python looks much better!
    geometry_msgs::Point point;
    point.x = q;
    point.y = 0.;
    point.y = 0.;
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.;
    quaternion.y = 0.;
    quaternion.z = 0.;
    quaternion.w = 1.;
    geometry_msgs::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;

    /* geometry_msgs::Pose pose = geometry_msgs::Pose( */
    /* geometry_msgs::Point(q, 0, 0), */
    /* geometry_msgs::Quaternion(0, 0, 0, 1)); */
    /*  elif model == ROTATIONAL: */
    /*    pose = Pose(Point(numpy.sin(q), numpy.cos(q) - 1.0, 0), */
    /*          Quaternion(0, 0, 0, 1)) */

    /* # TODO add noise */
    /*  pose.position.x += numpy.random.rand()*sigma_position */
    /*  pose.position.y += numpy.random.rand()*sigma_position */
    /*  pose.position.z += numpy.random.rand()*sigma_position */

    msg.pose.push_back(pose);
  }
  return msg;
}


// ============================================================================
int main(int argc, char** argv) {
  ors_test(argc, argv);
  return 0;

  // test biros
  biros();
  biros().dump();

  ros::init(argc, argv, "articulated_world_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient
                              <articulation_msgs::TrackModelSrv>("model_select");
  articulation_msgs::TrackModelSrv srv;

  // test a few tracks
  for(int i = 0; i < 200; i++) {
    // fill srv.request
    srv.request.model.track = getSampleTrack();
    /* std::cout << srv.request.model << std::endl; */
    if(client.call(srv)) {
      printf("selected model: '%s' (n = %ld, log LH = TODO)\n",
             srv.response.model.name.c_str(),
             srv.response.model.track.pose.size()
             /* [entry.value for entry in response.model.params if entry.name=='loglikelihood'][0] */
            );

      cout << "parameters ";
      for(size_t j = 0; j < srv.response.model.params.size(); j++) {
        std::cout << srv.response.model.params[j].name << ": "
                  << srv.response.model.params[j].value << " "
                  << std::endl;
      }
    } else {
      printf("call did not work\n");
    }
  }
}
