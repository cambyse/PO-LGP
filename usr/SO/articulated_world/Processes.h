#ifndef PROCESSES_H__
#define PROCESSES_H__

#include <biros/biros.h>
#include "Variables.h"
#include <motion/motion.h>

// ============================================================================
// Fake Perception
struct FakePerceptionP : public Process {

  // Variables
  GeometricState* geometricState;
  PerceptsVar* percepts;
  RobotPosVar* robot;

  FakePerceptionP() : Process("fake perception") {}
  virtual ~FakePerceptionP() {}

  void open() {};
  void close() {};
  void step();
};


// ============================================================================
// Cognition
struct CognitionP : public Process {
  PerceptsVar* percepts;
  /* WorldStateVar* worldState; */
  MovementRequestVar* movementRequest;
  RobotPosVar* robotPos;

  CognitionP() : Process("Cognition") {};

  virtual ~CognitionP() {};

  void open() {};
  void close() {};
  void step();
};


#endif /* end of include guard: PROCESSES_H__ */
