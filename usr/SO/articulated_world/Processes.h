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
class CognitionP : public Process {
public:
  PerceptsVar* percepts;
  WorldStateVar* worldState;
  MovementRequestVar* movementRequest;
  RobotPosVar* robotPos;

  CognitionP()
    : Process("Cognition")
    , zeroControl(0, 0, 0)
  {};
  virtual ~CognitionP() {};

  void open() {};
  void close() {};
  void step();

private:
  ors::Vector zeroControl;
};

// ============================================================================
// WorldStateProvider
class WorldStateProvider : public Process {
public:
  GeometricState* geometricState;
  WorldStateVar* worldState;

  WorldStateProvider()
    : Process("WorldStateProvider")
    , firstRun(true)
  {};
  virtual ~WorldStateProvider() {};

  void open() {};
  void close() {};
  void step();

private:
  bool firstRun;
  arr previous;
};

#endif /* end of include guard: PROCESSES_H__ */
