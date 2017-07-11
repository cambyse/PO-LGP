#pragma once

#include <Core/thread.h>
#include "percept.h"

struct PercViewer : Thread {
  Access<PerceptL> percepts;
  Access<mlr::KinematicWorld> modelWorld;
  PerceptL copy;
  MeshA modelCopy;
  struct OpenGL *gl;

  PercViewer(const char* percepts_name="percepts_input");
  ~PercViewer();
  void open();
  void step();
  void close();
};
