#include "simDrake.h"

#include "sim.h"
#include "filter.h"

#include <Gui/opengl.h>
#include <Kin/frame.h>

#include <Drake/drake.h>

SimDrake::SimDrake(double dt) : Thread("SimDrake", dt), dt(dt),
  path(this, "path"),
  currentQ(this, "currentQ"),
  nextQ(this, "nextQ"),
  switches(this, "switches"),
  world(this, "world"),
  timeToGo(this, "timeToGo"),
  percepts_input(this, "percepts_input"){
}

void SimDrake::step(){
  drake::MyDrake D(mlr::argc, mlr::argv);
  D.DoMain();
}
