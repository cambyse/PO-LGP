#pragma once

#include <Core/array.h>

namespace drake{
struct sMyDrake;

struct MyDrake{
  sMyDrake *s;

  MyDrake(int argc, char* argv[]);
  ~MyDrake();

  int DoMain(const arr& X);

  void addKukaPlant();
  void addController();
  void addLogger();
  void addReferenceTrajectory(const arr& knots=NoArr);

  void build();

  void simulate();
  void simulate2();

  arr getLog();

  //refactored from the mono demo
  void mono_setupGeometry();
  void addMonoPlant();
  void addWsgController();
  void addPlanInterpolator();
  void addStateMachine();
  void addRAIMachine();


  //high-level
  void setPath(const arr& X);
  void setGrip(double x);
};
}
