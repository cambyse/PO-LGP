#pragma once

#include <Core/array.h>

struct MyIIWA{
  struct sMyIIWA *s;

  MyIIWA(int argc, char* argv[]);
  ~MyIIWA();

  void addKukaPlant();
  void addController();
  void addLogger();
  void addReferenceTrajectory(const arr& knots=NoArr);

  void build();

  void simulate();

  arr getLog();
};
