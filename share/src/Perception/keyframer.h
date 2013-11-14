#pragma once

#include <Core/array.h>

struct KeyFramer {
  struct sKeyFramer;
  sKeyFramer *s;

  KeyFramer();
  ~KeyFramer();

  void addBody(uint body_ndof);
  uint getNBodies();
  uint getNDofs(uint b);
  uint getNDofs();
  uint getAgentNDofs();
  uint getCumNDofs(uint b);
  uint getNFrames();

  void clear();
  void clearState();
  void clearFrames();
  void addState(arr st);
  void addState(arr st, uint f);

  arr getState(uint f);
  arr getState(uint f, uint b);
  arr getWindow(uint f);
  arr getWindow(uint f, uint b);

  void setAgent(uint a);
  void setLWin(uint l);

  void run();
  arr getErr();
  arr getErr(uint b);
  void keyframes();

  void test();
};

