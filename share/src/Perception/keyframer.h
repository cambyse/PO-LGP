#pragma once

#include <Core/array.h>

struct KeyFramer {
  struct sKeyFramer;
  sKeyFramer *s;

  KeyFramer();
  ~KeyFramer();

  void addBody(String &name, uint body_ndof);
  uint getNBodies();
  uint getNDofs(uint b);
  uint getNDofs();
  uint getCumNDofs(uint b);
  uint getNFrames();
  uint getNWindows(uint wlen);

  void clear();
  void clearState();
  void clearFrames();

  void addState(arr st);
  void addState();

  void setState(uint b, arr st, uint f);
  void setState(String n, arr st, uint f);
  void setState(arr st, uint f);
  void setupWindows(MT::Array<arr> &wins, uint wlen);

  arr getState();
  arr getState(uint f);
  arr getState(uint f, uint b);
  //arr getWindow(uint f);
  //arr getWindow(uint f, uint b);

  arr getCorrPCA(uint b1, uint b2, uint wlen);
  arr getCorrWOPCA(uint b1, uint b2, uint wlen);
  arr getCorr(uint b1, uint b2, uint wlen, bool pca);
  MT::Array<arr> getCorrEnsemble(uint b1, uint b2, uintA &wlens, bool pca);
  MT::Array<arr> getCorrEnsemble(const String &n1, const String &n2, uintA &wlens, bool pca);

  arr getAngle(uint b1, uint b2);
  arr getAngle(const String &n1, const String &n2);
  arr getAngleVar(uint b1, uint b2, uint wlen);
  arr getAngleVar(const String &n1, const String &n2, uint wlen);

  // TODO define keyframe struct
  void keyframes();
};

