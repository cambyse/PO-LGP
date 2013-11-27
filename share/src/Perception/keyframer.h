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

  arr getState();
  arr getState(uint f);
  arr getState(uint f, uint b);
  //arr getWindow(uint f);
  //arr getWindow(uint f, uint b);

  void setAgent(uint a);
  void setLWin(uint l);

  arr getCorrPCA(uint b1, uint b2, uint wlen);
  arr getCorrWOPCA(uint b1, uint b2, uint wlen);
  arr getCorr(uint b1, uint b2, uint wlen, bool pca);
  MT::Array<arr> getCorrEnsemble(uint b1, uint b2, uintA wlens, bool pca);

  // TODO define keyframe struct
  void keyframes();
};

