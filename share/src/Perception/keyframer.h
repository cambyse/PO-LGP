#pragma once

#include <Core/array.h>
#include <Ors/ors.h>
#include <Perception/g4data.h>
#include "keyframe.h"

struct KeyFramer {
  struct sKeyFramer;
  sKeyFramer *s;

  KeyFramer(ors::KinematicWorld &kw, G4Data &g4d);
  ~KeyFramer();

  void updateOrs(uint f);

  arr getCorr(uint b1, uint b2, uint wlen);
  arr getCorr(const String &n1, const String &n2, uint wlen);
  arr getCorrPCA(uint b1, uint b2, uint wlen, uint npc);
  arr getCorrPCA(const String &n1, const String &n2, uint wlen, uint npc);

  arr getState(uint b);
  arr getState(const String &n);
  arr getStateVar(uint b, uint wlen);
  arr getStateVar(const String &n, uint wlen);

  arr getAngle(uint b1, uint b2);
  arr getAngle(const String &n1, const String &n2);
  arr getAngleVar(uint b1, uint b2, uint wlen);
  arr getAngleVar(const String &n1, const String &n2, uint wlen);

  arr getQuat(uint b1, uint b2);
  arr getQuat(const String &n1, const String &n2);
  arr getQuatVar(uint b1, uint b2, uint wlen);
  arr getQuatVar(const String &n1, const String &n2, uint wlen);

  arr getPos(uint b1, uint b2);
  arr getPos(const String &n1, const String &n2);
  arr getPosVar(uint b1, uint b2, uint wlen);
  arr getPosVar(const String &n1, const String &n2, uint wlen);

  arr getDiff(uint b1, uint b2);
  arr getDiff(const String &n1, const String &n2);
  arr getDiffVar(uint b1, uint b2, uint wlen);
  arr getDiffVar(const String &n1, const String &n2, uint wlen);

  arr getPosLen(uint b1, uint b2);
  arr getPosLen(const String &n1, const String &n2);
  arr getPosLenVar(uint b1, uint b2, uint wlen);
  arr getPosLenVar(const String &n1, const String &n2, uint wlen);

  arr getTransfVar(uint b1, uint b2, uint wlen);
  arr getTransfVar(const String &n1, const String &n2, uint wlen);
  
  KeyFrameL getKeyFrames(const uintA &vit);
  void saveKeyFrameScreens(const KeyFrameL &keyframes, uint df = 60);

  void EM(uintA &vit, const String &b1, const String &b2, uint wlen);
};

