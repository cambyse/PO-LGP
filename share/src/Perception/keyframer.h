#pragma once

#include <Core/array.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/g4data.h>
#include "keyframe.h"

struct KeyFramer {
  struct sKeyFramer;
  sKeyFramer *s;

  KeyFramer(ors::KinematicWorld &G, G4Data &g4d);
  ~KeyFramer();

  //uint getNBodies();
  //uint getNDofs(uint b);
  //uint getNDofs();
  //uint getCumNDofs(uint b);
  //uint getNFrames();
  //uint getNWindows(uint wlen);

  //arr getState();
  //arr getState(uint f);
  //arr getState(uint f, uint b);
  //arr getWindow(uint f);
  //arr getWindow(uint f, uint b);

  void updateOrs(uint f);

  arr getCorrPCA(uint b1, uint b2, uint wlen);
  arr getCorrWOPCA(uint b1, uint b2, uint wlen);
  arr getCorr(uint b1, uint b2, uint wlen, bool pca);
  arr getCorr(const String &n1, const String &n2, uint wlen, bool pca);
  /*
  MT::Array<arr> getCorrEnsemble(uint b1, uint b2, uintA &wlens, bool pca);
  MT::Array<arr> getCorrEnsemble(const String &n1, const String &n2, uintA &wlens, bool pca);
  */

  arr getAngle(uint b1, uint b2);
  arr getAngle(const String &n1, const String &n2);
  arr getAngleVar(uint b1, uint b2, uint wlen);
  arr getAngleVar(const String &n1, const String &n2, uint wlen);

  ProxyL getProxies(uint b1, uint b2);
  ProxyL getProxies(const String &n1, const String &n2);
  void calcProxies(uint b1, uint b2);
  void calcProxies(const String &n1, const String &n2);
  void clearProxies();

  // TODO probably change all indeces b1 and b2, into body indeces
  arr getDists(uint b1, uint b2);
  arr getDists(const String &n1, const String &n2);

  KeyFrameL getKeyFrames(const arr &corr, const ProxyL &proxies);
  void saveKeyFrameScreens(const KeyFrameL &keyframes, uint df = 60);
};

