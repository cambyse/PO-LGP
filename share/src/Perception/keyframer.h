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

  arr getCorr(uint b1, uint b2, uint wlen);
  arr getCorr(const String &n1, const String &n2, uint wlen);
  arr getCorrPCA(uint b1, uint b2, uint wlen, uint npc);
  arr getCorrPCA(const String &n1, const String &n2, uint wlen, uint npc);

  /*
  MT::Array<arr> getCorrEnsemble(uint b1, uint b2, uintA &wlens, bool pca);
  MT::Array<arr> getCorrEnsemble(const String &n1, const String &n2, uintA &wlens, bool pca);
  */

  arr getAngle(uint b1, uint b2);
  arr getAngle(const String &n1, const String &n2);
  arr getAngleVar(uint b1, uint b2, uint wlen);
  arr getAngleVar(const String &n1, const String &n2, uint wlen);

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
  
  ProxyL getProxies(uint b1, uint b2);
  ProxyL getProxies(const String &n1, const String &n2);
  void calcProxies(uint b1, uint b2);
  void calcProxies(const String &n1, const String &n2);
  void clearProxies();

  // TODO probably change all indeces b1 and b2, into body indeces
  arr getDists(uint b1, uint b2);
  arr getDists(const String &n1, const String &n2);

  KeyFrameL getKeyFrames(const uintA &vit);
  void saveKeyFrameScreens(const KeyFrameL &keyframes, uint df = 60);

  void computeEvidences(arrL &rho, const arr &c, const arr &v, const arrL &theta);
  void Estep(arrL &ql, const arrL &theta, const arrL &rho);
  void Mstep(arrL &theta, const arrL &ql, const arr& c, const arr &v);
  void EM(uintA &vit, const arr &c, const arr &v);

  void viterbiZ(uintA &vit, arrL &theta, arrL &rho, const arr &c, const arr &v);
  //void viterbiZY(uintA &vit, arrL &theta, arrL &rho, const arr &c, const arr &v);
};

