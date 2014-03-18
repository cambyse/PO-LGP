#pragma once

#include <Core/array.h>
#include <Ors/ors.h>
#include <Perception/g4data.h>

typedef MT::Array<KeyValueGraph*> kvgL;

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

  void computeVar(const String &type, uint wlen, bool force = false);
  void computeVar(const StringA &types, uint wlen, bool force = false);
  void computeSpline(const StringA &types, double lambda, bool force = false);
  void computeSpline(const String &type, double lambda, bool force = false);
  void computeFilter(const StringA &types, double alpha, bool force = false);
  void computeFilter(const String &type, double alpha, bool force = false);
  void computeSmooth(const StringA &types, double alpha, bool force = false);
  void computeSmooth(const String &type, double alpha, bool force = false);
  void computeSpeed(const StringA &types, bool force = false);
  void computeSpeed(const String &type, bool force = false);
  void computeGP(const StringA &types, bool force = false);
  void computeGP(const String &type, bool force = false);
  void computeDPos(const String &b, bool force = false);
  void computeDQuat(const String &b, bool force = false);

  arr getState(uint b);
  arr getState(const String &n);
  arr getStateVar(uint b, uint wlen);
  arr getStateVar(const String &n, uint wlen);
  arr getStateSpeed(uint b);
  arr getStateSpeed(const String &n);

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
  
  void EM_c(KeyValueGraph &kvg, const StringA &bAs, const String &bB);
  void EM_c(KeyValueGraph &kvg, const String &bA, const String &bB);
  void EM_r(KeyValueGraph &kvg, const String &bA, const String &bB);
  void EM_m(KeyValueGraph &kvg, const String &b);

  void playScene(const String &b);
  void playScene(const StringA &bb);
  void testSmoothing(KeyValueGraph &kvg, const String &b, double alpha);

  void getCtrlSeq(kvgL &ctrls, const String &b1, const String &b2);
  void getDeltaSeq(kvgL &deltas, kvgL ctrls);

  void objFeatures(KeyValueGraph &feats, const String &b, uint fnum);
};

