#pragma once

#include <Core/array.h>
#include <Ors/ors.h>
#include <Perception/g4data.h>

typedef MT::Array<KeyValueGraph*> kvgL;

struct KeyFramer {
  struct sKeyFramer;
  sKeyFramer *s;

  KeyFramer();
  ~KeyFramer();

  ors::KinematicWorld& kw();
  G4Data& g4d();
  //GraphicalModel& model() { NIY; }

  void updateOrs(uint f, bool show);

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

  void computeDist();
  void computeDist(uint f);
  void testDist(KeyValueGraph &kvg, const String &a, const String &b);

  void EM_z(KeyValueGraph &kvg, const StringA &bAs, const String &bB);
  void EM_z(KeyValueGraph &kvg, const String &bA, const String &bB);
  void EM_z_with_c(KeyValueGraph &kvg, const String &bA, const String &bB);
  void EM_c(KeyValueGraph &kvg, const String &bA, const String &bB);
  void EM_r(KeyValueGraph &kvg, const String &bA, const String &bB);
  void EM_m(KeyValueGraph &kvg, const String &b);

  void testSmoothing(KeyValueGraph &kvg, const String &b, double alpha);

  void getVitSeq(arr &vit, const String &b1, const String &b2);
  void getVitSeq(arr &vit, const StringA &b1s, const String &b2);
  void vitLogicMachine(KeyValueGraph &kvg, arr &vit2, const arr &vit);
  void getCtrlSeq(kvgL &ctrls, const String &b1, const String &b2);
  void getCtrlSeq_old(kvgL &ctrls, const String &b1, const String &b2);
  void getDeltaSeq(kvgL &deltas, kvgL ctrls);
  void getDeltaCluster(kvgL &deltas, kvgL ctrls);

  void objFeatures(KeyValueGraph &feats, const String &b, uint fnum);
  void trainOnDeltas(const kvgL &deltas);

  // High-level methods: can be invoked on any level of subject
  void process(KeyValueGraph &kvg, const String &subj, const String &obj);
  void process(KeyValueGraph &kvg, const StringA &subj, const StringA &objs);
  void play();
  void playScene(KeyValueGraph &kvg, const String &name_subj, bool record = false);
  void playScene(KeyValueGraph &kvg, const StringA &name_subj, bool record = false);

  arr annOf(const String &sensor1, const String &sensor2);
  void load_ann(const String &dir);

  void dlib_train(const String &basedir, const String &traindir, uint wlen);
  void dlib_test(KeyValueGraph &kvg, const String &name_subj, const String &name_obj, uint wlen);

};

