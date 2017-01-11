#pragma once

#include <climits>
#include <Core/array.h>
#include <Core/graph.h>
#include <Kin/kin.h>
#include "utilAB.h"

#define Target_list(f) f(NO_TARGET) f(TS) f(CE)
ENUM_MACRO_H(Target)

#define Thickness_list(f) f(THICK) f(THIN)
ENUM_MACRO_H(Thickness)

struct MocapID;
struct MocapRec;
struct MocapData;
struct MocapSeq;

typedef Graph MocapLabel; 

typedef mlr::Array<MocapRec*> MocapRecL;
typedef mlr::Array<MocapSeq*> MocapSeqL;

// =============================================================================
// MocapID
//

struct MocapID {
  struct sMocapID;
  sMocapID *s;

  MocapID();
  ~MocapID();

  void clear();
  void load(const char *meta);
  void load_json(const char *json);

  int root_id();

  int sensor_id(const char *sensor) const;
  int sensor_rn(const char *sensor) const;
  const String &sensor_type(const char *sensor) const;
  const String &sensor_mesh(const char *sensor) const;
  const mlr::Transformation &sensor_meshtr(const char *sensor) const;
  const String &root() const;


  const String& sensor(uint i) const;
  const StringA& sensors() const;
  const StringA& sensors_struct() const;
  const StringA& sensors_unstruct() const;

  StringA sensorsof(const char *obj) const;

  const StringA& agents() const;
  const StringA& agent_limbs() const;
  const StringA& agent_digits() const;

  const StringA& objects() const;
  const StringA& object_parts() const;

  template <class T>
  mlr::Array<T> query(const mlr::Array<T> &data, const String &sensor);
  template <class T>
  mlr::Array<T> query(const mlr::Array<T> &data, const StringA &sensors);

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(MocapID);

// =============================================================================
// MocapRec
//

struct MocapRec: Parametric {
  String dir;
  MocapID mid;
  MocapLabel mlabel;

  Graph kvg, kvgann;

  // /human/rh/thumb TS_ground_truth
  // /CE_ground_truth

  Graph agent_targets, object_targets;
  StringA targets;
  // Params params; // TODO ideally, I'd want to use the param class here..

  uint nsensors;
  uint nframes, nframes_thin;

  MocapRec();
  virtual ~MocapRec();

  // TODO ideally, I'd want to use the param class here..
  void setDefaultParams();

  virtual MocapRec *clone() = 0;
  virtual bool loadable(const char *recdir) = 0;
  virtual void load(const char *recdir) = 0;
  void save();

  const MocapID &id() const;
  MocapID &id();
  const MocapLabel &label() const;
  MocapLabel &label();

  // arr ann(Target type, const char *sensor1, const char *sensor2) const;
  // Graph& ann(const char *sensor1, const char *sensor2);

  uint numSensors() const;
  uint numFrames(Thickness thickness) const;
  uint numDim(const char *bam);

  void appendBam(const char *bam, const arr &data);
  bool hasBam(const char *type);
  arr query(const char *type);
  arr query(const char *type, const char *sensor);
  arr query(const char *type, const char *sensor, uint f);
  arr query(const char *type, const StringA &sensors);
  arr query(const char *type, const StringA &sensors, uint f);

  void computeDPos(const char *frame_sensor, const char *sensor);
  void computeDQuat(const char *frame_sensor, const char *sensor);
  // void computeDDist();
  // void computeIDist();

  void computeVarPast(const char *type, const char *sensor);
  void computeVarPast(const StringA &types, const StringA &sensors);
  void computeLinCoeffPast(const char *type, const char *sensor);
  void computeLinCoeffPast(const StringA &types, const StringA &sensors);

  void computeVarFuture(const char *type, const char *sensor);
  void computeVarFuture(const StringA &types, const StringA &sensors);
  void computeLinCoeffFuture(const char *type, const char *sensor);
  void computeLinCoeffFuture(const StringA &types, const StringA &sensors);

  MocapSeq *seq(const char *sens1, const char *sens2);
  MocapSeqL seqlist(const char *obj1 = nullptr, const char *obj2 = nullptr);

  mlr::KinematicWorld *newKW();

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(MocapRec);

// =============================================================================
// MocapData
//

struct MocapData {
  StringA bases;
  Graph kvg;
  MocapRecL reclist;

  MocapData();
  ~MocapData();

  StringA &base();
  MocapRec &rec(const char *recdir);

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(MocapData);

// =============================================================================
// MocapSeq
//

struct MocapSeq: Parametric {
  MocapRec &rec;

  String sensor1, sensor2;
  arr rawdata, featdata, data;
  uint nframes, nframes_thin;
  arr ann, ann_thin;

  MocapSeq(MocapRec &rec);
  MocapSeq(const MocapSeq &seq) = delete;
  ~MocapSeq();

  void init(const char *sens1, const char *sens2);
  void setAnn(const String &target);
  void clearFeatData();
  void appendFeatData(const arr &new_featdata);

  void appendObs();
  // void appendDDist();
  // void appendIDist();
  void appendVarPast();
  void appendLinCoeffPast(bool sqr_feats);
  void appendVarFuture();
  void appendLinCoeffFuture(bool sqr_feats);

  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(MocapSeq);

#include "mocapdata_t.h"
