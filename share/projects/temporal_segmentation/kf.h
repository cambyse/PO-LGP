#pragma once
#include <Core/array.h>
#include <Core/keyValueGraph.h>
#include <Core/util.h>
#include <Perception/g4data.h>
#include <iomanip>
#include "util.h"

// KFScore {{{
struct KFScore {
  uint tot, totpos, totneg;
  uint scores[2][2];

  KFScore();
  KFScore(const arr &t, const arr &y);

  void clear();

  double tpr() const;
  double fpr() const;
  double accuracy() const;
  double f1() const;

  KFScore operator+(const KFScore &x);
  KFScore& operator+=(const KFScore &x);
  void write(std::ostream &os = std::cout) const;
};
stdOutPipe(KFScore)
// KFScore }}}
// ROCCurve {{{
struct ROCCurve {
    arr thresholds;
    uintA ind;
    MT::Array<KFScore> scores;

    void append(double thresh, const KFScore &score);
    double area();
    void show(const char *pdf = NULL);
  private:
    void sort();
};
// }}}
// KFModel {{{
struct KFModel: Parametric {
  /* protected: */
  /*   virtual void setDefaultParams() = 0; */
  public:
    virtual void train(G4FeatSeqL &seqlist) = 0;
    virtual arr query(G4FeatSeq &seq) = 0;

    KFScore test(const G4FeatSeqL &seqlist);
    KFScore crossval(const G4FeatSeqL &seqlist);
    ROCCurve ROC(const G4FeatSeqL &seqlist, const arr &thresholds);
};
// KFModel }}}
// KFThresh {{{
struct KFThresh: KFModel {
  private:
    double threshPos, threshQuat, threshDPos, threshDQuat;

  public:
    KFThresh();
    void setDefaultParams();

    void train(G4FeatSeqL &seqlist);
    arr query(G4FeatSeq &seq);
};
// KFThresh }}}
// KFLogitReg {{{
struct KFLogitReg: KFModel {
  private:
    arr data, target, beta;
    double thresh;

  public:
    KFLogitReg();
    void setDefaultParams();

    void train(G4FeatSeqL &seqlist);
    arr query(G4FeatSeq &seq);
    arr sigma(G4FeatSeq &seq);
};
// KFLogitReg }}}
// KFLogitCRF {{{
struct KFLogitCRF: KFLogitReg {
  private:
    arr A;

  public:
    KFLogitCRF();
    void setDefaultParams();

    void train(G4FeatSeqL &seqlist);
    arr query(G4FeatSeq &seq);
};
// KFLogitCRF }}}
// PhiMachine {{{
struct PhiMachine {
  std::function<arr(arr)> phi;

  PhiMachine(): phi([](arr x){ return x; }) { }
  void set(std::function<arr(arr)> _phi) {
    phi = _phi;
  }
  
  arr convert(arr data) {
    return phi(data);
  }
};
std::function<arr(arr)> PhiBias();
std::function<arr(arr)> PhiLin();
std::function<arr(arr)> PhiPoly(double c, double d);
// }}}
struct KFPlayer {
  
};
