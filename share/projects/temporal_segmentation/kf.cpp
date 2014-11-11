#include "kf.h"
#include "gui.h"
#include "util.h"
#include "optim.h"

// KFScore {{{
KFScore::KFScore() { clear(); }
KFScore::KFScore(const arr &t, const arr &y) {
  CHECK_EQ(t.N , y.N, "Output and target arrays must have the same size.");
  clear();
  tot = t.N;
  totpos = sum(t);
  totneg = tot - totpos;
  // TODO change types of target and y to uintA..
  for(uint i = 0; i < tot; i++)
    scores[(uint)y(i)][(uint)t(i)]++;
}

void KFScore::clear() {
  tot = totpos = totneg = 0;
  memset(scores, 0, 4 * sizeof(uint));
}

double KFScore::tpr() const { return double(scores[1][1]) / totpos; }
double KFScore::fpr() const { return double(scores[1][0]) / totneg; }
double KFScore::accuracy() const { return double(scores[0][0] + scores[1][1]) / tot; }
double KFScore::f1() const { return 2.*scores[1][1] / (2.*scores[1][1] + scores[1][0] + scores[0][1]); }

KFScore KFScore::operator+(const KFScore &x) {
  KFScore s;
  s.tot = tot + x.tot;
  s.totpos = totpos + x.totpos;
  s.totneg = totneg + x.totneg;
  s.scores[0][0] = scores[0][0] + x.scores[0][0];
  s.scores[0][1] = scores[0][1] + x.scores[0][1];
  s.scores[1][0] = scores[1][0] + x.scores[1][0];
  s.scores[1][1] = scores[1][1] + x.scores[1][1];
  return s;
}

KFScore& KFScore::operator+=(const KFScore &x) {
  tot += x.tot;
  totpos += x.totpos;
  totneg += x.totneg;
  scores[0][0] += x.scores[0][0];
  scores[0][1] += x.scores[0][1];
  scores[1][0] += x.scores[1][0];
  scores[1][1] += x.scores[1][1];
  return *this;
}

void KFScore::write(std::ostream &os) const {
  os << "scores" << endl;
  os << "======" << endl;
  os << "tot: " << tot << " (neg: " << totneg << ", pos: " << totpos << ")" << endl;
  os << "     t=0    t=1" << endl;
  os << "y=0 " << std::setw(5) << std::fixed << std::setprecision(2) << (1-fpr())
      << " " << std::setw(5) << std::fixed << std::setprecision(2) << (1-tpr()) << endl;
  os << "y=1 " << std::setw(5) << std::fixed << std::setprecision(2) << fpr()
      << " " << std::setw(5) << std::fixed << std::setprecision(2) << tpr() << endl;
  os << "accuracy: " << accuracy() << endl;
  os << "f1: " << f1() << endl;
}
// KFScore }}}
// ROCCurve {{{
void ROCCurve::append(double thresh, const KFScore &score) {
  thresholds.append(thresh);
  scores.append(score);
  ind.append(0);
}
double ROCCurve::area() {
  sort();

  double tpr1, tpr2, fpr1, fpr2;
  double a = 0;
  for(uint i = 0; i < ind.N -1; i++) {
    tpr1 = scores(ind(i)).tpr();
    tpr2 = scores(ind(i+1)).tpr();
    fpr1 = scores(ind(i)).fpr();
    fpr2 = scores(ind(i+1)).fpr();
    a += .5 * (fpr2 - fpr1) * (tpr1 + tpr2);
  }
  a += .5 * scores(ind(0)).fpr() * scores(ind(0)).tpr()
       + .5 * (1 - scores(ind.last()).fpr()) * (1 + scores(ind.last()).tpr());
  return a;
}
void ROCCurve::show(const char *pdf) {
  sort();

  FGP fgplot, fgplot_pdf;
  fgplot.set("domain", true);
  fgplot.set("lines", true);
  fgplot.set("ymin_b", true);
  fgplot.set("ymax_b", true);
  fgplot.set("ymin", 0.);
  fgplot.set("ymax", 1.);
  fgplot.open();
  if(pdf) {
    fgplot_pdf.set("domain", true);
    fgplot_pdf.set("lines", true);
    fgplot_pdf.set("ymin_b", true);
    fgplot_pdf.set("ymax_b", true);
    fgplot_pdf.set("ymin", 0.);
    fgplot_pdf.set("ymax", 1.);
    fgplot_pdf.set("hardcopy", pdf);
    fgplot_pdf.open();
  }
  for(uint i = 0; i < scores.N; i++) {
    fgplot() << scores(ind(i)).fpr() << " " << scores(ind(i)).tpr();
    if(pdf)
      fgplot_pdf() << scores(ind(i)).fpr() << " " << scores(ind(i)).tpr();
  }
  fgplot.close();
  if(pdf)
    fgplot_pdf.close();
}
void ROCCurve::sort() {
  if(scores.N > 1 && ind.last() == 0) {
    for(uint i = 0; i < ind.N; i++)
      ind(i) = i;
    std::sort(ind.begin(), ind.end(),
              [this](uint i1, uint i2) {
                return scores(i1).fpr() < scores(i2).fpr();
              });
  }
}
// }}}
// KFModel {{{
KFScore KFModel::test(const G4FeatSeqL &seqlist) {
  KFScore score;
  for(G4FeatSeq *seq: seqlist)
    score += KFScore(seq->ann, query(*seq));
  return score;
}

KFScore KFModel::crossval(const G4FeatSeqL &seqlist) {
  KFScore score;

  uint nsplits = *params.get<uint>("nsplits");

  MT::Array<G4FeatSeqL> trainseqlist, testseqlist;
  split(trainseqlist, testseqlist, seqlist, nsplits);
  for(uint n = 0; n < nsplits; n++) {
    // take nth training split
    train(trainseqlist(n));
    // take nth testing split
    score += test(testseqlist(n));
  }
  return score;
}

ROCCurve KFModel::ROC(const G4FeatSeqL &seqlist, const arr &thresholds) {
  ROCCurve roc;
  KFScore score;
  ProgressBar pb;

  pb.prefix << "Calc. ROC curve";
  pb.reset(thresholds.N);
  for(double thresh: thresholds) {
    params.set("thresh", thresh);
    score = crossval(seqlist);
    pb.step() << "thresh = " << thresh;
    roc.append(thresh, score);
  }
  pb.stop() << "done";
  return roc;
}
// }}}
// KFThresh {{{
KFThresh::KFThresh() { setDefaultParams(); }
void KFThresh::setDefaultParams() {
  params.clear();
  params.set("wlen", 100u);
}

void KFThresh::train(G4FeatSeqL &seqlist) {
}

arr KFThresh::query(G4FeatSeq &seq) {
  // FIXME set wlen according to the parameters
  // uint wlen = 100;

  // G4Data g4d;
  // arr target, y;

  // String obj1, obj2;

  KFScore score;

  // g4d.clear();
  // g4d.load(testdir, true);
  // uint nframes = g4d.numFrames();

  // g4d.computeVar(STRING("pos"), wlen);
  // g4d.computeVar(STRING("quat"), wlen);

  // for(auto pair: g4d.ann()) {
  //   obj1 = pair->keys(0);
  //   obj2 = pair->keys(1);
  //   target.referTo(*pair->getValue<KeyValueGraph>()->getValue<arr>("ann"));
  //   for(String sens1: g4d.id().sensorsof(obj1)) {
  //     for(String sens2: g4d.id().sensorsof(obj2)) {
  //       // TODO how to define features? some type of lambda?
  //       // TODO first do it as you would normally.. then think of how to generalize..
  //       String name_subj = sens1;
  //       String name_obj = sens2;

  //       String subj_dPos = STRING(name_subj << "_dPos");
  //       String subj_dQuat = STRING(name_subj << "_dQuat");
  //       String subj_dPosVar = STRING(subj_dPos << "Var");
  //       String subj_dQuatVar = STRING(subj_dQuat << "Var");

  //       g4d.computeDPos(name_subj);
  //       g4d.computeDQuat(name_subj);
  //       g4d.computeVar(subj_dPos, wlen);
  //       g4d.computeVar(subj_dQuat, wlen);

  //       // model observations
  //       arr subj_posVar = g4d.query("posVar", name_subj);
  //       arr subj_quatVar = g4d.query("quatVar", name_subj);
  //       arr obj_posVar = g4d.query("posVar", name_obj);
  //       arr obj_quatVar = g4d.query("quatVar", name_obj);
  //       arr delta_posVar = g4d.query(subj_dPosVar, name_obj);
  //       arr delta_quatVar = g4d.query(subj_dQuatVar, name_obj);

  //       y.resize(nframes);
  //       y.setZero();
  //       for(uint f = 0; f < nframes; f++)
  //         y(f) = subj_posVar(f) <= threshPos
  //             && subj_quatVar(f) <= threshQuat
  //             && obj_posVar(f) <= threshPos
  //             && obj_quatVar(f) <= threshQuat
  //             && delta_posVar(f) >= threshDPos
  //             && delta_quatVar(f) >= threshDQuat;
  //       score += KFScore(target, y);
  //     }
  //   }
  // }
  arr y;
  return y;
}
// KFThresh }}}
// KFLogitReg {{{
KFLogitReg::KFLogitReg() { setDefaultParams(); }
void KFLogitReg::setDefaultParams() {
  params.clear();
  params.set("thresh", .5);
}

void KFLogitReg::train(G4FeatSeqL &seqlist) {
  uint ndata, ndim;

  arr data, target;
  ndata = 0;
  for(G4FeatSeq *seq: seqlist) {
    ndata += seq->nframes_thin;
    data.append(seq->data);
    // TODO check that the annotation exists
    target.append(seq->ann_thin);
  }
  ndim = data.N / ndata;
  data.reshape(ndata, ndim);

  arr dataT = ~data;

  arr p;
  ObjectiveFunction objf;
  objf.jacobian = [&](const arr &beta) {
    p = data * beta;
    for(uint i = 0; i < p.N; i++)
      p.elem(i) = MT::sigmoid(p.elem(i));
    return dataT * (p - target);
  };

  objf.hessian = [&](const arr &beta) {
    uint ndim = dataT.d0;
    arr H(ndim, ndim);
    arr tmp_p = p % (1. - p);
    for(uint i = 0; i < ndim; i++)
      for(uint j = 0; j < ndim; j++)
        H(i, j) = scalarProduct(dataT[i] % tmp_p, dataT[j]);
    return H;
  };
  
  beta = zeros(ndim);

  Newton newt;
  newt.objf = objf;
  newt.loopUntil(beta, 1e-2);
}

arr KFLogitReg::query(G4FeatSeq &seq) {
  double thresh = *params.get<double>("thresh");

  arr s = sigma(seq);
  arr y(seq.nframes);
  for(uint f = 0; f < seq.nframes; f++)
    y(f) = (thresh <= s(f));

  return y;
}

arr KFLogitReg::sigma(G4FeatSeq &seq) {
  arr sigma_thin(seq.nframes_thin);
  arr sigma(seq.nframes);
  uint thinning = *seq.params.get<uint>("thinning");

  for(uint f_thin = 0; f_thin < seq.nframes_thin; f_thin++)
    sigma.subRange(f_thin * thinning, (f_thin + 1) * thinning - 1)() = MT::sigmoid(scalarProduct(beta, seq.data[f_thin]));
  if(seq.nframes_thin * thinning < seq.nframes)
    sigma.subRange(seq.nframes_thin * thinning + 1, -1)() = sigma_thin.last();

  return sigma;
}
// }}}
// KFLogitCRF {{{
KFLogitCRF::KFLogitCRF() { setDefaultParams(); }
void KFLogitCRF::setDefaultParams() {
  params.clear();
  KFLogitReg::setDefaultParams();
}

void KFLogitCRF::train(G4FeatSeqL &seqlist) {
  KFLogitReg::train(seqlist);
}

arr KFLogitCRF::query(G4FeatSeq &seq) {
  arr y = KFLogitReg::query(seq);
  return y;
}
// }}}
// PhiMachine stuff {{{
std::function<arr(arr)> PhiBias() {
  return [](arr x) -> arr { return x; };
}
std::function<arr(arr)> PhiLin() {
  return [](arr x) -> arr {
    arr phi;
    phi.append(1.);
    phi.append(x);
    return phi;
  };
}
std::function<arr(arr)> PhiPoly(double d) {
  // TODO finish this..
  return [](arr x) -> arr { return x; };
}
// }}}
