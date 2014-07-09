#include <extern/SWIFT/SWIFT.h>
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>
#include <Algo/gaussianProcess.h>
#include <Perception/videoEncoder.h>
#include "keyframer.h"

// TODO replace with appropriate tensor function
//arr my_sum(const arr &v, uint d) {
  //arr x, S;
  //uintA I;
  //x.referTo(v);
  //x.reshape(x.N);
  //S.resize(v.dim(d));
  //S.setZero();
  //for(uint i = 0; i < x.N; i++) {
    //v.getIndexTuple(I, i);
    //S(I(d)) += x(i);
  //}
  //return S;
//}

struct KeyFramer::sKeyFramer {
  ors::KinematicWorld *kw;
  G4Data *g4d;

  uint nbodies, ndofs;
  uintA dofs, cumdofs;
  StringA names;

  uint nframes;
  arr state;
  arr ood, dird, indd; // distance matrices

  sKeyFramer();
  ~sKeyFramer();
};

// ============================================================================
// sKeyFramer
//

KeyFramer::sKeyFramer::sKeyFramer(): kw(NULL), g4d(NULL) { }
KeyFramer::sKeyFramer::~sKeyFramer() { if(kw) delete kw; if(g4d) delete g4d; }

// ============================================================================
// KeyFramer
//

KeyFramer::KeyFramer() {
  s = new sKeyFramer();
}

KeyFramer::~KeyFramer() {
  delete s;
}

ors::KinematicWorld& KeyFramer::kw() {
  if(!s->kw) s->kw = new ors::KinematicWorld;
  return *s->kw;
}

G4Data& KeyFramer::g4d() {
  if(!s->g4d) s->g4d = new G4Data;
  return *s->g4d;
}

// updateOrs {{{
#define PREC_POS 1e-2
#define PREC_ORI 1e-1
void KeyFramer::updateOrs(uint f, bool show) {
  arr sensor_pos, sensor_quat;
  ors::Quaternion quat;
  ors::Vector x_vec(1, 0, 0),
              y_vec(0, 1, 0),
              z_vec(0, 0, 1);
  ors::Shape *sh;

  arr y, J, yVec, JVec;
  arr Phi, PhiJ, PhiJT;
  arr q, dq;
  arr y_target, W;

  uint n = kw().getJointStateDimension();
  W.setDiag(1e-4, n);
  for(;;) {
    Phi.resize(0);
    PhiJ.resize(0);
    kw().getJointState(q);

    for(const String &sensor: g4d().sensors()) {
      sh = kw().getShapeByName(STRING("sh:task:" << sensor));

      sensor_pos.referTo(g4d().query("pos", sensor, f));
      sensor_quat.referTo(g4d().query("quat", sensor, f));
      quat = ors::Quaternion(sensor_quat);

      kw().kinematicsPos(y, J, sh->body, &sh->rel.pos);
      Phi.append((y - sensor_pos) / PREC_POS);
      PhiJ.append(J / PREC_POS);
        
      kw().kinematicsVec(y, J, sh->body, &x_vec);
      y_target.setCarray((quat * x_vec).p(), 3);
      Phi.append((y - y_target) / PREC_ORI);
      PhiJ.append(J / PREC_ORI);

      kw().kinematicsVec(y, J, sh->body, &y_vec);
      y_target.setCarray((quat * y_vec).p(), 3);
      Phi.append((y - y_target) / PREC_ORI);
      PhiJ.append(J / PREC_ORI);

      kw().kinematicsVec(y, J, sh->body, &z_vec);
      y_target.setCarray((quat * z_vec).p(), 3);
      Phi.append((y - y_target) / PREC_ORI);
      PhiJ.append(J / PREC_ORI);
    }
    PhiJT = ~PhiJ;
    dq = inverse(PhiJT * PhiJ + W) * PhiJT * Phi;

    q -= dq;
    kw().setJointState(q);
    kw().calc_fwdPropagateShapeFrames();
    if(absMax(dq) < 1e-2)
      break;
  }
  // TODO remove computeProxies?
  /* kw().computeProxies(); */
  if(show) {
    kw().gl().text.clear() << "frame " << f << "/" << g4d().numFrames();
    kw().gl().update(NULL, true);
  }
}
// }}}

// computeVar {{{
void KeyFramer::computeVar(const StringA &types, uint wlen, bool force) {
  for(const String &type: types)
    computeVar(type, wlen, force);
}
// }}}
// computeVar {{{
void KeyFramer::computeVar(const String &type, uint wlen, bool force) {
  String typeVar = STRING(type << "Var");
  cout << " * computing " << typeVar << endl;
  if(!force && g4d().hasBAM(typeVar)) {
    cout << " * " << typeVar << " already computed (force = 0). Skipping." << endl;
    return;
  }

  arr x, y, win, m;

  x = g4d().query(type);
  y.resize(x.d0, x.d1);
  y.setZero();

  uint ff = wlen / 2;
  uint ft = g4d().numFrames() - ff;
  for(uint i = 0; i < x.d0; i++) {
    for(uint fi = ff; fi < ft; fi++) {
      uint wi = fi - ff;
      win.referToSubRange(x[i], wi, wi + wlen - 1);
      m = sum(win, 0) / (double)wlen;
      m = ~repmat(m, 1, wlen);
      y(i, fi) = sumOfSqr(win - m);
    }
  }
  y /= (double)wlen;

  g4d().appendBam(typeVar, y);
}
// }}}
// computeFilter {{{
void KeyFramer::computeFilter(const StringA &types, double alpha, bool force) {
  for(const String &type: types)
    computeFilter(type, alpha, force);
}
// }}}
// computeFilter {{{
void KeyFramer::computeFilter(const String &type, double alpha, bool force) {
  CHECK(alpha >= 0. && alpha <= 1., "Parameter alpha (" << alpha << ") must be 0 <= alpha <= 1.");
  String typeFilter = STRING(type << "Filter");
  cout << " * computing " << typeFilter << endl;
  if(!force && g4d().hasBAM(typeFilter)) {
    cout << " * " << typeFilter << " already computed (force = 0). Skipping." <<
      endl;
    return;
  }
  uint numS = g4d().sensors().N;
  uint numF = g4d().numFrames();
  uint numD = g4d().numDim(type);
  double beta = 1-alpha;

  arr x, y, es;
  x = g4d().query(type);
  y.resizeAs(x);
  y.setZero();

  for(uint i = 0; i < numS; i++) {
    for(uint d = 0; d < numD; d++) {
      es = (~x[i])[d];
      for(uint f = 1; f < numF; f++)
        es(f) = es(f) * alpha + es(f-1) * beta;
      for(uint f = 0; f < numF; f++)
        y(i, f, d) = es(f);
    }
  }

  g4d().appendBam(typeFilter, y);
}
// }}}
// computeSmooth {{{
void KeyFramer::computeSmooth(const StringA &types, double alpha, bool force) {
  for(const String &type: types)
    computeSmooth(type, alpha, force);
}
// }}}
// computeSmooth {{{
void KeyFramer::computeSmooth(const String &type, double alpha, bool force) {
  CHECK(alpha >= 0. && alpha <= 1., "Parameter alpha (" << alpha << ") must be 0 <= alpha <= 1.");
  String typeSmooth = STRING(type << "Smooth");
  cout << " * computing " << typeSmooth << endl;
  if(!force && g4d().hasBAM(typeSmooth)) {
    cout << " * " << typeSmooth << " already computed (force = 0). Skipping." <<
      endl;
    return;
  }
  uint numS = g4d().sensors().N;
  uint numF = g4d().numFrames();
  uint numD = g4d().numDim(type);
  double beta = 1-alpha;

  arr x, y, es;
  x = g4d().query(type);
  y.resizeAs(x);
  y.setZero();

  for(uint i = 0; i < numS; i++) {
    for(uint d = 0; d < numD; d++) {
      es = (~x[i])[d];
      for(uint f = numF-2; f-- > 0;)
        es(f) = es(f) * alpha + es(f+1) * beta;
      for(uint f = 1; f < numF; f++)
        es(f) = es(f) * alpha + es(f-1) * beta;
      for(uint f = 0; f < numF; f++)
        y(i, f, d) = es(f);
    }
  }

  g4d().appendBam(typeSmooth, y);
}
// }}}
// computeSpline {{{
void KeyFramer::computeSpline(const StringA &types, double lambda, bool force) {
  for(const String &type: types)
    computeSpline(type, lambda, force);
}
// }}}
// computeSpline {{{
#include<Algo/spline.h>
void KeyFramer::computeSpline(const String &type, double lambda, bool force) {
  CHECK(lambda >= 0., "Parameter lambda (" << lambda << ") must be non-negative.");
  String typeSpline = STRING(type << "Spline");
  cout << " * computing " << typeSpline << endl;
  if(!force && g4d().hasBAM(typeSpline)) {
    cout << " * " << typeSpline << " already computed (force = 0). Skipping." <<
      endl;
    return;
  }
  uint numS = g4d().sensors().N;
  uint numF = g4d().numFrames();
  uint numD = g4d().numDim(type);

  arr x, y, tx, ty;

  x = g4d().query(type);
  y.resizeAs(x);
  y.setZero();

  for(uint i = 0; i < numS; i++) {
    for(uint d = 0; d < numD; d++) {
      cout << " * computing " << typeSpline << " for sensor " << i << ", dim " << d << endl;
      tx = (~x[i])[d];
      MT::Spline spl(numF, tx);
      ty = spl.smooth(lambda);
      for(uint f = 0; f < numF; f++)
        y(i, f, d) = ty(f);
    }
  }

  g4d().appendBam(typeSpline, y);
}
// }}}
// computeSpeed {{{
void KeyFramer::computeSpeed(const StringA &types, bool force) {
  for(const String &type: types)
    computeSpeed(type, force);
}
// }}}
// computeSpeed {{{
void KeyFramer::computeSpeed(const String &type, bool force) {
  String typeSpeed = STRING(type << "Speed");
  cout << " * computing " << typeSpeed << endl;
  if(!force && g4d().hasBAM(typeSpeed)) {
    cout << " * " << typeSpeed << " already computed (force = 0). Skipping." << endl;
    return;
  }

  arr x, y;

  x = g4d().query(type);
  y.resize(x.d0, x.d1);
  y.setZero();

  uint numS = g4d().sensors().N;
  uint numF = g4d().numFrames();
  for(uint i = 0; i < numS; i++) {
    for(uint f = 1; f < numF; f++)
      // speed in cm/s (measurements in 100cm, and at 120/s)
      y(i, f) = 12000. * sqrt(sumOfSqr(x[i][f] - x[i][f-1]));
  }

  g4d().appendBam(typeSpeed, y);
}
// }}}
// computeGP {{{
void KeyFramer::computeGP(const StringA &types, bool force) {
  for(const String &type: types)
    computeGP(type, force);
}
// }}}
// computeGP {{{
void KeyFramer::computeGP(const String &type, bool force) {
  String typeGP = STRING(type << "GP");
  cout << " * computing " << typeGP << endl;
  if(!force && g4d().hasBAM(typeGP)) {
    cout << " * " << typeGP << " already computed (force = 0). Skipping." << endl;
    return;
  }
  uint numS = g4d().sensors().N;
  uint numF = g4d().numFrames();
  uint numD = g4d().numDim(type);

  arr x, y;
  x = g4d().query(type);
  y.resize(numS, numF, numD);
  y.setZero();

  GaussianProcess gp;
  GaussKernelParams gpp(100., 20., .1);
  gp.obsVar = .05;
  gp.setKernel(GaussKernel, &gpp);
  arr mm, ss, tt;
  tt = linspace(0, numF-1, numF-1);

  for(uint i = 0; i < numS; i++) {
    for(uint d = 0; d < numD; d++) {
      cout << " * computing " << typeGP << " for sensor " << i << ", dim " << d << endl;
      gp.recompute(tt, (~x[i])[d]);
      gp.evaluate(tt, mm, ss);
      for(uint f = 0; f < numF; f++)
        y(i, f, d) = mm(f);
    }
  }

  g4d().appendBam(typeGP, y);
}
// }}}
// computeDPos {{{
void KeyFramer::computeDPos(const String &b, bool force) {
  String typeDPos;
  typeDPos << b << "_dPos";
  cout << " * computing " << typeDPos << endl;
  if(!force && g4d().hasBAM(typeDPos)) {
    cout << " * " << typeDPos << " already computed (force = 0). Skipping." << endl;
    return;
  }
  uint numS = g4d().sensors().N;
  uint numF = g4d().numFrames();
  uint numD = g4d().numDim("pos");

  arr y(numS, numF, numD);
  y.setZero();

  arr posX, quatX, posY;
  posX = g4d().query("pos", b);
  posY = g4d().query("pos");
  quatX = g4d().query("quat", b);
  ors::Vector v1, v2, v, A;
  ors::Quaternion q1;
  for(uint j = 0; j < numS; j++) {
    for(uint f = 0; f < numF; f++) {
      v1.set(posX[f].p);
      v2.set(posY[j][f].p);
      q1.set(quatX[f].p);
      if(f == 0)
        A = q1 * (v2 - v1);
      v = q1 * (v2 - v1) - A;
      y[j][f]() = { v.x, v.y, v.z };
    }
  }

  g4d().appendBam(typeDPos, y);
}
// }}}
// computeDQuat {{{
void KeyFramer::computeDQuat(const String &b, bool force) {
  String typeDQuat;
  typeDQuat << b << "_dQuat";
  cout << " * computing " << typeDQuat << endl;
  if(!force && g4d().hasBAM(typeDQuat)) {
    cout << " * " << typeDQuat << " already computed (force = 0). Skipping." << endl;
    return;
  }
  uint numS = g4d().sensors().N;
  uint numF = g4d().numFrames();
  uint numD = g4d().numDim("quat");

  arr y(numS, numF, numD);
  y.setZero();

  arr quatX, quatY;
  quatX = g4d().query("quat", b);
  quatY = g4d().query("quat");
  ors::Quaternion q1, q2, q, A;
  for(uint j = 0; j < numS; j++) {
    for(uint f = 0; f < numF; f++) {
      q1.set(quatX[f].p);
      q2.set(quatY[j][f].p);
      if(f == 0)
        A = q1 / q2;
      q = q1 / ( A * q2 );
      y[j][f]() = { q.w, q.x, q.y, q.z };
    }
  }

  g4d().appendBam(typeDQuat, y);
}
// }}}

// computeDist {{{
void KeyFramer::computeDist(uint f) {
  kw().swift();
  uint numA = g4d().digits().N;
  uint numO = g4d().objects().N;

  updateOrs(f, false);
  // direct agent-object and object-object distances
  uint idsa, idsb, ida, idb;
  double d;
  bool aa, ab;
  ors::Shape *sa, *sb;
  ors::Body *ba, *bb;

  // MY SWIFT STUFF FOR DISTANCES
  int np, *oids;
  SWIFT_Real *dists;
  kw().swift().scene->Query_Exact_Distance(false, SWIFT_INFINITY, np, &oids, &dists);
  //CHECK(np == (int)numO*(2*numA+numO-1)/2, STRING("number of distances (" << np << ") not right (" << numO*(2*numA+numO-1)/2 << ")."));
  for(int i = 0; i < np; i++) {
    idsa = oids[i<<1];
    idsb = oids[(i<<1)+1];
    d = dists[i]>=0? dists[i]: 0;

    sa = kw().shapes(idsa);
    sb = kw().shapes(idsb);
    ba = sa->body;
    bb = sb->body;
    aa = g4d().digits().contains(ba->name);
    ab = g4d().digits().contains(bb->name);
    ida = aa? g4d().digits().findValue(ba->name): g4d().objects().findValue(ba->name);
    idb = ab? g4d().digits().findValue(bb->name): g4d().objects().findValue(bb->name);
    if(aa && !ab && d < s->dird(f, ida, idb)) s->dird(f, ida, idb) = d;
    if(ab && !aa && d < s->dird(f, idb, ida)) s->dird(f, idb, ida) = d;
    if(!aa && !ab && d < s->ood(f, ida, idb)) s->ood(f, ida, idb) = s->ood(f, idb, ida) = d;
  }
  for(uint o = 0; o < numO; o++)
    s->ood(f, o, o) = 0;
  // object-object indirect distances
  arr tmpd;
  for(bool done = false; !done; ) {
    tmpd = s->ood[f];
    for(uint o = 0; o < numO; o++)
      for(uint o2 = 0; o2 < numO; o2++)
        tmpd(o, o2) = (s->ood[f][o] + s->ood[f][o2]).min();
    if(s->ood[f] == tmpd)
      done = true;
    s->ood[f]() = tmpd;
  }
  // agent-object indirect distances
  for(uint a = 0; a < numA; a++)
    for(uint o = 0; o < numO; o++)
      s->indd(f, a, o) = (s->dird[f][a] + s->ood[f][o]).min();
}
// }}}
// computeDist {{{
void KeyFramer::computeDist() {
  uint numF = g4d().numFrames();
  uint numA = g4d().digits().N;
  uint numO = g4d().objects().N;

  s->ood.resize(numF, numO, numO); s->ood.setZero(100);
  s->dird.resize(numF, numA, numO); s->dird.setZero(100); // can remove this?
  s->indd.resize(numF, numA, numO); s->indd.setZero(100);

  // swift preparations
  cout << " * deactivating proxies within bodies:" << endl;
  for(ors::Body *b: kw().bodies) {
    cout << " *** " << b->name << endl;
    kw().swift().deactivate(b->shapes);
  }
  BodyL agents;
  cout << " * deactivating proxies between agents :" << endl;
  for(const String &a: g4d().digits()) {
    cout << " *** " << a << endl;
    agents.append(kw().getBodyByName(a));
  }
  kw().swift().deactivate(agents);
  //kw().swift().setCutoff(100.);
  for(uint f = 0; f < numF; f++)
    computeDist(f);
}
// }}}
// testDist {{{
void KeyFramer::testDist(KeyValueGraph &kvg, const String &a, const String &o) {
  KeyValueGraph *plot;
  arr dir_d, ind_d;

  computeDist();

  uint numF = g4d().numFrames();
  uint ia = g4d().digits().findValue(a);
  uint io = g4d().objects().findValue(o);
  dir_d.resize(numF);
  ind_d.resize(numF);
  for(uint f = 0; f < numF; f++) {
    dir_d(f) = s->dird(f, ia, io);
    ind_d(f) = s->indd(f, ia, io);
  }
  kvg.append("data", "dird", new arr(dir_d));
  kvg.append("data", "indd", new arr(ind_d));
  plot = new KeyValueGraph();
  plot->append("title", new String(STRING("Dist " << a << "-" << o)));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("dird"));
  plot->append("data", new String("indd"));
  kvg.append("plot", plot);
}
// }}}

// getVitSeq {{{
void KeyFramer::getVitSeq(arr &vit, const String &a, const String &o) {
  KeyValueGraph data_kvg;
  EM_z_with_c(data_kvg, a, o);
  vit = *data_kvg.getValue<arr>("vit");
}
// }}}
// getVitSeq {{{
void KeyFramer::getVitSeq(arr &vit, const StringA &as, const String &o) {
  uint N = as.N;
  uint T = g4d().numFrames();

  vit.resize(N, T);
  for(uint i = 0; i < N; i++)
    getVitSeq(vit[i](), as(i), o);
}
// }}}
// vitLogicMachine {{{
void KeyFramer::vitLogicMachine(KeyValueGraph &kvg, arr &vit2, const arr &vit) {
  uint N = vit.d0;
  uint T = vit.d1;

  vit2.resize(vit.d1);
  for(uint t = 0; t < T; t++) {
    vit2(t) = 0;
    for(uint a = 0; a < N; a++) {
      if(vit(a, t)) {
        vit2(t) = 1;
        break;
      }
    }
  }
}
// }}}
// getCtrlSeq_old {{{
void KeyFramer::getCtrlSeq_old(kvgL &ctrls, const String &a, const String &o) {
  KeyValueGraph data_kvg;
  EM_z_with_c(data_kvg, a, o);
  arr *vit = data_kvg.getValue<arr>("vit");

  KeyValueGraph *kf, *feats;
  bool kf_flag = false;
  for(uint f = 0; f < vit->d0; f++) {
    if(!kf_flag && vit->elem(f)) { // inset
      feats = new KeyValueGraph();
      objFeatures(*feats, o, f);
      kf = new KeyValueGraph();
      kf->append("inset", feats);
      kf_flag = true;
    }
    else if(kf_flag && !vit->elem(f)) { // offset
      feats = new KeyValueGraph();
      objFeatures(*feats, o, f);
      kf->append("offset", feats);
      ctrls.append(new KeyValueGraph(*kf));
      kf_flag = false;
    }
  }
}
// }}}
// getCtrlSeq {{{
void KeyFramer::getCtrlSeq(kvgL &ctrls, const String &a, const String &o) {
  KeyValueGraph data_kvg;
  EM_z_with_c(data_kvg, a, o);
  arr *vit = data_kvg.getValue<arr>("vit");

  KeyValueGraph *kf;
  bool kf_flag = false;
  for(uint f = 0; f < vit->d0; f++) {
    if(!kf_flag && vit->elem(f)) { // inset
      kf = new KeyValueGraph();
      kf->append("fnum", new double(f));
      kf->append("type", "on");
      // TODO agents
      // TODO objects
      ctrls.append(kf);
      kf_flag = true;
    }
    else if(kf_flag && !vit->elem(f)) { // offset
      kf = new KeyValueGraph();
      kf->append("fnum", new double(f));
      kf->append("type", "off");
      // TODO agents
      // TODO objects
      ctrls.append(kf);
      kf_flag = false;
    }
  }
}
// }}}
// getDeltaSeq {{{
void KeyFramer::getDeltaSeq(kvgL &deltas, kvgL ctrls) {
  KeyValueGraph *inset, *offset, *delta;
  arr *posi, *poso;
  uint fi, fo;

  for(auto ctrl: ctrls) {
    inset = ctrl->getValue<KeyValueGraph>("inset");
    offset = ctrl->getValue<KeyValueGraph>("offset");

    fi = *inset->getValue<double>("fnum");
    posi = inset->getValue<arr>("f_pos");

    fo = *offset->getValue<double>("fnum");
    poso = offset->getValue<arr>("f_pos");

    delta = new KeyValueGraph();
    delta->append("fi", new double(fi));
    delta->append("fo", new double(fo));
    delta->append("f_dpos", new arr(*poso - *posi));
    deltas.append(delta);
  }
}
// }}}
// getDeltaCluster {{{
void KeyFramer::getDeltaCluster(kvgL &deltas, kvgL ctrls) {
  KeyValueGraph *inset, *offset, *delta;
  arr *posi, *poso;
  uint fi, fo;

  cout << "#ctrls: " << ctrls.N << endl;
  for(auto ctrl: ctrls) {
    inset = ctrl->getValue<KeyValueGraph>("inset");
    offset = ctrl->getValue<KeyValueGraph>("offset");

    fi = *inset->getValue<double>("fnum");
    posi = inset->getValue<arr>("f_pos");

    fo = *offset->getValue<double>("fnum");
    poso = offset->getValue<arr>("f_pos");

    delta = new KeyValueGraph();
    delta->append("fi", new double(fi));
    delta->append("fo", new double(fo));
    delta->append("f_dpos", new arr(*poso - *posi));
    deltas.append(delta);
  }
}
// }}}

//#define EM_CORR_ORIG
//#define EM_CORR_NEW
//#define EM_NO_CORR
//#define EM_MOV

// EM NO CORR {{{
#ifdef EM_NO_CORR
#define update_mu_pAVar
//#define update_sigma_pAVar
#define update_mu_qAVar
//#define update_sigma_qAVar
#define update_p_r_z0
//#define update_p_r_z1
//#define update_mu_dpVar
#define update_sigma_dpVar
//#define update_mu_dqVar
#define update_sigma_dqVar
void KeyFramer::EM(KeyValueGraph &kvg, const String &bA, const String &bB, uint wlen) {
  // Computing other BAMS {{{
  cout << " * computing posVar" << endl;
  computeVar(STRING("pos"), wlen);
  cout << " * computing quatVar" << endl;
  computeVar(STRING("quat"), wlen);

  String bA_dPos = STRING(bA << "_dPos");
  String bA_dQuat = STRING(bA << "_dQuat");
  String bA_dPosVar = STRING(bA_dPos << "Var");
  String bA_dPosQuat = STRING(bA_dQuat << "Var");

  cout << " * computing " << bA_dPos << endl;
  computeDPos(bA);
  cout << " * computing " << bA_dQuat << endl;
  computeDQuat(bA);
  cout << " * computing " << bA_dPosVar << endl;
  computeVar(bA_dPos, wlen);
  cout << " * computing " << bA_dQuatVar << endl;
  computeVar(bA_dQuat, wlen);
  // }}}
  // Observations {{{
  //String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  //String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));
  // TODO CHECK IF THIS WORKS
  arr pAVar = g4d().query("posVar", bA);
  arr qAVar = g4d().query("quatVar", bA);
  arr dpVar = g4d().query(bA_posDeltaVar, bB);
  arr dqVar = g4d().query(bA_quatDeltaVar, bB);
  // }}}
  //Parameters & other {{{
  double mu_H, sigma_S, sigma_L;
  arr pi, A;
  arr p_r;
  arr mu_pAVar, mu_qAVar, mu_dpVar, mu_dqVar;
  arr sigma_pAVar, sigma_qAVar, sigma_dpVar, sigma_dqVar;

  arr rho_z, rho_z_pAVar, rho_z_qAVar, rho_z_dVar, rho_r;
  arr qz, qzz, qzr;
  arr a, b;
  arr pz, pzz, pzmpAVar, pzspAVar, pzmqAVar, pzsqAVar, pzr;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);

  mu_H = 1;
  sigma_S = .05;
  sigma_L = .1;
  mu_pAVar = { 0, .1 };
  sigma_pAVar = { sigma_L, sigma_L };
  mu_qAVar = { 0, .1 };
  sigma_qAVar = { sigma_L, sigma_L };

  p_r = { .1, 0,
          .9, 1 };
  p_r.reshape(2, 2);
  mu_dpVar = { 0, 0 };
  sigma_dpVar = { sigma_L, sigma_S };
  mu_dqVar = { 0, 0 };
  sigma_dqVar = { sigma_L, sigma_S };

  double T, K, P, Q, R;
  T = pAVar.d0;
  K = 2;
  R = 2;

  rho_z.resize(T, K);
  rho_z_pAVar.resize(T, K);
  rho_z_qAVar.resize(T, K);
  rho_z_dVar.resize(T, K);
  rho_r.resize(T, R);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  qzr.resize(T, K, R);
  a.resize(T, K);
  b.resize(T, K);
  pz.resize(K);
  pzz.resize(K, K);
  pzmpAVar.resize(K);
  pzspAVar.resize(K);
  pzmqAVar.resize(K);
  pzsqAVar.resize(K);
  pzr.resize(K, R);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "mu_pAVar: " << mu_pAVar << endl;
    cout << "sigma_pAVar: " << sigma_pAVar << endl;
    cout << "mu_qAVar: " << mu_qAVar << endl;
    cout << "sigma_qAVar: " << sigma_qAVar << endl;
    cout << "p_r: " << p_r << endl;
    cout << "mu_dpVar: " << mu_dpVar << endl;
    cout << "sigma_dpVar: " << sigma_dpVar << endl;
    cout << "mu_dqVar: " << mu_dqVar << endl;
    cout << "sigma_dqVar: " << sigma_dqVar << endl;
    cout << endl << "---------------------------" << endl;
    // }}}
    // COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++) {
        rho_z_pAVar(t, k) = ::exp(
            -.5 * MT::sqr(pAVar(t) - mu_pAVar(k)) / MT::sqr(sigma_pAVar(k))
        );
        rho_z_qAVar(t, k) = ::exp(
            -.5 * MT::sqr(qAVar(t) - mu_qAVar(k)) / MT::sqr(sigma_qAVar(k))
        );
      }
      for(uint r = 0; r < R; r++) {
        rho_r(t, r) = ::exp(
            -.5 * MT::sqr(dpVar(t) - mu_dpVar(r)) / MT::sqr(sigma_dpVar(r))
            -.5 * MT::sqr(dqVar(t) - mu_dqVar(r)) / MT::sqr(sigma_dqVar(r))
        );
        //double tmp = 0;
        //for(uint tt = MT::MAX(0, t - wlen/2); tt < MT::MIN(T, t + wlen/2); tt++)
          //tmp += MT::sqr( dp(t, 0) - mu_r_p(r, 0)) / MT::sqr(sigma_r_p(r, 0))
                //+ MT::sqr( dp(t, 1) - mu_r_p(r, 1)) / MT::sqr(sigma_r_p(r, 1))
                //+ MT::sqr( dp(t, 2) - mu_r_p(r, 2)) / MT::sqr(sigma_r_p(r, 2));
                //+ MT::sqr( dq(t, 0) - mu_r_q(r, 0)) / MT::sqr(sigma_r_q(r, 0))
                //+ MT::sqr( dq(t, 1) - mu_r_q(r, 1)) / MT::sqr(sigma_r_q(r, 1))
                //+ MT::sqr( dq(t, 2) - mu_r_q(r, 2)) / MT::sqr(sigma_r_q(r, 2))
                //+ MT::sqr( dq(t, 3) - mu_r_q(r, 3)) / MT::sqr(sigma_r_q(r, 3))
        //rho_r(t, r) = ::exp( -.5 * tmp);
      }
      rho_z_dVar[t]() = ~rho_r[t] * p_r;
    }
    rho_z = rho_z_pAVar % rho_z_qAVar % rho_z_dVar;
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  A * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~A * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      // TODO CHECK THIS
      qzr[t]() = ~p_r % ( (a[t] % b[t] % rho_z_pAVar[t] % rho_z_qAVar[t]) ^ rho_r[t] );
      qz[t]() = a[t] % b[t] % rho_z[t];
      normalizeDist(qzr[t]());
      normalizeDist(qz[t]());
    }
    for(uint t = 0; t < T-1; t++) {
      qzz[t]() = A % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
      normalizeDist(qzz[t]());
    }
    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    pzmpAVar.setZero();
    pzspAVar.setZero();
    pzmqAVar.setZero();
    pzsqAVar.setZero();
    pzr.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      pzmpAVar += pAVar(t) * qz[t];
      pzspAVar += sqr(pAVar(t) - mu_pAVar) % qz[t];
      pzmqAVar += qAVar(t) * qz[t];
      pzsqAVar += sqr(qAVar(t) - mu_qAVar) % qz[t];
      pzr += qzr[t];
      if(t < T-1)
        pzz += qzz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);

#ifdef update_mu_pAVar
    mu_pAVar = pzmpAVar / pz;
#endif
#ifdef update_sigma_pAVar
    sigma_pAVar = sqrt(pzspAVar / pz);
#endif
#ifdef update_mu_qAVar
    mu_qAVar = pzmqAVar / pz;
#endif
#ifdef update_sigma_qAVar
    sigma_qAVar = sqrt(pzsqAVar / pz);
#endif
#ifdef update_p_r_z0
    for(int r = 0; r < R; r++)
      p_r(r, 0) = pzr(0, r) / pz(0);
#endif
#ifdef update_p_r_z1
    for(int r = 0; r < R; r++)
      p_r(r, 1) = pzr(1, r) / pz(1);
#endif
#ifdef update_sigma_dp
    //sigma_dp = sqrt(pzsp / pz);
#endif
#ifdef update_sigma_dq
    //sigma_q = sqrt(pzsq / pz);
#endif
#ifdef update_mu_dp
    //mu_p = pzmp / pz;
#endif
#ifdef update_mu_dq
    //mu_q = pzmq / pz;
#endif
    // }}}

    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  double m;
  int mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test

    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      m = temp(k, 0);
      mi = 0;
      for(uint kk = 1; kk < K; kk++) {
        if(m < temp(k, kk)) {
          m = temp(k, kk);
          mi = kk;
        }
      }
      wz(t, k) += m;
      wzind(t, k) = mi;
    }
  }

  arr vitz(T);
  vitz(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vitz(t-1) = wzind(t, vitz(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vitz", new arr(vitz));
  kvg.append("data", "pAVar", new arr(pAVar));
  kvg.append("data", "qAVar", new arr(qAVar));
  kvg.append("data", "dpVar", new arr(dpVar));
  kvg.append("data", "dqVar", new arr(dqVar));
  
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Viterbi + Observations"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-1.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("pAVar"));
  plot->append("data", new String("qAVar"));
  plot->append("data", new String("dpVar"));
  plot->append("data", new String("dqVar"));
  plot->append("data", new String("vitz"));
  kvg.append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("Motion A"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-1.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("pAVar"));
  plot->append("data", new String("qAVar"));
  plot->append("data", new String("vitmA"));
  // }}}
}
#endif // EM_NO_CORR
// }}}
 // EM_CORR_NEW {{{
#ifdef EM_CORR_NEW
#define update_mu_c
//#define update_sigma_c
#define update_p_y_z0
//#define update_p_y_z1
#define update_mu_pVar
//#define update_sigma_pVar
#define update_mu_qVar
//#define update_sigma_qVar
//#define update_mu_dp // TODO
//#define update_sigma_dp // TODO
//#define update_mu_dq // TODO
//#define update_sigma_dq // TODO
void KeyFramer::EM(KeyValueGraph &kvg, const String &bA, const String &bB, uint wlen) {
  // Computing other BAMS {{{
  cout << " * computing posVar" << endl;
  computeVar(STRING("pos"), wlen);
  cout << " * computing quatVar" << endl;
  computeVar(STRING("quat"), wlen);

  String bA_dPos = STRING(bA << "_dPos");
  String bA_dQuat = STRING(bA << "_dQuat");
  String bA_dPosDelta = STRING(bA_dPos << "Delta");
  String bA_dQuatDelta = STRING(bA_dQuat << "Delta");
  // }}}
  // Observations {{{
  String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));

  // TODO compute this using the new data structure..
  arr c = getCorrPCA(bp1, bp2, wlen, 1).flatten();
  arr pAVar = g4d().query("posVar", bA);
  arr qAVar = g4d().query("quatVar", bA);
  arr pBVar = g4d().query("posVar", bB);
  arr qBVar = g4d().query("quatVar", bB);
  arr dp = g4d().query(bA_posDelta, bB);
  arr dq = g4d().query(bA_quatDelta, bB);

  arr pVarMean = (pAVar + pBVar) / 2.;
  arr qVarMean = (qAVar + qBVar) / 2.;

  // }}}
  // Parameters & other {{{
  uint T, K, J;
  double sigma_small, sigma_big;
  arr pi, A;
  arr mu_c, sigma_c;
  arr p_y;
  arr mu_pVar, sigma_pVar;
  arr mu_qVar, sigma_qVar;
  arr mu_dp, sigma_dp;
  arr mu_dq, sigma_dq;
  arr rho_z_c, rho_z_cpq, rho_z_cypq;
  arr rho_y_pVar, rho_y_qVar, rho_y_dp, rho_y_dq, rho_y_pqd;
  arr qz, qzz, qzy, qy;
  arr a, b;
  arr pz, pzz, pzmc, pzsc, pzy, py, pympVar, pyspVar, pymqVar, pysqVar;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);
  mu_c = { 0, 1 };
  sigma_c = { 1, .2 };
  p_y = { .3, 0,
        .7, 1 };
  p_y.reshape(2, 2);
  sigma_small = .3; sigma_big = .7;
  mu_pVar = {0, 0}; sigma_pVar = { sigma_big, sigma_small };
  mu_qVar = {0, 0}; sigma_qVar = { sigma_big, sigma_small };
  mu_dp = zeros(2, 3);
  sigma_dp = ones(2, 3);
  sigma_dp[0]() *= sigma_big;
  sigma_dp[1]() *= sigma_small;
  mu_dq = zeros(2, 4);
  mu_dq(0, 0) = mu_dq(1, 0) = 1;
  sigma_dq = ones(2, 4);
  sigma_dq[0]() *= sigma_big;
  sigma_dq[1]() *= sigma_small;

  T = c.d0;
  K = 2;
  J = 2;

  rho_z_c.resize(T, K);
  rho_z_cpq.resize(T, K);
  rho_z_cypq.resize(T, J, K);
  rho_y_pVar.resize(T, J);
  rho_y_qVar.resize(T, J);
  rho_y_dp.resize(T, J);
  rho_y_dq.resize(T, J);
  rho_y_pqd.resize(T, J);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  qzy.resize(T, K, J);
  qy.resize(T, J);

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  pzmc.resize(K);
  pzsc.resize(K);
  pzy.resize(K, J);
  py.resize(J);
  pympVar.resize(J);
  pyspVar.resize(J);
  pymqVar.resize(J);
  pysqVar.resize(J);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "mu_c: " << mu_c << endl;
    cout << "sigma_c: " << sigma_c << endl;
    cout << "p_y " << p_y << endl;
    cout << "mu_pVar: " << mu_pVar << endl;
    cout << "sigma_pVar: " << sigma_pVar << endl;
    cout << "mu_qVar: " << mu_qVar << endl;
    cout << "sigma_qVar: " << sigma_qVar << endl;
    cout << "mu_dp: " << mu_dp << endl;
    cout << "sigma_dp: " << sigma_dp << endl;
    cout << "mu_dq: " << mu_dq << endl;
    cout << "sigma_dq: " << sigma_dq << endl;
    // }}}
    // COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint j = 0; j < J; j++) {
        rho_y_pVar(t, j) = ::exp(
            -.5 * MT::sqr(pAVar(t) - mu_pVar(j)) / MT::sqr(sigma_pVar(j))
            -.5 * MT::sqr(pBVar(t) - mu_pVar(j)) / MT::sqr(sigma_pVar(j))
          );
        rho_y_qVar(t, j) = ::exp(
            -.5 * MT::sqr(qAVar(t) - mu_qVar(j)) / MT::sqr(sigma_qVar(j))
            -.5 * MT::sqr(qBVar(t) - mu_qVar(j)) / MT::sqr(sigma_qVar(j))
          );
        double tmp_dp, tmp_dq;
        tmp_dp = tmp_dq = 0;
        for(uint tt = MT::MAX(0, t - wlen/2); tt < MT::MIN(T, t + wlen/2); tt++) {
          tmp_dp += MT::sqr( dp(t, 0) - mu_dp(j, 0)) / MT::sqr(sigma_dp(j, 0))
                  + MT::sqr( dp(t, 1) - mu_dp(j, 1)) / MT::sqr(sigma_dp(j, 1))
                  + MT::sqr( dp(t, 2) - mu_dp(j, 2)) / MT::sqr(sigma_dp(j, 2));
          tmp_dq += MT::sqr( dq(t, 0) - mu_dq(j, 0)) / MT::sqr(sigma_dq(j, 0))
                  + MT::sqr( dq(t, 1) - mu_dq(j, 1)) / MT::sqr(sigma_dq(j, 1))
                  + MT::sqr( dq(t, 2) - mu_dq(j, 2)) / MT::sqr(sigma_dq(j, 2))
                  + MT::sqr( dq(t, 3) - mu_dq(j, 3)) / MT::sqr(sigma_dq(j, 3));
        }
        rho_y_dp(t, j) = ::exp( -.5 * tmp_dp);
        rho_y_dq(t, j) = ::exp( -.5 * tmp_dq);
      }
    }
    rho_y_pqd = rho_y_pVar % rho_y_qVar % rho_y_dp % rho_y_dq;
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++)
        rho_z_c(t, k) = ::exp(
            -.5 * MT::sqr(c(t) - mu_c(k)) / (sigma_c(k) * sigma_c(k))
            );
      rho_z_cpq[t]() = rho_z_c[t] % (~p_y * rho_y_pqd[t]);
      rho_z_cypq[t]() = p_y % (rho_y_pqd[t] ^ rho_z_c[t]);
    }
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  A * (rho_z_cpq[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~A * (rho_z_cpq[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z_cpq[t] % b[t];
      qzy[t]() = repmat(a[t] % b[t], 1, J) % ~rho_z_cypq[t];
      //qy[t]() = my_sum(qzy[t], 1);
      tensorMarginal(qy[y](), qzy[t], TUP(1));
      normalizeDist(qz[t]());
      normalizeDist(qzy[t]());
      normalizeDist(qy[t]());
    }
    for(uint t = 0; t < T-1; t++) {
      qzz[t]() = A % ( (rho_z_cpq[t+1] % b[t+1]) ^ (a[t] % rho_z_cpq[t]) );
      normalizeDist(qzz[t]());
    }
    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    pzmc.setZero();
    pzsc.setZero();
    pzy.setZero();
    py.setZero();
    pympVar.setZero();
    pyspVar.setZero();
    pymqVar.setZero();
    pysqVar.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      pzmc += c(t) * qz[t];
      pzsc += sqr(c(t) - mu_c) % qz[t];
      pzy += qzy[t];
      py += qy[t];
      pympVar += pVarMean(t) * qy[t];
      pyspVar += sqr(pVarMean(t) - mu_pVar) % qy[t];
      pymqVar += qVarMean(t) * qy[t];
      pysqVar += sqr(qVarMean(t) - mu_qVar) % qy[t];
      if(t < T-1)
        pzz += qzz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);

#ifdef update_mu_c
    mu_c = pzmc / pz;
#endif
#ifdef update_sigma_c
    sigma_c = sqrt(pzsc / pz);
#endif
#ifdef update_p_y_z0
    for(int j = 0; j < J; j++)
      p_y(j, 0) = pzy(0, j) / pz(0);
#endif
#ifdef update_p_y_z1
    for(int j = 0; j < J; j++)
      p_y(j, 1) = pzy(1, j) / pz(1);
#endif
#ifdef update_mu_pVar
    mu_pVar = pympVar / py;
#endif
#ifdef update_sigma_pVar
    sigma_pVar = sqrt(pyspVar / py);
#endif
#ifdef update_mu_qVar
    mu_qVar = pymqVar / py;
#endif
#ifdef update_sigma_qVar
    sigma_qVar = sqrt(pysqVar / py);
#endif
#ifdef update_mu_dp
    // TODO
#endif
#ifdef update_sigma_dp
    // TODO
#endif
#ifdef update_mu_dq
    // TODO
#endif
#ifdef update_sigma_dq
    // TODO
#endif
    // }}}

    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  double m;
  int mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z_cpq[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test

    wz[t]() = log(rho_z_cpq[t]);
    for(uint k = 0; k < K; k++) {
      m = temp(k, 0);
      mi = 0;
      for(uint kk = 1; kk < K; kk++) {
        if(m < temp(k, kk)) {
          m = temp(k, kk);
          mi = kk;
        }
      }
      wz(t, k) += m;
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "c", new arr(c));
  kvg.append("data", "pAVar", new arr(pAVar));
  kvg.append("data", "pBVar", new arr(pBVar));
  kvg.append("data", "qAVar", new arr(qAVar));
  kvg.append("data", "qBVar", new arr(qBVar));
  kvg.append("data", "dp", new arr(dp));
  kvg.append("data", "dq", new arr(dq));
  
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Viterbi + Observations"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-1.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("c"));
  plot->append("data", new String("pAVar"));
  plot->append("data", new String("pBVar"));
  plot->append("data", new String("qAVar"));
  plot->append("data", new String("qBVar"));
  plot->append("data", new String("vit"));
  kvg.append("plot", plot);
  // }}}
}
#endif // EM_CORR_NEW
// }}}
// EM CORR_ORIG {{{
#ifdef EM_CORR_ORIG
#define update_mu_c
//#define update_sigma_c
#define update_p_y_0
//#define update_p_y_1
#define update_mu_dpVar
//#define update_sigma_dpVar
#define update_mu_dqVar
//#define update_sigma_dqVar
void KeyFramer::EM(KeyValueGraph &kvg, const String &b1, const String &b2, uint wlen) {
  // Observations {{{
  String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));

  arr c = getCorrPCA(bp1, bp2, wlen, 1).flatten();
  arr dpVar = getPosVar(b1, b2, wlen);
  arr dqVar = getQuatVar(bo1, bo2, wlen);
  // }}}
  // Parameters & other {{{
  uint T, K, J;
  double sigma_small, sigma_big;
  arr pi, A;
  arr mu_c, sigma_c;
  arr p_y;
  arr mu_dpVar, sigma_dpVar;
  arr mu_dqVar, sigma_dqVar;
  arr rho_z_c, rho_z_cpq, rho_z_cypq, rho_y_p, rho_y_q, rho_y_d;
  arr rho_y_pqd ;
  arr qz, qzz, qzy, qy;
  arr a, b;
  arr pz, pzz, pzmc, pzsc, pzy, py, pymdpVar, pysdpVar, pymdqVar, pysdqVar;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);
  mu_c = { 0, 1 };
  sigma_c = { 1, .2 };
  p_y = { .3, .1,
          .7, .9 };
  p_y.reshape(2, 2);
  sigma_small = .3; sigma_big = .7;
  mu_dpVar = {0, 0}; sigma_dpVar = { sigma_big, sigma_small };
  mu_dqVar = {0, 0}; sigma_dqVar = { sigma_big, sigma_small };

  T = c.d0;
  K = 2;
  J = 2;

  rho_z_c.resize(T, K);
  rho_z_cpq.resize(T, K);
  rho_z_cypq.resize(T, J, K);
  rho_y_p.resize(T, J);
  rho_y_q.resize(T, J);
  rho_y_d.resize(T, J);
  rho_y_pqd.resize(T, J);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  qzy.resize(T, K, J);
  qy.resize(T, J);

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  pzmc.resize(K);
  pzsc.resize(K);
  pzy.resize(K, J);
  py.resize(J);
  pymdpVar.resize(J);
  pysdpVar.resize(J);
  pymdqVar.resize(J);
  pysdqVar.resize(J);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "mu_c: " << mu_c << endl;
    cout << "sigma_c: " << sigma_c << endl;
    cout << "p_y: " << p_y << endl;
    cout << "mu_dpVar: " << mu_dpVar << endl;
    cout << "sigma_dpVar: " << sigma_dpVar << endl;
    cout << "mu_dqVar: " << mu_dqVar << endl;
    cout << "sigma_dqVar: " << sigma_dqVar << endl;
    // }}}
     //COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint j = 0; j < J; j++) {
        rho_y_p(t, j) = ::exp(
            -.5 * MT::sqr(dpVar(t) - mu_dpVar(j)) / MT::sqr(sigma_dpVar(j))
          );
        rho_y_q(t, j) = ::exp(
            -.5 * MT::sqr(dqVar(t) - mu_dqVar(j)) / MT::sqr(sigma_dqVar(j))
          );
      }
    }
    rho_y_pqd = rho_y_p % rho_y_q;
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++)
        rho_z_c(t, k) = ::exp(
            -.5 * MT::sqr(c(t) - mu_c(k)) / (sigma_c(k) * sigma_c(k))
            );
      rho_z_cpq[t]() = rho_z_c[t] % (~p_y * rho_y_pqd[t]);
      rho_z_cypq[t]() = p_y % (rho_y_pqd[t] ^ rho_z_c[t]);
    }
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  A * (rho_z_cpq[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~A * (rho_z_cpq[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z_cpq[t] % b[t];
      for(uint k = 0; k < K; k++)
        qzy[t][k]() = a(t, k) * b(t, k) * (~rho_z_cypq[t])[k];
      //qzy[t]() = repmat(a[t] % b[t], 1, J) % ~rho_z_cypq[t];
      //qy[t]() = my_sum(qzy[t], 1);
      tensorMarginal(qy[t](), qzy[t], TUP(1));
      normalizeDist(qz[t]());
      normalizeDist(qzy[t]());
      normalizeDist(qy[t]());
    }
    for(uint t = 0; t < T-1; t++) {
      qzz[t]() = A % ( (rho_z_cpq[t+1] % b[t+1]) ^ (a[t] % rho_z_cpq[t]) );
      normalizeDist(qzz[t]());
    }
    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    pzmc.setZero();
    pzsc.setZero();
    pzy.setZero();
    py.setZero();
    pymdpVar.setZero();
    pysdpVar.setZero();
    pymdqVar.setZero();
    pysdqVar.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      pzmc += c(t) * qz[t];
      pzsc += sqr(c(t) - mu_c) % qz[t];
      pzy += qzy[t];
      py += qy[t];
      pymdpVar += dpVar(t) * qy[t];
      pysdpVar += sqr(dpVar(t) - mu_dpVar) % qy[t];
      pymdqVar += dqVar(t) * qy[t];
      pysdqVar += sqr(dqVar(t) - mu_dqVar) % qy[t];
      if(t < T-1)
        pzz += qzz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);

#ifdef update_mu_c
    mu_c = pzmc / pz;
#endif
#ifdef update_sigma_c
    sigma_c = sqrt(pzsc / pz);
#endif
#ifdef update_p_y_z0
    for(int j = 0; j < J; j++)
      p_y(j, 0) = pzy(0, j) / pz(0);
#endif
#ifdef update_p_y_z1
    for(int j = 0; j < J; j++)
      p_y(j, 1) = pzy(1, j) / pz(1);
#endif
#ifdef update_mu_dpVar
    mu_dpVar = pymdpVar / py;
#endif
#ifdef update_sigma_dpVar
    sigma_dpVar = sqrt(pysdpVar / py);
#endif
#ifdef update_mu_dqVar
    mu_dqVar = pymdqVar / py;
#endif
#ifdef update_sigma_dqVar
    sigma_dqVar = sqrt(pysdqVar / py);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  double m;
  int mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z_cpq[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test

    wz[t]() = log(rho_z_cpq[t]);
    for(uint k = 0; k < K; k++) {
      m = temp(k, 0);
      mi = 0;
      for(uint kk = 1; kk < K; kk++) {
        if(m < temp(k, kk)) {
          m = temp(k, kk);
          mi = kk;
        }
      }
      wz(t, k) += m;
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "c", new arr(c));
  kvg.append("data", "dpVar", new arr(dpVar));
  kvg.append("data", "dqVar", new arr(dqVar));
  
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Viterbi + Observations"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-1.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("c"));
  plot->append("data", new String("dpVar"));
  plot->append("data", new String("dqVar"));
  plot->append("data", new String("vit"));
  kvg.append("plot", plot);
  // }}}
}
#endif // EM_CORR_ORIG
// }}}
// EM MOV {{{
#ifdef EM_MOV
#define update_mu_pVar
//#define update_sigma_pVar
#define update_mu_qVar
//#define update_sigma_qVar
void KeyFramer::EM(KeyValueGraph &kvg, const String &bb, uint wlen) {
  // Computing other BAMS {{{
  cout << " * computing posVar" << endl;
  computeVar(STRING("pos"));
  cout << " * computing quatVar" << endl;
  computeVar(STRING("quat"));
  // }}}
  // Observations {{{
  //String bp(STRING(bb << ":pos"));
  //String bo(STRING(bb << ":ori"));
  arr pVar = g4d().query("posVar", bb);
  arr qVar = g4d().query("quatVar", bb);
  // }}}
  // Parameters & other {{{
  uint T, K, J;
  double sigma_L, sigma_H;
  arr pi, A;
  arr mu_pVar, sigma_pVar;
  arr mu_qVar, sigma_qVar;
  arr rho_z, rho_z_pVar, rho_z_qVar;
  arr qz, qzz;
  arr a, b;
  arr pz, pzz, pzmpVar, pzspVar, pzmqVar, pzsqVar;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);
  sigma_L = .05;
  sigma_H = .1;
  mu_pVar = { 0, .05 };
  sigma_pVar = { sigma_H, sigma_L };
  mu_qVar = { 0, .1 };
  sigma_qVar = { sigma_H, sigma_L };

  T = pVar.d0;
  K = 2;
  J = 2;

  rho_z.resize(T, K);
  rho_z_pVar.resize(T, K);
  rho_z_qVar.resize(T, K);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  pzmpVar.resize(K);
  pzspVar.resize(K);
  pzmqVar.resize(K);
  pzsqVar.resize(K);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "mu_pVar: " << mu_pVar << endl;
    cout << "sigma_pVar: " << sigma_pVar << endl;
    cout << "mu_qVar: " << mu_qVar << endl;
    cout << "sigma_qVar: " << sigma_qVar << endl;
    // }}}
     //COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++) {
        rho_z_pVar(t, k) = ::exp(
            -.5 * MT::sqr(pVar(t) - mu_pVar(k)) / MT::sqr(sigma_pVar(k))
          );
        rho_z_qVar(t, k) = ::exp(
            -.5 * MT::sqr(qVar(t) - mu_qVar(k)) / MT::sqr(sigma_qVar(k))
          );
      }
    }
    rho_z = rho_z_pVar; // % rho_z_qVar; // TODO uncomment this??
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  A * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~A * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z[t] % b[t];
      normalizeDist(qz[t]());
    }
    for(uint t = 0; t < T-1; t++) {
      qzz[t]() = A % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
      normalizeDist(qzz[t]());
    }
    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    pzmpVar.setZero();
    pzspVar.setZero();
    pzmqVar.setZero();
    pzsqVar.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      // TODO check if qz down here is right
      pzmpVar += pVar(t) * qz[t];
      pzspVar += sqr(pVar(t) - mu_pVar) % qz[t];
      pzmqVar += qVar(t) * qz[t];
      pzsqVar += sqr(qVar(t) - mu_qVar) % qz[t];
      if(t < T-1)
        pzz += qzz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);

#ifdef update_mu_pVar
    mu_pVar = pzmpVar / pz;
#endif
#ifdef update_sigma_pVar
    sigma_pVar = sqrt(pzspVar / pz);
#endif
#ifdef update_mu_qVar
    mu_qVar = pzmqVar / pz;
#endif
#ifdef update_sigma_qVar
    sigma_qVar = sqrt(pzsqVar / pz);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  double m;
  int mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test

    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      m = temp(k, 0);
      mi = 0;
      for(uint kk = 1; kk < K; kk++) {
        if(m < temp(k, kk)) {
          m = temp(k, kk);
          mi = kk;
        }
      }
      wz(t, k) += m;
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "pVar", new arr(pVar));
  kvg.append("data", "qVar", new arr(qVar));
  
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Viterbi + Observations"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-1.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("vit"));
  plot->append("data", new String("pVar"));
  plot->append("data", new String("qVar"));
  kvg.append("plot", plot);
  // }}}
}
#endif // EM_MOV
// }}}

// EM_m {{{
//#define update_mu_pSpeedGP
//#define update_sigma_pSpeedGP
//#define update_mu_qSpeedGP
//#define update_sigma_qSpeedGP
void KeyFramer::EM_m(KeyValueGraph &kvg, const String &bb) {
  // Computing other BAMS {{{
  cout << " * computing posSpeed" << endl;
  computeSpeed(STRING("pos"));
  cout << " * computing quatSpeed" << endl;
  computeSpeed(STRING("quat"));
  cout << " * computing posSpeedGP" << endl;
  computeGP(STRING("posSpeed"));
  cout << " * computing quatSpeedGP" << endl;
  computeGP(STRING("quatSpeed"));

  cout << " * computing posGP" << endl;
  computeGP(STRING("pos"));
  cout << " * computing quatGP" << endl;
  computeGP(STRING("quat"));
  cout << " * computing posGPSpeed" << endl;
  computeSpeed(STRING("posGP"));
  cout << " * computing quatGPSpeed" << endl;
  computeSpeed(STRING("quatGP"));
  // }}}
  // Observations {{{
  arr pSpeedGP = g4d().query("posGPSpeed", bb);
  arr qSpeedGP = g4d().query("quatGPSpeed", bb);
  uint T = pSpeedGP.d0;
  // }}}
  // Parameters & other {{{
  double sigma_L, sigma_H;
  arr pi, A;
  arr p_zmm, p_z;
  arr mu_pSpeedGP, sigma_pSpeedGP;
  arr mu_qSpeedGP, sigma_qSpeedGP;
  arr rho_z, rho_mp, rho_mq;
  arr qz, qzz, qmp, qmq, qzmm;
  arr a, b;
  arr pz, pzz, pmp, pmq, pmpmpSpeedGP, pmpspSpeedGP, pmqmqSpeedGP, pmqsqSpeedGP;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);
  p_zmm = { 50, 1,
            1, 1,

            1, 9,
            9, 50 };
  p_zmm.reshape(2, 2, 2);
  normalizeDist(p_zmm);
  //p_z = my_sum(p_zmm, 0);
  tensorMarginal(p_z, p_zmm, TUP(0));

  sigma_L = 50;
  sigma_H = 50;
  mu_pSpeedGP = { 0, 100 };
  sigma_pSpeedGP = { sigma_L, sigma_H };
  mu_qSpeedGP = { 0, 100 };
  sigma_qSpeedGP = { sigma_L, sigma_H };

  uint K = 2;

  rho_mp.resize(T, K);
  rho_mq.resize(T, K);
  rho_z.resize(T, K);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  qmp.resize(T, K);
  qmq.resize(T, K);
  qzmm.resize({T, K, K, K});

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  pmp.resize(K);
  pmq.resize(K);
  pmpmpSpeedGP.resize(K);
  pmpspSpeedGP.resize(K);
  pmqmqSpeedGP.resize(K);
  pmqsqSpeedGP.resize(K);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "mu_pSpeedGP: " << mu_pSpeedGP << endl;
    cout << "sigma_pSpeedGP: " << sigma_pSpeedGP << endl;
    cout << "mu_qSpeedGP: " << mu_qSpeedGP << endl;
    cout << "sigma_qSpeedGP: " << sigma_qSpeedGP << endl;
    // }}}
    // COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++) {
        rho_mp(t, k) = ::exp(
            -.5 * MT::sqr(pSpeedGP(t) - mu_pSpeedGP(k)) / MT::sqr(sigma_pSpeedGP(k))
          );
        rho_mq(t, k) = ::exp(
            -.5 * MT::sqr(qSpeedGP(t) - mu_qSpeedGP(k)) / MT::sqr(sigma_qSpeedGP(k))
          );
      }
      rho_z[t]().setZero();
      for(uint k = 0; k < K; k++)
        rho_z(t, k) += sum((rho_mp[t] ^ rho_mq[t]) % p_zmm[k]);
    }
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  A * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~A * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z[t] % b[t];
      normalizeDist(qz[t]());

      if(t < T-1) {
        qzz[t]() = A % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
        normalizeDist(qzz[t]());
      }

      for(uint k = 0; k < K; k++)
        qzmm[t][k]() = a(t, k) * b(t, k) * p_zmm[k] % ( rho_mp[t] ^ rho_mq[t] );
      normalizeDist(qzmm[t]());

      //qmp[t]() = my_sum(qzmm[t], 1);
      tensorMarginal(qmp[t](), qzmm[t], TUP(1));
      normalizeDist(qmp[t]());

      //qmq[t]() = my_sum(qzmm[t], 2);
      tensorMarginal(qmq[t](), qzmm[t], TUP(2));
      normalizeDist(qmq[t]());
    }

    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    pmp.setZero();
    pmq.setZero();
    pmpmpSpeedGP.setZero();
    pmpspSpeedGP.setZero();
    pmqmqSpeedGP.setZero();
    pmqsqSpeedGP.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      if(t < T-1)
        pzz += qzz[t];
      pmp += qmp[t];
      pmq += qmq[t];
      pmpmpSpeedGP += pSpeedGP(t) * qz[t];
      pmpspSpeedGP += sqr(pSpeedGP(t) - mu_pSpeedGP) % qz[t];
      pmqmqSpeedGP += qSpeedGP(t) * qz[t];
      pmqsqSpeedGP += sqr(qSpeedGP(t) - mu_qSpeedGP) % qz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);
    // I almost find it absurd that I have to normalize here too..
    arr tmp = sum(A, 0);
    for(uint k = 0; k < K; k++)
        A[k]() /= tmp;

#ifdef update_mu_pSpeedGP
    mu_pSpeedGP = pmpmpSpeedGP / pz;
#endif
#ifdef update_sigma_pSpeedGP
    sigma_pSpeedGP = sqrt(pmpspSpeedGP / pz);
#endif
#ifdef update_mu_qSpeedGP
    mu_qSpeedGP = pmqmqSpeedGP / pz;
#endif
#ifdef update_sigma_qSpeedGP
    sigma_qSpeedGP = sqrt(pmqsqSpeedGP / pz);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  uint mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test
    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      mi = temp[k].maxIndex();
      wz(t, k) += temp(k, mi);
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "pSpeedGP", new arr(pSpeedGP));
  kvg.append("data", "qSpeedGP", new arr(qSpeedGP));
  KeyValueGraph *plot;

  plot = new KeyValueGraph();
  plot->append("title", new String("Movement: Viterbi"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("vit"));
  kvg.append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("Observations"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pSpeedGP"));
  plot->append("data", new String("qSpeedGP"));
  kvg.append("plot", plot);
  // }}}
}
#undef update_mu_pSpeedGP
#undef update_sigma_pSpeedGP
#undef update_mu_qSpeedGP
#undef update_sigma_qSpeedGP
// }}}
// EM_r {{{
//#define update_mu_dpSpeedGP
//#define update_sigma_dpSpeedGP
//#define update_mu_dqSpeedGP
//#define update_sigma_dqSpeedGP
void KeyFramer::EM_r(KeyValueGraph &kvg, const String &bA, const String &bB) {
  // Computing other BAMS {{{
  String bA_dPos = STRING(bA << "_dPos");
  String bA_dQuat = STRING(bA << "_dQuat");

  String bA_dPosSpeed = STRING(bA_dPos << "Speed");
  String bA_dQuatSpeed = STRING(bA_dQuat << "Speed");
  String bA_dPosSpeedGP = STRING(bA_dPosSpeed << "GP");
  String bA_dQuatSpeedGP = STRING(bA_dQuatSpeed << "GP");

  String bA_dPosGP = STRING(bA_dPos << "GP");
  String bA_dQuatGP = STRING(bA_dQuat << "GP");
  String bA_dPosGPSpeed = STRING(bA_dPosGP << "Speed");
  String bA_dQuatGPSpeed = STRING(bA_dQuatGP << "Speed");

  cout << " * computing " << bA_dPos << endl;
  computeDPos(bA);
  cout << " * computing " << bA_dQuat << endl;
  computeDQuat(bA);

  cout << " * computing " << bA_dPosSpeed << endl;
  computeSpeed(bA_dPos);
  cout << " * computing " << bA_dQuatSpeed << endl;
  computeSpeed(bA_dQuat);
  cout << " * computing " << bA_dPosSpeedGP << endl;
  computeGP(bA_dPosSpeed);
  cout << " * computing " << bA_dQuatSpeedGP << endl;
  computeGP(bA_dQuatSpeed);

  cout << " * computing " << bA_dPosGP << endl;
  computeGP(bA_dPos);
  cout << " * computing " << bA_dPosGPSpeed << endl;
  computeSpeed(bA_dPosGP);
  cout << " * computing " << bA_dQuatGP << endl;
  computeGP(bA_dQuat);
  cout << " * computing " << bA_dQuatGPSpeed << endl;
  computeSpeed(bA_dQuatGP);
  // }}}
  // Observations {{{
  arr dpSpeedGP = g4d().query(bA_dPosGPSpeed, bB);
  arr dqSpeedGP = g4d().query(bA_dQuatGPSpeed, bB);
  uint T = dpSpeedGP.d0;
  // }}}
  // Parameters & other {{{
  double sigma_L, sigma_H;
  arr pi, A;
  arr p_zmm, p_z;
  arr mu_dpSpeedGP, sigma_dpSpeedGP;
  arr mu_dqSpeedGP, sigma_dqSpeedGP;
  arr rho_z, rho_mp, rho_mq;
  arr qz, qzz, qmp, qmq, qzmm;
  arr a, b;
  arr pz, pzz, pmp, pmq, pmpmdpSpeedGP, pmpsdpSpeedGP, pmqmdqSpeedGP, pmqsdqSpeedGP;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);
  p_zmm = { 50, 9,
            9, 1,

            1, 1,
            1, 50 };
  p_zmm.reshape(2, 2, 2);
  normalizeDist(p_zmm);
  //p_z = my_sum(p_zmm, 0);
  tensorMarginal(p_z, p_zmm, TUP(0));

  sigma_L = 50;
  sigma_H = 50;
  mu_dpSpeedGP = { 100, 0 };
  sigma_dpSpeedGP = { sigma_H, sigma_L };
  mu_dqSpeedGP = { 200, 0 };
  sigma_dqSpeedGP = { 2*sigma_H, 2*sigma_L };

  uint K = 2;

  rho_mp.resize(T, K);
  rho_mq.resize(T, K);
  rho_z.resize(T, K);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  qmp.resize(T, K);
  qmq.resize(T, K);
  qzmm.resize({T, K, K, K});

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  pmp.resize(K);
  pmq.resize(K);
  pmpmdpSpeedGP.resize(K);
  pmpsdpSpeedGP.resize(K);
  pmqmdqSpeedGP.resize(K);
  pmqsdqSpeedGP.resize(K);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "mu_dpSpeedGP: " << mu_dpSpeedGP << endl;
    cout << "sigma_dpSpeedGP: " << sigma_dpSpeedGP << endl;
    cout << "mu_dqSpeedGP: " << mu_dqSpeedGP << endl;
    cout << "sigma_dqSpeedGP: " << sigma_dqSpeedGP << endl;
    // }}}
    //COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++) {
        rho_mp(t, k) = ::exp(
            -.5 * MT::sqr(dpSpeedGP(t) - mu_dpSpeedGP(k)) / MT::sqr(sigma_dpSpeedGP(k))
          );
        rho_mq(t, k) = ::exp(
            -.5 * MT::sqr(dqSpeedGP(t) - mu_dqSpeedGP(k)) / MT::sqr(sigma_dqSpeedGP(k))
          );
      }
      rho_z[t]().setZero();
      for(uint k = 0; k < K; k++)
        rho_z(t, k) += sum((rho_mp[t] ^ rho_mq[t]) % p_zmm[k]);
    }
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  A * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~A * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z[t] % b[t];
      normalizeDist(qz[t]());

      if(t < T-1) {
        qzz[t]() = A % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
        normalizeDist(qzz[t]());
      }

      for(uint k = 0; k < K; k++)
        qzmm[t][k]() = a(t, k) * b(t, k) * p_zmm[k] % ( rho_mp[t] ^ rho_mq[t] );
      normalizeDist(qzmm[t]());

      //qmp[t]() = my_sum(qzmm[t], 1);
      tensorMarginal(qmp[t](), qzmm[t], TUP(1));
      normalizeDist(qmp[t]());

      //qmq[t]() = my_sum(qzmm[t], 2);
      tensorMarginal(qmq[t](), qzmm[t], TUP(2));
      normalizeDist(qmq[t]());
    }

    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    pmp.setZero();
    pmq.setZero();
    pmpmdpSpeedGP.setZero();
    pmpsdpSpeedGP.setZero();
    pmqmdqSpeedGP.setZero();
    pmqsdqSpeedGP.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      if(t < T-1)
        pzz += qzz[t];
      pmp += qmp[t];
      pmq += qmq[t];
      pmpmdpSpeedGP += dpSpeedGP(t) * qz[t];
      pmpsdpSpeedGP += sqr(dpSpeedGP(t) - mu_dpSpeedGP) % qz[t];
      pmqmdqSpeedGP += dqSpeedGP(t) * qz[t];
      pmqsdqSpeedGP += sqr(dqSpeedGP(t) - mu_dqSpeedGP) % qz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);
    // I almost find it absurd that I have to normalize here too..
    arr tmp = sum(A, 0);
    for(uint k = 0; k < K; k++)
        A[k]() /= tmp;

#ifdef update_mu_dpSpeedGP
    mu_dpSpeedGP = pmpmdpSpeedGP / pz;
#endif
#ifdef update_sigma_dpSpeedGP
    sigma_dpSpeedGP = sqrt(pmpsdpSpeedGP / pz);
#endif
#ifdef update_mu_dqSpeedGP
    mu_dqSpeedGP = pmqmdqSpeedGP / pz;
#endif
#ifdef update_sigma_dqSpeedGP
    sigma_dqSpeedGP = sqrt(pmqsdqSpeedGP / pz);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  uint mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test
    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      mi = temp[k].maxIndex();
      wz(t, k) += temp(k, mi);
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "dpSpeedGP", new arr(dpSpeedGP));
  kvg.append("data", "dqSpeedGP", new arr(dqSpeedGP));
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Rigidity: Viterbi"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("vit"));
  kvg.append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("Observations"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("dpSpeedGP"));
  plot->append("data", new String("dqSpeedGP"));
  kvg.append("plot", plot);
  // }}}
}
#undef update_mu_dpSpeedGP
#undef update_sigma_dpSpeedGP
#undef update_mu_dqSpeedGP
#undef update_sigma_dqSpeedGP
// }}}
// EM_c {{{
//#define update_mu_dist
//#define update_sigma_dist
void KeyFramer::EM_c(KeyValueGraph &kvg, const String &bA, const String &bB) {
  CHECK(g4d().digits().contains(bA), "First body must be agent.");
  CHECK(g4d().objects().contains(bB), "First body must be object.");
  // Computing other BAMS {{{
  computeDist();
  // }}}
  // Observations {{{
  uint T = g4d().numFrames();
  uint ia = g4d().digits().findValue(bA);
  uint io = g4d().objects().findValue(bB);
  arr dist = s->indd.sub(0, -1, ia, ia, io, io).flatten();
  // }}}
  // Parameters & other {{{
  arr pi, P;
  arr mu_dist, sigma_dist;
  arr rho_z;
  arr qz, qzz;
  arr a, b;
  arr pz, pzz, pmd, psd;

  pi = { 1, 0 };
  P = { .95, .05,
        .05, .95 };
  P.reshape(2, 2);

  mu_dist = { .05, 0 };
  sigma_dist = { .02, .02 };

  uint K = 2;
  rho_z.resize(T, K);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  pmd.resize(K);
  psd.resize(K);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "P: " << P << endl;
    cout << "mu_dist: " << mu_dist << endl;
    cout << "sigma_dist: " << sigma_dist << endl;
    // }}}
    //COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++)
      for(uint k = 0; k < K; k++)
        rho_z(t, k) = ::exp(
            -.5 * MT::sqr(dist(t) - mu_dist(k)) / MT::sqr(sigma_dist(k))
          );
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  P * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~P * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z[t] % b[t];
      normalizeDist(qz[t]());
      if(t < T-1) {
        qzz[t]() = P % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
        normalizeDist(qzz[t]());
      }
    }
    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    pmd.setZero();
    psd.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      if(t < T-1)
        pzz += qzz[t];
      pmd += dist(t) * qz[t];
      psd += sqr(dist(t) - mu_dist) % qz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        P(k, l) = pzz(k, l) / pz(l);
    // I almost find it absurd that I have to normalize here too..
    arr tmp = sum(P, 0);
    for(uint k = 0; k < K; k++)
        P[k]() /= tmp;

#ifdef update_mu_dist
    mu_dist = pmd / pz;
#endif
#ifdef update_sigma_dist
    sigma_dist = sqrt(psd / pz);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  uint mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(P) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test
    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      mi = temp[k].maxIndex();
      wz(t, k) += temp(k, mi);
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "dist", new arr(dist));
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Contact: Viterbi"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("vit"));
  kvg.append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("Observations"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("dist"));
  kvg.append("plot", plot);
  // }}}
}
#undef update_mu_dist
#undef update_sigma_dist
// }}}
// EM_z_with_c (single-agent, with contact) {{{
//#define with_object_emission
//#define update_mu_c
//#define update_sigma_c
//#define update_mu_p
//#define update_sigma_p
//#define update_mu_q
//#define updage_sigma_q
//#define update_mu_dp
//#define update_sigma_dp
//#define update_mu_dq
//#define update_sigma_dq
void KeyFramer::EM_z_with_c(KeyValueGraph &kvg, const String &subj, const String &obj) {
  CHECK(g4d().digits().contains(subj), "First body must be agent.");
  CHECK(g4d().objects().contains(obj), "First body must be object.");
  // Computing other BAMS {{{
  double alpha = .3;
  bool force = false;
  computeSmooth(STRINGS("pos", "quat"), alpha, force);
  computeSpeed(STRINGS("posSmooth", "quatSmooth"), force);

  String subj_dPos = STRING(subj << "_dPos");
  String subj_dQuat = STRING(subj << "_dQuat");
  String subj_dPosSmooth = STRING(subj_dPos << "Smooth");
  String subj_dQuatSmooth = STRING(subj_dQuat << "Smooth");
  String subj_dPosSmoothSpeed = STRING(subj_dPosSmooth << "Speed");
  String subj_dQuatSmoothSpeed = STRING(subj_dQuatSmooth << "Speed");

  computeDPos(subj, force);
  computeDQuat(subj, force);
  computeSmooth(subj_dPos, alpha, force);
  computeSmooth(subj_dQuat, alpha, force);
  computeSpeed(subj_dPosSmooth, force);
  computeSpeed(subj_dQuatSmooth, force);

  computeDist();
  // }}}
  // Observations {{{
  uint T = g4d().numFrames();
  uint ia = g4d().digits().findValue(subj);
  uint io = g4d().objects().findValue(obj);
  arr dist = s->indd.sub(0, -1, ia, ia, io, io).flatten();
  arr pSpeed = g4d().query("posSmoothSpeed", subj);
  arr qSpeed = g4d().query("quatSmoothSpeed", subj);
#ifdef with_object_emission
  arr pBSpeed = g4d().query("posSmoothSpeed", obj);
  arr qBSpeed = g4d().query("quatSmoothSpeed", obj);
#endif
  arr dpSpeed = g4d().query(subj_dPosSmoothSpeed, obj);
  arr dqSpeed = g4d().query(subj_dQuatSmoothSpeed, obj);
  // }}}
  // Parameters & other {{{
  uint K = 2;
  uint C = 2;
  uint R = 2, Rp = 2, Rq = 2;
  uint M = 2, Mp = 2, Mq = 2;

  double mu_c_OFF, sigma_c_OFF, mu_c_ON, sigma_c_ON;
  double mu_r_OFF, sigma_r_OFF, mu_r_ON, sigma_r_ON;
  double mu_m_OFF, sigma_m_OFF, mu_m_ON, sigma_m_ON;

  arr pi, P;
  arr p_zmm, p_z;
  arr mu_c, sigma_c;
  arr mu_dp, sigma_dp;
  arr mu_dq, sigma_dq;
  arr mu_p, sigma_p;
  arr mu_q, sigma_q;
  arr rho_z, rho_cr, rho_crm, rho_crmm;
  arr rho_c;
  arr rho_r, rho_rdp, rho_rdq, rho_rdpdq;
  arr rho_m, rho_mp, rho_mq, rho_mpq;
  arr rho_mB, rho_mpB, rho_mqB, rho_mpqB;
  arr qz, qzz, qmp, qmq, qzmm;
  arr a, b;
  arr pz, pzz, pmp, pmq;
  arr pmpmp, pmpsp, pmqmq, pmqsq;
  arr pmpmdp, pmpsdp, pmqmdq, pmqsdq;
  //arr phi_zmr, phi_mpq, phi_rpq;
  arr phi_zcrm, phi_rpq, phi_mpq;

  pi = { 1, 0 };
  P = { .5, .5,
        .5, .5 };
  P.reshape(2, 2);
#ifdef with_object_emission
  // TODO add c here..
  phi_zmr = { 50, 9,
              9, 9,

              9, 9,
              9, 1,

              1, 1,
              1, 1,

              1, 1,
              1, 50 };
  phi_zmr.reshape(TUP(2, 2, 2, 2));
#else
  phi_zcrm = {  99, 9,
                9, 6,

                6, 3,
                3, 1,

                1, 3,
                3, 6,

                6, 9,
                9, 99 };
  phi_zcrm.reshape(TUP(K, C, R, M));
#endif
  normalizeDist(phi_zcrm);
  //p_z = my_sum(p_zmm, 0);
  //tensorMarginal(p_z, p_zmm, TUP(0));
  
  phi_mpq = { 50, 1,
              1, 1,

              1, 9,
              9, 50 };
  phi_mpq.reshape(M, Mp, Mq);
  normalizeDist(phi_mpq);

  phi_rpq = { 50, 9,
              9, 1,

              1, 1,
              1, 50 };
  phi_rpq.reshape(R, Rp, Rq);
  normalizeDist(phi_rpq);

  mu_c_OFF = .03; sigma_c_OFF = .02;
  mu_c_ON = 0;    sigma_c_ON = .02;
  mu_c = { mu_c_OFF, mu_c_ON };
  sigma_c = { sigma_c_OFF, sigma_c_ON };

  mu_r_OFF = 100; sigma_r_OFF = 50;
  mu_r_ON = 0;    sigma_r_ON = 50;
  mu_dp= { mu_r_OFF, mu_r_ON };
  sigma_dp= { sigma_r_OFF, sigma_r_ON };
  mu_dq= { 2*mu_r_OFF, 2*mu_r_ON };
  sigma_dq= { 2*sigma_r_OFF, 2*sigma_r_ON };

  mu_m_OFF = 0;   sigma_m_OFF = 50;
  mu_m_ON = 100;  sigma_m_ON = 50;
  mu_p = { mu_m_OFF, mu_m_ON };
  sigma_p = { sigma_m_OFF, sigma_m_ON };
  mu_q = { mu_m_OFF, mu_m_ON };
  sigma_q = { sigma_m_OFF, sigma_m_ON };

  rho_z.resize(T, K);
  rho_cr.resize(T, C, R);
  rho_crm.resize(TUP(T, C, R, M));
  rho_crmm.resize(TUP(T, C, R, M, M));
  // C
  rho_c.resize(T, C);
  // R
  rho_r.resize(T, R);
  rho_rdp.resize(T, Rp);
  rho_rdq.resize(T, Rq);
  rho_rdpdq.resize(T, Rp, Rq);
  // M
  rho_m.resize(T, M);
  rho_mp.resize(T, Mp);
  rho_mq.resize(T, Mq);
  rho_mpq.resize(T, Mp, Mq);
  // Mo
#ifdef with_object_emission
  rho_mB.resize(T, M);
  rho_mpB.resize(T, Mp);
  rho_mqB.resize(T, Mq);
  rho_mpqB.resize(T, Mp, Mq);
#endif

  // TODO change Mb -> mo
  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  //qmp.resize(T, K);
  //qmq.resize(T, K);
  //qzmm.resize({T, K, K, K});

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  //pmp.resize(K);
  //pmq.resize(K);
  //pmpmdp.resize(K);
  //pmpsdp.resize(K);
  //pmqmdq.resize(K);
  //pmqsdq.resize(K);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "P: " << P << endl;
    cout << "mu_c: " << mu_c << endl;
    cout << "sigma_c: " << sigma_c << endl;
    cout << "mu_dp: " << mu_dp << endl;
    cout << "sigma_dp: " << sigma_dp << endl;
    cout << "mu_dq: " << mu_dq << endl;
    cout << "sigma_dq: " << sigma_dq << endl;
    cout << "mu_p: " << mu_p << endl;
    cout << "sigma_p: " << sigma_p << endl;
    cout << "mu_q: " << mu_q << endl;
    cout << "sigma_q: " << sigma_q << endl;
    // }}}
    // COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++) {
        rho_c(t, k) = ::exp(
            -.5 * MT::sqr(dist(t) - mu_c(k)) / MT::sqr(sigma_c(k))
          );
        rho_rdp(t, k) = ::exp(
            -.5 * MT::sqr(dpSpeed(t) - mu_dp(k)) / MT::sqr(sigma_dp(k))
          );
        rho_rdq(t, k) = ::exp(
            -.5 * MT::sqr(dqSpeed(t) - mu_dq(k)) / MT::sqr(sigma_dq(k))
          );
        rho_mp(t, k) = ::exp(
            -.5 * MT::sqr(pSpeed(t) - mu_p(k)) / MT::sqr(sigma_p(k))
          );
        rho_mq(t, k) = ::exp(
            -.5 * MT::sqr(qSpeed(t) - mu_q(k)) / MT::sqr(sigma_q(k))
          );
#ifdef with_object_emission
        rho_mpB(t, k) = ::exp(
            -.5 * MT::sqr(pBSpeed(t) - mu_p(k)) / MT::sqr(sigma_p(k))
          );
        rho_mqB(t, k) = ::exp(
            -.5 * MT::sqr(qBSpeed(t) - mu_q(k)) / MT::sqr(sigma_q(k))
          );
#endif
      }
    }
    // R
    tensorEquation(rho_rdpdq, rho_rdp, TUP(0, 1), rho_rdq, TUP(0, 2), 0);
    tensorEquation(rho_r, phi_rpq, TUP(1, 2, 3), rho_rdpdq, TUP(0, 2, 3), 2);
    // M
    tensorEquation(rho_mpq, rho_mp, TUP(0, 1), rho_mq, TUP(0, 2), 0);
    tensorEquation(rho_m, phi_mpq, TUP(1, 2, 3), rho_mpq, TUP(0, 2, 3), 2);
    // CRM
    //tensorEquation(rho_mr, rho_m, TUP(0, 1), rho_r, TUP(0, 2), 0);
    tensorEquation(rho_cr, rho_c, TUP(0, 1), rho_r, TUP(0, 2), 0);
    tensorEquation(rho_crm, rho_cr, TUP(0, 1, 2), rho_m, TUP(0, 3), 0);
#ifndef with_object_emission
    tensorEquation(rho_z, phi_zcrm, TUP(1, 2, 3, 4), rho_crm, TUP(0, 2, 3, 4), 3);
#else
    // TODO
    tensorEquation(rho_mpqB, rho_mpB, TUP(0, 1), rho_mqB, TUP(0, 2), 0);
    tensorEquation(rho_mB, phi_mpq, TUP(1, 2, 3), rho_mpqB, TUP(0, 2, 3), 2);
    tensorEquation(rho_mmr, rho_mr, TUP(0, 1, 3), rho_mB, TUP(0, 2), 0);
    tensorEquation(rho_z, phi_zmr, TUP(1, 2, 3, 4), rho_mmr, TUP(0, 2, 3, 4), 3);
#endif
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  P * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~P * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z[t] % b[t];
      normalizeDist(qz[t]());

      if(t < T-1) {
        qzz[t]() = P % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
        normalizeDist(qzz[t]());
      }

      //for(uint k = 0; k < K; k++)
        //qzmm[t][k]() = a(t, k) * b(t, k) * p_zmm[k] % ( rho_mp[t] ^ rho_mq[t] );
      //normalizeDist(qzmm[t]());

      //qmp[t]() = my_sum(qzmm[t], 1);
      //tensorMarginal(qmp[t](), qzmm[t], TUP(1));
      //normalizeDist(qmp[t]());

      //qmq[t]() = my_sum(qzmm[t], 2);
      //tensorMarginal(qmq[t](), qzmm[t], TUP(2));
      //normalizeDist(qmq[t]());
    }

    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    //pmp.setZero();
    //pmq.setZero();
    //pmpmp.setZero();
    //pmpsp.setZero();
    //pmqmq.setZero();
    //pmqsq.setZero();
    //pmpmdp.setZero();
    //pmpsdp.setZero();
    //pmqmdq.setZero();
    //pmqsdq.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      if(t < T-1)
        pzz += qzz[t];
      //pmp += qmp[t];
      //pmq += qmq[t];
      //pmpmp+= pSpeed(t) * qz[t];
      //pmpsp+= sqr(pSpeed(t) - mu_p) % qz[t];
      //pmqmq+= qSpeed(t) * qz[t];
      //pmqsq+= sqr(qSpeed(t) - mu_q) % qz[t];
      //pmpmdp+= dpSpeed(t) * qz[t];
      //pmpsdp+= sqr(dpSpeed(t) - mu_dp) % qz[t];
      //pmqmdq+= dqSpeed(t) * qz[t];
      //pmqsdq+= sqr(dqSpeed(t) - mu_dq) % qz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        P(k, l) = pzz(k, l) / pz(l);
    arr tmp = sum(P, 0);
    for(uint k = 0; k < K; k++)
      P[k]() /= tmp;

#ifdef update_mu_c
    NIY;
#endif
#ifdef update_sigma_c
    NIY;
#endif
#ifdef update_mu_dp
    NIY;
    //mu_dp = pmpmdp/ pz;
#endif
#ifdef update_sigma_dp
    NIY;
    //sigma_dp = sqrt(pmpsdp/ pz);
#endif
#ifdef update_mu_dq
    NIY;
    //mu_dq = pmqmdq/ pz;
#endif
#ifdef update_sigma_dq
    NIY;
    //sigma_dq = sqrt(pmqsdq/ pz);
#endif
#ifdef update_mu_p
    NIY;
    //mu_p = pmpmp/ pz;
#endif
#ifdef update_sigma_p
    NIY;
    //sigma_p = sqrt(pmpsp/ pz);
#endif
#ifdef update_mu_q
    NIY;
    //mu_q = pmqmq/ pz;
#endif
#ifdef update_sigma_q
    NIY;
    //sigma_q = sqrt(pmqsq/ pz);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  uint mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(P) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test
    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      mi = temp[k].maxIndex();
      wz(t, k) += temp(k, mi);
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  if(!kvg["vit"]) {
    arr *p_vit = new arr(g4d().subjects().N, g4d().objects().N, g4d().numFrames());
    kvg.append("vit", p_vit);
  }

  int ind_subj, ind_obj;
  ind_subj = g4d().subjects().findValue(subj);
  ind_obj = g4d().objects().findValue(obj);
  kvg.getValue<arr>("vit")->subDim(ind_subj, ind_obj)() = vit;

  KeyValueGraph *hmm, *plot;
  
  hmm = new KeyValueGraph;
  hmm->append("data", "vit", new arr(vit));
  hmm->append("data", "dist", new arr(dist));
  hmm->append("data", "dpSpeed", new arr(dpSpeed));
  hmm->append("data", "dqSpeed", new arr(dqSpeed));
  hmm->append("data", "pSpeed", new arr(pSpeed));
  hmm->append("data", "qSpeed", new arr(qSpeed));
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Full System: Viterbi"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("vit"));
  hmm->append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("dist"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("dist"));
  hmm->append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("dp, dq"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("dpSpeed"));
  plot->append("data", new String("dqSpeed"));
  hmm->append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("p, q"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pSpeed"));
  plot->append("data", new String("qSpeed"));
  hmm->append("plot", plot);

#ifdef with_object_emission
  hmm->append("data", "pBSpeed", new arr(pBSpeed));
  hmm->append("data", "qBSpeed", new arr(qBSpeed));

  plot = new KeyValueGraph();
  plot->append("title", new String("pBSpeed, qBSpeed"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pBSpeed"));
  plot->append("data", new String("qBSpeed"));
  kvg.append("plot", plot);
#endif

  kvg.append(STRINGS("hmm", subj, obj), hmm);
  // }}}
}
#undef with_object_emission
#undef update_mu_p
#undef update_sigma_p
#undef update_mu_q
#undef updage_sigma_q
#undef update_mu_dp
#undef update_sigma_dp
#undef update_mu_dq
#undef update_sigma_dq
// }}}
// EM_z (single-agent) {{{
//#define with_object_emission
//#define update_mu_p
//#define update_sigma_p
//#define update_mu_q
//#define updage_sigma_q
//#define update_mu_dp
//#define update_sigma_dp
//#define update_mu_dq
//#define update_sigma_dq
void KeyFramer::EM_z(KeyValueGraph &kvg, const String &bA, const String &bB) {
  // Computing other BAMS {{{
  double alpha = .3;
  bool force = false;
  computeSmooth(STRINGS("pos", "quat"), alpha, force);
  computeSpeed(STRINGS("posSmooth", "quatSmooth"), force);

  String bA_dPos = STRING(bA << "_dPos");
  String bA_dQuat = STRING(bA << "_dQuat");
  String bA_dPosSmooth = STRING(bA_dPos << "Smooth");
  String bA_dQuatSmooth = STRING(bA_dQuat << "Smooth");
  String bA_dPosSmoothSpeed = STRING(bA_dPosSmooth << "Speed");
  String bA_dQuatSmoothSpeed = STRING(bA_dQuatSmooth << "Speed");

  computeDPos(bA, force);
  computeDQuat(bA, force);
  computeSmooth(bA_dPos, alpha, force);
  computeSmooth(bA_dQuat, alpha, force);
  computeSpeed(bA_dPosSmooth, force);
  computeSpeed(bA_dQuatSmooth, force);
  // }}}
  // Observations {{{
  arr pSpeed = g4d().query("posSmoothSpeed", bA);
  arr qSpeed = g4d().query("quatSmoothSpeed", bA);
#ifdef with_object_emission
  arr pBSpeed = g4d().query("posSmoothSpeed", bB);
  arr qBSpeed = g4d().query("quatSmoothSpeed", bB);
#endif
  arr dpSpeed = g4d().query(bA_dPosSmoothSpeed, bB);
  arr dqSpeed = g4d().query(bA_dQuatSmoothSpeed, bB);
  uint T = pSpeed.d0;
  // }}}
  // Parameters & other {{{
  double mu_m_L, mu_m_H;
  double sigma_m_L, sigma_m_H;
  double mu_r_L, mu_r_H;
  double sigma_r_L, sigma_r_H;
  arr pi, A;
  arr p_zmm, p_z;
  arr mu_p, sigma_p;
  arr mu_q, sigma_q;
  arr mu_dp, sigma_dp;
  arr mu_dq, sigma_dq;
  arr rho_z, rho_mr, rho_mmr;
  arr rho_m, rho_mp, rho_mq, rho_mpq;
  arr rho_mB, rho_mpB, rho_mqB, rho_mpqB;
  arr rho_r, rho_rdp, rho_rdq, rho_rdpdq;
  arr qz, qzz, qmp, qmq, qzmm;
  arr a, b;
  arr pz, pzz, pmp, pmq;
  arr pmpmp, pmpsp, pmqmq, pmqsq;
  arr pmpmdp, pmpsdp, pmqmdq, pmqsdq;
  arr phi_zmr, phi_mpq, phi_rpq;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);
#ifdef with_object_emission
  phi_zmr = { 50, 9,
              9, 9,

              9, 9,
              9, 1,

              1, 1,
              1, 1,

              1, 1,
              1, 50 };
  phi_zmr.reshape(TUP(2, 2, 2, 2));
#else
  phi_zmr = { 50, 9,
              9, 1,

              1, 1,
              1, 50 };
  phi_zmr.reshape(2, 2, 2);
#endif
  normalizeDist(phi_zmr);
  //p_z = my_sum(p_zmm, 0);
  //tensorEquation(p_z, p_zmm, TUP(0));
  
  phi_mpq = { 50, 1,
              1, 1,

              1, 9,
              9, 50 };
  phi_mpq.reshape(2, 2, 2);
  normalizeDist(phi_mpq);

  phi_rpq = { 50, 9,
              9, 1,

              1, 1,
              1, 50 };
  phi_rpq.reshape(2, 2, 2);
  normalizeDist(phi_rpq);

  mu_m_L = 0;
  mu_m_H = 100;
  sigma_m_L = 50;
  sigma_m_H = 50;
  mu_p= { mu_m_L, mu_m_H };
  sigma_p= { sigma_m_L, sigma_m_H };
  mu_q= { mu_m_L, mu_m_H };
  sigma_q= { sigma_m_L, sigma_m_H };

  mu_r_L = 0;
  mu_r_H = 100;
  sigma_r_L = 50;
  sigma_r_H = 50;
  mu_dp= { mu_r_H, mu_r_L };
  sigma_dp= { sigma_r_H, sigma_r_L };
  mu_dq= { 2*mu_r_H, 2*mu_r_L };
  sigma_dq= { 2*sigma_r_H, 2*sigma_r_L };

  uint K = 2;
  uint M = 2;
  uint Mp = 2;
  uint Mq = 2;
  uint R = 2;
  uint Rdp = 2;
  uint Rdq = 2;

  rho_z.resize(T, K);
  rho_mr.resize(T, M, R);
  rho_mmr.resize(TUP(T, M, M, R));
  rho_m.resize(T, M);
  rho_mp.resize(T, Mp);
  rho_mq.resize(T, Mq);
  rho_mpq.resize(T, Mp, Mq);
#ifdef with_object_emission
  rho_mB.resize(T, M);
  rho_mpB.resize(T, Mp);
  rho_mqB.resize(T, Mq);
  rho_mpqB.resize(T, Mp, Mq);
#endif
  rho_r.resize(T, R);
  rho_rdp.resize(T, Rdp);
  rho_rdq.resize(T, Rdq);
  rho_rdpdq.resize(T, Rdp, Rdq);

  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  //qmp.resize(T, K);
  //qmq.resize(T, K);
  //qzmm.resize({T, K, K, K});

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  //pmp.resize(K);
  //pmq.resize(K);
  //pmpmdp.resize(K);
  //pmpsdp.resize(K);
  //pmqmdq.resize(K);
  //pmqsdq.resize(K);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "mu_p: " << mu_p<< endl;
    cout << "sigma_p: " << sigma_p<< endl;
    cout << "mu_q: " << mu_q<< endl;
    cout << "sigma_q: " << sigma_q<< endl;
    cout << "mu_dp: " << mu_dp<< endl;
    cout << "sigma_dp: " << sigma_dp<< endl;
    cout << "mu_dq: " << mu_dq<< endl;
    cout << "sigma_dq: " << sigma_dq<< endl;
    // }}}
    //COMPUTE EVIDENCES {{{
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++) {
        rho_mp(t, k) = ::exp(
            -.5 * MT::sqr(pSpeed(t) - mu_p(k)) / MT::sqr(sigma_p(k))
          );
        rho_mq(t, k) = ::exp(
            -.5 * MT::sqr(qSpeed(t) - mu_q(k)) / MT::sqr(sigma_q(k))
          );
        rho_rdp(t, k) = ::exp(
            -.5 * MT::sqr(dpSpeed(t) - mu_dp(k)) / MT::sqr(sigma_dp(k))
          );
        rho_rdq(t, k) = ::exp(
            -.5 * MT::sqr(dqSpeed(t) - mu_dq(k)) / MT::sqr(sigma_dq(k))
          );
#ifdef with_object_emission
        rho_mpB(t, k) = ::exp(
            -.5 * MT::sqr(pBSpeed(t) - mu_p(k)) / MT::sqr(sigma_p(k))
          );
        rho_mqB(t, k) = ::exp(
            -.5 * MT::sqr(qBSpeed(t) - mu_q(k)) / MT::sqr(sigma_q(k))
          );
#endif
      }
    }
    tensorEquation(rho_mpq, rho_mp, TUP(0, 1), rho_mq, TUP(0, 2), 0);
    tensorEquation(rho_m, phi_mpq, TUP(1, 2, 3), rho_mpq, TUP(0, 2, 3), 2);
    tensorEquation(rho_rdpdq, rho_rdp, TUP(0, 1), rho_rdq, TUP(0, 2), 0);
    tensorEquation(rho_r, phi_rpq, TUP(1, 2, 3), rho_rdpdq, TUP(0, 2, 3), 2);
    tensorEquation(rho_mr, rho_m, TUP(0, 1), rho_r, TUP(0, 2), 0);
#ifdef with_object_emission
    tensorEquation(rho_mpqB, rho_mpB, TUP(0, 1), rho_mqB, TUP(0, 2), 0);
    tensorEquation(rho_mB, phi_mpq, TUP(1, 2, 3), rho_mpqB, TUP(0, 2, 3), 2);
    tensorEquation(rho_mmr, rho_mr, TUP(0, 1, 3), rho_mB, TUP(0, 2), 0);
    tensorEquation(rho_z, phi_zmr, TUP(1, 2, 3, 4), rho_mmr, TUP(0, 2, 3, 4), 3);
#else
    tensorEquation(rho_z, phi_zmr, TUP(1, 2, 3), rho_mr, TUP(0, 2, 3), 2);
#endif
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  A * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~A * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z[t] % b[t];
      normalizeDist(qz[t]());

      if(t < T-1) {
        qzz[t]() = A % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
        normalizeDist(qzz[t]());
      }

      //for(uint k = 0; k < K; k++)
        //qzmm[t][k]() = a(t, k) * b(t, k) * p_zmm[k] % ( rho_mp[t] ^ rho_mq[t] );
      //normalizeDist(qzmm[t]());

      //qmp[t]() = my_sum(qzmm[t], 1);
      //tensorMarginal(qmp[t](), qzmm[t], TUP(1));
      //normalizeDist(qmp[t]());

      //qmq[t]() = my_sum(qzmm[t], 2);
      //tensorMarginal(qmq[t](), qzmm[t], TUP(2));
      //normalizeDist(qmq[t]());
    }

    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    //pmp.setZero();
    //pmq.setZero();
    //pmpmp.setZero();
    //pmpsp.setZero();
    //pmqmq.setZero();
    //pmqsq.setZero();
    //pmpmdp.setZero();
    //pmpsdp.setZero();
    //pmqmdq.setZero();
    //pmqsdq.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      if(t < T-1)
        pzz += qzz[t];
      //pmp += qmp[t];
      //pmq += qmq[t];
      //pmpmp+= pSpeed(t) * qz[t];
      //pmpsp+= sqr(pSpeed(t) - mu_p) % qz[t];
      //pmqmq+= qSpeed(t) * qz[t];
      //pmqsq+= sqr(qSpeed(t) - mu_q) % qz[t];
      //pmpmdp+= dpSpeed(t) * qz[t];
      //pmpsdp+= sqr(dpSpeed(t) - mu_dp) % qz[t];
      //pmqmdq+= dqSpeed(t) * qz[t];
      //pmqsdq+= sqr(dqSpeed(t) - mu_dq) % qz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);
    arr tmp = sum(A, 0);
    for(uint k = 0; k < K; k++)
      A[k]() /= tmp;

    // TODO change pz to appropriate
#ifdef update_mu_p
    mu_p = pmpmp/ pz;
#endif
#ifdef update_sigma_p
    sigma_p = sqrt(pmpsp/ pz);
#endif
#ifdef update_mu_q
    mu_q = pmqmq/ pz;
#endif
#ifdef update_sigma_q
    sigma_q = sqrt(pmqsq/ pz);
#endif
#ifdef update_mu_dp
    mu_dp = pmpmdp/ pz;
#endif
#ifdef update_sigma_dp
    sigma_dp = sqrt(pmpsdp/ pz);
#endif
#ifdef update_mu_dq
    mu_dq = pmqmdq/ pz;
#endif
#ifdef update_sigma_dq
    sigma_dq = sqrt(pmqsdq/ pz);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  uint mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test
    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      mi = temp[k].maxIndex();
      wz(t, k) += temp(k, mi);
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "pSpeed", new arr(pSpeed));
  kvg.append("data", "qSpeed", new arr(qSpeed));
  kvg.append("data", "dpSpeed", new arr(dpSpeed));
  kvg.append("data", "dqSpeed", new arr(dqSpeed));
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Full System: Viterbi"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("vit"));
  kvg.append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("p, q"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pSpeed"));
  plot->append("data", new String("qSpeed"));
  kvg.append("plot", plot);

#ifdef with_object_emission
  kvg.append("data", "pBSpeed", new arr(pBSpeed));
  kvg.append("data", "qBSpeed", new arr(qBSpeed));

  plot = new KeyValueGraph();
  plot->append("title", new String("pBSpeed, qBSpeed"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pBSpeed"));
  plot->append("data", new String("qBSpeed"));
  kvg.append("plot", plot);
#endif

  plot = new KeyValueGraph();
  plot->append("title", new String("dp, dq"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("dpSpeed"));
  plot->append("data", new String("dqSpeed"));
  kvg.append("plot", plot);
  // }}}
}
#undef with_object_emission
#undef update_mu_p
#undef update_sigma_p
#undef update_mu_q
#undef updage_sigma_q
#undef update_mu_dp
#undef update_sigma_dp
#undef update_mu_dq
#undef update_sigma_dq
// }}}
// EM_z (multi-agent) {{{
#define with_object_emission
//#define update_mu_p
//#define update_sigma_p
//#define update_mu_q
//#define updage_sigma_q
//#define update_mu_dp
//#define update_sigma_dp
//#define update_mu_dq
//#define update_sigma_dq
void KeyFramer::EM_z(KeyValueGraph &kvg, const StringA &bA, const String &bB) {
  // Computing other BAMS {{{
  double alpha = .3;
  bool force = false;
  computeSmooth(STRINGS("pos", "quat"), alpha, force);
  computeSpeed(STRINGS("posSmooth", "quatSmooth"), force);

  StringA bA_dPos, bA_dQuat, bA_dPosSmooth, bA_dQuatSmooth, bA_dPosSmoothSpeed, bA_dQuatSmoothSpeed;

  uint A = bA.N;
  for(uint a = 0; a < A; a++) {
    bA_dPos.append(STRING(bA(a) << "_dPos"));
    bA_dQuat.append(STRING(bA(a) << "_dQuat"));
    bA_dPosSmooth.append(STRING(bA_dPos(a) << "Smooth"));
    bA_dQuatSmooth.append(STRING(bA_dQuat(a) << "Smooth"));
    bA_dPosSmoothSpeed.append(STRING(bA_dPosSmooth(a) << "Speed"));
    bA_dQuatSmoothSpeed.append(STRING(bA_dQuatSmooth(a) << "Speed"));

    computeDPos(bA(a), force);
    computeDQuat(bA(a), force);
    computeSmooth(bA_dPos(a), alpha, force);
    computeSmooth(bA_dQuat(a), alpha, force);
    computeSpeed(bA_dPosSmooth(a), force);
    computeSpeed(bA_dQuatSmooth(a), force);
  }
  // }}}
  // Observations {{{
  uint T = g4d().numFrames();
  arr pSpeed(A, T), qSpeed(A, T);
  arr dpSpeed(A, T), dqSpeed(A, T);
  
  for(uint a = 0; a < A; a++) {
    pSpeed[a]() = g4d().query("posSmoothSpeed", bA(a));
    qSpeed[a]() = g4d().query("quatSmoothSpeed", bA(a));
    dpSpeed[a]() = g4d().query(bA_dPosSmoothSpeed(a), bB);
    dqSpeed[a]() = g4d().query(bA_dQuatSmoothSpeed(a), bB);
  }
#ifdef with_object_emission
  arr pBSpeed = g4d().query("posSmoothSpeed", bB);
  arr qBSpeed = g4d().query("quatSmoothSpeed", bB);
#endif
  // }}}
  // Parameters & other {{{
  double mu_m_L, mu_m_H;
  double sigma_m_L, sigma_m_H;
  double mu_r_L, mu_r_H;
  double sigma_r_L, sigma_r_H;
  arr pi, P;
  arr p_zmm, p_z;
  arr mu_p, sigma_p;
  arr mu_q, sigma_q;
  arr mu_dp, sigma_dp;
  arr mu_dq, sigma_dq;
  arr rho_z, rho_za, rho_mr;
  arr rho_m, rho_mp, rho_mq, rho_mpq;
  arr rho_mB, rho_mpB, rho_mqB, rho_mpqB;
  arr rho_r, rho_rdp, rho_rdq, rho_rdpdq;
  arr qz, qzz, qmp, qmq, qzmm;
  arr a, b;
  arr pz, pzz, pmp, pmq;
  arr pmpmp, pmpsp, pmqmq, pmqsq;
  arr pmpmdp, pmpsdp, pmqmdq, pmqsdq;
  arr phi_zmr, phi_mpq, phi_rpq;

  pi = { 1, 0 };
  P = { .5, .5,
        .5, .5 };
  P.reshape(2, 2);
#ifdef with_object_emission
  phi_zmr = { 50, 9,
              9, 9,

              9, 9,
              9, 1,

              1, 1,
              1, 1,

              1, 1,
              1, 50 };
  phi_zmr.reshape(TUP(2, 2, 2, 2));
#else
  phi_zmr = { 50, 9,
              9, 1,

              1, 1,
              1, 50 };
  phi_zmr.reshape(2, 2, 2);
#endif
  normalizeDist(phi_zmr);
  //p_z = my_sum(p_zmm, 0);
  //tensorMarginal(p_z, p_zmm, TUP(0));
  
  phi_mpq = { 50, 1,
              1, 1,

              1, 9,
              9, 50 };
  phi_mpq.reshape(2, 2, 2);
  normalizeDist(phi_mpq);

  phi_rpq = { 50, 9,
              9, 1,

              1, 1,
              1, 50 };
  phi_rpq.reshape(2, 2, 2);
  normalizeDist(phi_rpq);

  mu_m_L = 0;
  mu_m_H = 100;
  sigma_m_L = 50;
  sigma_m_H = 50;
  mu_p= { mu_m_L, mu_m_H };
  sigma_p= { sigma_m_L, sigma_m_H };
  mu_q= { mu_m_L, mu_m_H };
  sigma_q= { sigma_m_L, sigma_m_H };

  mu_r_L = 0;
  mu_r_H = 100;
  sigma_r_L = 50;
  sigma_r_H = 50;
  mu_dp= { mu_r_H, mu_r_L };
  sigma_dp= { sigma_r_H, sigma_r_L };
  mu_dq= { 2*mu_r_H, 2*mu_r_L };
  sigma_dq= { 2*sigma_r_H, 2*sigma_r_L };

  uint K = 2;
  uint M = 2;
  uint Mp = 2;
  uint Mq = 2;
  uint R = 2;
  uint Rdp = 2;
  uint Rdq = 2;

  rho_z.resize(T, K);
  rho_za.resize(A, T, K);
  rho_mr.resize(TUP(A, T, M, R));
  rho_m.resize(A, T, M);
  rho_mp.resize(A, T, Mp);
  rho_mq.resize(A, T, Mq);
  rho_mpq.resize(TUP(A, T, Mp, Mq));
#ifdef with_object_emission
  rho_za.resize(TUP(A, T, K, M));
  rho_mB.resize(T, M);
  rho_mpB.resize(T, Mp);
  rho_mqB.resize(T, Mq);
  rho_mpqB.resize(T, Mp, Mq);
#endif
  rho_r.resize(A, T, R);
  rho_rdp.resize(A, T, Rdp);
  rho_rdq.resize(A, T, Rdq);
  rho_rdpdq.resize(TUP(A, T, Rdp, Rdq));

  // TODO do I have to resize these too?
  qz.resize(T, K);
  qzz.resize(T-1, K, K);
  //qmp.resize(T, K);
  //qmq.resize(T, K);
  //qzmm.resize({T, K, K, K});

  a.resize(T, K);
  b.resize(T, K);

  pz.resize(K);
  pzz.resize(K, K);
  //pmp.resize(K);
  //pmq.resize(K);
  //pmpmdp.resize(K);
  //pmpsdp.resize(K);
  //pmqmdq.resize(K);
  //pmqsdq.resize(K);
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "P: " << P << endl;
    cout << "mu_p: " << mu_p<< endl;
    cout << "sigma_p: " << sigma_p<< endl;
    cout << "mu_q: " << mu_q<< endl;
    cout << "sigma_q: " << sigma_q<< endl;
    cout << "mu_dp: " << mu_dp<< endl;
    cout << "sigma_dp: " << sigma_dp<< endl;
    cout << "mu_dq: " << mu_dq<< endl;
    cout << "sigma_dq: " << sigma_dq<< endl;
    // }}}
    //COMPUTE EVIDENCES {{{
    for(uint a = 0; a < A; a++) {
      for(uint t = 0; t < T; t++) {
        for(uint k = 0; k < K; k++) {
          rho_mp(a, t, k) = ::exp(
              -.5 * MT::sqr(pSpeed(a, t) - mu_p(k)) / MT::sqr(sigma_p(k))
            );
          rho_mq(a, t, k) = ::exp(
              -.5 * MT::sqr(qSpeed(a, t) - mu_q(k)) / MT::sqr(sigma_q(k))
            );
          rho_rdp(a, t, k) = ::exp(
              -.5 * MT::sqr(dpSpeed(a, t) - mu_dp(k)) / MT::sqr(sigma_dp(k))
            );
          rho_rdq(a, t, k) = ::exp(
              -.5 * MT::sqr(dqSpeed(a, t) - mu_dq(k)) / MT::sqr(sigma_dq(k))
            );
#ifdef with_object_emission
          rho_mpB(t, k) = ::exp(
              -.5 * MT::sqr(pBSpeed(t) - mu_p(k)) / MT::sqr(sigma_p(k))
            );
          rho_mqB(t, k) = ::exp(
              -.5 * MT::sqr(qBSpeed(t) - mu_q(k)) / MT::sqr(sigma_q(k))
            );
#endif
        }
      }
    }
    tensorEquation(rho_mpq, rho_mp, TUP(0, 1, 2), rho_mq, TUP(0, 1, 3), 0);
    tensorEquation(rho_m, phi_mpq, TUP(2, 3, 4), rho_mpq, TUP(0, 1, 3, 4), 2);
    tensorEquation(rho_rdpdq, rho_rdp, TUP(0, 1, 2), rho_rdq, TUP(0, 1, 3), 0);
    tensorEquation(rho_r, phi_rpq, TUP(2, 3, 4), rho_rdpdq, TUP(0, 1, 3, 4), 2);
    tensorEquation(rho_mr, rho_m, TUP(0, 1, 2), rho_r, TUP(0, 1, 3), 0);
#ifdef with_object_emission
    tensorEquation(rho_mpqB, rho_mpB, TUP(0, 1), rho_mqB, TUP(0, 2), 0);
    tensorEquation(rho_mB, phi_mpq, TUP(1, 2, 3), rho_mpqB, TUP(0, 2, 3), 2);

    arr rho_zmB;
    arr phi_z;
    rho_zmB.resize(TUP(A, T, K, M));
    phi_z.resize(TUP(T, K, M));
    tensorEquation(rho_zmB, rho_mr, TUP(0, 1, 4, 5), phi_zmr, TUP(2, 4, 3, 5), 2);
    phi_z = rho_zmB[0];
    for(uint a = 1; a < A; a++)
      tensorMultiply(phi_z, rho_zmB[a], TUP(0, 1, 2));
    tensorEquation(rho_z, rho_mB, TUP(0, 1), phi_z, TUP(0, 1, 2), 1);
#else
    tensorEquation(rho_za, phi_zmr, TUP(2, 3, 4), rho_mr, TUP(0, 1, 3, 4), 2);
    rho_z = rho_za[0];
    for(uint a = 1; a < A; a++)
      tensorMultiply(rho_z, rho_za[a], TUP(0, 1));
#endif
    // }}}
    // E-STEP {{{
    a[0]() = pi;   //initialization of alpha
    b[T-1]() = 1; //initialization of beta
    //--- fwd and bwd iterations:
    for(uint t = 1; t < T; t++) {
      a[t]() =  P * (rho_z[t-1] % a[t-1]);
      normalizeDist(a[t]());
    }
    for(uint t = T-1; t--; ) {
      b[t]() = ~P * (rho_z[t+1] % b[t+1]);
      normalizeDist(b[t]());
    }

    for(uint t = 0; t < T; t++) {
      qz[t]() = a[t] % rho_z[t] % b[t];
      normalizeDist(qz[t]());

      if(t < T-1) {
        qzz[t]() = P % ( (rho_z[t+1] % b[t+1]) ^ (a[t] % rho_z[t]) );
        normalizeDist(qzz[t]());
      }

      //for(uint k = 0; k < K; k++)
        //qzmm[t][k]() = a(t, k) * b(t, k) * p_zmm[k] % ( rho_mp[t] ^ rho_mq[t] );
      //normalizeDist(qzmm[t]());

      //qmp[t]() = my_sum(qzmm[t], 1);
      //tensorMarginal(qmp[t](), qzmm[t], TUP(1));
      //normalizeDist(qmp[t]());

      //qmq[t]() = my_sum(qzmm[t], 2);
      //tensorMarginal(qmq[t](), qzmm[t], TUP(2));
      //normalizeDist(qmq[t]());
    }

    // }}}
    // M-STEP {{{
    pz.setZero();
    pzz.setZero();
    //pmp.setZero();
    //pmq.setZero();
    //pmpmp.setZero();
    //pmpsp.setZero();
    //pmqmq.setZero();
    //pmqsq.setZero();
    //pmpmdp.setZero();
    //pmpsdp.setZero();
    //pmqmdq.setZero();
    //pmqsdq.setZero();
    for(uint t = 0; t < T; t++) {
      pz += qz[t];
      if(t < T-1)
        pzz += qzz[t];
      //pmp += qmp[t];
      //pmq += qmq[t];
      //pmpmp+= pSpeed(t) * qz[t];
      //pmpsp+= sqr(pSpeed(t) - mu_p) % qz[t];
      //pmqmq+= qSpeed(t) * qz[t];
      //pmqsq+= sqr(qSpeed(t) - mu_q) % qz[t];
      //pmpmdp+= dpSpeed(t) * qz[t];
      //pmpsdp+= sqr(dpSpeed(t) - mu_dp) % qz[t];
      //pmqmdq+= dqSpeed(t) * qz[t];
      //pmqsdq+= sqr(dqSpeed(t) - mu_dq) % qz[t];
    }

    pi = qz[0];
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        P(k, l) = pzz(k, l) / pz(l);
    arr tmp = sum(P, 0);
    for(uint k = 0; k < K; k++)
      P[k]() /= tmp;

    // TODO change pz to appropriate
#ifdef update_mu_p
    mu_p = pmpmp/ pz;
#endif
#ifdef update_sigma_p
    sigma_p = sqrt(pmpsp/ pz);
#endif
#ifdef update_mu_q
    mu_q = pmqmq/ pz;
#endif
#ifdef update_sigma_q
    sigma_q = sqrt(pmqsq/ pz);
#endif
#ifdef update_mu_dp
    mu_dp = pmpmdp/ pz;
#endif
#ifdef update_sigma_dp
    sigma_dp = sqrt(pmpsdp/ pz);
#endif
#ifdef update_mu_dq
    mu_dq = pmqmdq/ pz;
#endif
#ifdef update_sigma_dq
    sigma_dq = sqrt(pmqsdq/ pz);
#endif
    // }}}
    
    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  uint mi;
  arr wz(T, K), wzind(T, K), temp;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(P) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test
    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      mi = temp[k].maxIndex();
      wz(t, k) += temp(k, mi);
      wzind(t, k) = mi;
    }
  }

  arr vit(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
  // Return results as KVG {{{
  kvg.append("data", "vit", new arr(vit));
  kvg.append("data", "pSpeed", new arr(pSpeed));
  kvg.append("data", "qSpeed", new arr(qSpeed));
  kvg.append("data", "dpSpeed", new arr(dpSpeed));
  kvg.append("data", "dqSpeed", new arr(dqSpeed));
  KeyValueGraph *plot;
  
  plot = new KeyValueGraph();
  plot->append("title", new String("Full System: Viterbi"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("ymin", new double(-.1));
  plot->append("ymax", new double(1.1));
  plot->append("data", new String("vit"));
  kvg.append("plot", plot);

  plot = new KeyValueGraph();
  plot->append("title", new String("p, q"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pSpeed"));
  plot->append("data", new String("qSpeed"));
  kvg.append("plot", plot);

#ifdef with_object_emission
  kvg.append("data", "pBSpeed", new arr(pBSpeed));
  kvg.append("data", "qBSpeed", new arr(qBSpeed));

  plot = new KeyValueGraph();
  plot->append("title", new String("pBSpeed, qBSpeed"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pBSpeed"));
  plot->append("data", new String("qBSpeed"));
  kvg.append("plot", plot);
#endif

  plot = new KeyValueGraph();
  plot->append("title", new String("dp, dq"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("dpSpeed"));
  plot->append("data", new String("dqSpeed"));
  kvg.append("plot", plot);
  // }}}
}
#undef with_object_emission
#undef update_mu_p
#undef update_sigma_p
#undef update_mu_q
#undef updage_sigma_q
#undef update_mu_dp
#undef update_sigma_dp
#undef update_mu_dq
#undef update_sigma_dq
// }}}

void KeyFramer::testSmoothing(KeyValueGraph &kvg, const String &bA, double alpha) {
  computeFilter(STRING("pos"), alpha, true);
  computeSmooth(STRING("pos"), alpha, true);
  computeSpeed(STRINGS("pos", "posFilter", "posSmooth"), true);
  arr pSpeed_orig = g4d().query("posSpeed", bA);
  arr pSpeed_smooth = g4d().query("posSmoothSpeed", bA);
  arr pSpeed_filter = g4d().query("posFilterSpeed", bA);
  kvg.append("data", "pSpeed_orig", new arr(pSpeed_orig));
  kvg.append("data", "pSpeed_smooth", new arr(pSpeed_smooth));
  kvg.append("data", "pSpeed_filter", new arr(pSpeed_filter));
  KeyValueGraph *plot = new KeyValueGraph();
  plot->append("title", new String("Smoothing"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pSpeed_orig"));
  plot->append("data", new String("pSpeed_smooth"));
  plot->append("data", new String("pSpeed_filter"));
  kvg.append("plot", plot);
}

void KeyFramer::objFeatures(KeyValueGraph &feats, const String &b, uint fnum) {
  feats.clear();

  feats.append("fnum", new double(fnum));
  // TODO only xy features for now.
  arr pos = g4d().query("pos", b, fnum);
  feats.append("f_pos", new arr(pos.sub(0, 1)));
}

void KeyFramer::process(KeyValueGraph &kvg, const StringA &name_subjs, const StringA &name_objs) {
  for(auto name_subj: name_subjs)
    for(auto name_obj: name_objs)
      process(kvg, name_subj, name_obj);
}

// process {{{
void KeyFramer::process(KeyValueGraph &kvg, const String &name_subj, const String &name_obj) {
  if(g4d().digits().contains(name_subj)) {
    EM_z_with_c(kvg, name_subj, name_obj);
    return;
  }
  const StringA &sublimbs = g4d().sublimbs(name_subj);

  uint ind_subj, ind_obj, ind_sublimb;
  ind_subj = g4d().subjects().findValue(name_subj);
  ind_obj = g4d().objects().findValue(name_obj);

  arr *p_vit = NULL;
  for(const String &sublimb: sublimbs) {
    ind_sublimb = g4d().subjects().findValue(sublimb);
    process(kvg, sublimb, name_obj);
    if(!p_vit)
      p_vit = kvg.getValue<arr>("vit");
    for(uint f = 0; f < g4d().numFrames(); f++)
      if((*p_vit)(ind_subj, ind_obj, f) == 0)
        (*p_vit)(ind_subj, ind_obj, f) = (*p_vit)(ind_sublimb, ind_obj, f);
  }
}
// }}}

void KeyFramer::playScene(KeyValueGraph &kvg, const StringA &name_subjs, bool video) {
  CHECK(name_subjs.N <= 6, "Not done yet this many colors");
  VideoEncoder_x264_simple *vid;
  if(video)
   vid = new VideoEncoder_x264_simple(STRING("z.video.h264"), 80, 0, true);

  double gray[3] = { 1, 1, 1 };
  double col_subj[6][3] = {
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
    { 1, 1, 0 },
    { 1, 0, 1 },
    { 0, 1, 1 }
  };
  double col_obj[6][3] = {
    { 1, .2, .2 },
    { .2, 1, .2 },
    { .2, .2, 1 },
    { 1, 1, .2 },
    { 1, .2, 1 },
    { .2, 1, 1 }
  };

  arr *p_vit = kvg.getValue<arr>("vit");

  for(uint f = 0; f < g4d().numFrames(); f++) {
    for(uint ind_obj = 0; ind_obj < g4d().objects().N; ind_obj++)
      for(ors::Shape *shape: kw().getBodyByName(g4d().objects().elem(ind_obj))->shapes)
          memcpy(shape->color, gray, 3*sizeof(double));

    for(uint ns = 0; ns < name_subjs.N; ns++) {
      auto name_subj = name_subjs(ns);
      int ind_subj = g4d().subjects().findValue(name_subj);
      for(const String &digit: g4d().digitsof(name_subj))
        for(ors::Shape *shape: kw().getBodyByName(digit)->shapes)
          memcpy(shape->color, col_subj[ns], 3*sizeof(double));
      for(uint ind_obj = 0; ind_obj < g4d().objects().N; ind_obj++)
        for(ors::Shape *shape: kw().getBodyByName(g4d().objects().elem(ind_obj))->shapes)
          if((*p_vit)(ind_subj, ind_obj, f))
            memcpy(shape->color, col_obj[ns], 3*sizeof(double));
    }
    updateOrs(f, true);
    flip_image(kw().gl().captureImage);
    if(video)
      vid->addFrame(kw().gl().captureImage);
  }
  if(video)
    vid->close();
}

void KeyFramer::playScene(KeyValueGraph &kvg, const String &name_subj, bool video) {
  VideoEncoder_x264_simple *vid;
  if(video) vid = new VideoEncoder_x264_simple(STRING("z.video.h264"), 80, 0, true);

  double gray[3] = { 1, 1, 1 };
  double col_subj[3] = { 1, 0, 0 };
  double col_obj[3] = { 1, .5, .5 };

  for(const String &digit: g4d().digitsof(name_subj))
    for(ors::Shape *shape: kw().getBodyByName(digit)->shapes)
      memcpy(shape->color, col_subj, 3*sizeof(double));

  int ind_subj = g4d().subjects().findValue(name_subj);
  arr *p_vit = kvg.getValue<arr>("vit");
  for(uint f = 0; f < g4d().numFrames(); f++) {
    for(uint ind_obj = 0; ind_obj < g4d().objects().N; ind_obj++)
      for(ors::Shape *shape: kw().getBodyByName(g4d().objects().elem(ind_obj))->shapes)
        if((*p_vit)(ind_subj, ind_obj, f))
          memcpy(shape->color, col_obj, 3*sizeof(double));
        else
          memcpy(shape->color, gray, 3*sizeof(double));
    updateOrs(f, true);
    if(video)
      vid->addFrame(kw().gl().captureImage);
  }
  if(video)
    vid->close();
}

