#include "keyframer.h"
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>
#include <Algo/gaussianProcess.h>

// TODO replace with appropriate tensor function
arr my_sum(const arr &v, uint d) {
  arr x, S;
  uintA I;
  x.referTo(v);
  x.reshape(x.N);
  S.resize(v.dim(d));
  S.setZero();
  for(uint i = 0; i < x.N; i++) {
    v.getIndexTuple(I, i);
    S(I(d)) += x(i);
  }
  return S;
}

struct KeyFramer::sKeyFramer {
  ors::KinematicWorld *kw;
  G4Data *g4d;

  uint nbodies, ndofs;
  uintA dofs, cumdofs;
  StringA names;

  uint nframes;
  arr state;

  sKeyFramer(ors::KinematicWorld *_kw, G4Data *_g4d);
  ~sKeyFramer();

  void clear();
  void clearState();
  void clearFrames();

  void init();
  void addBody(String &name, uint body_ndof);

  void addState();
  void addState(arr st);
  void setState(uint b, arr st, uint f);
  void setState(String n, arr st, uint f);
  void setState(arr st, uint f);
  void setupWindows(MT::Array<arr> &wins, uint wlen);
};

// ============================================================================
// sKeyFramer
//

KeyFramer::sKeyFramer::sKeyFramer(ors::KinematicWorld *_kw, G4Data *_g4d): kw(_kw), g4d(_g4d) {
  clear();
  //init();
}

KeyFramer::sKeyFramer::~sKeyFramer() {}

// clear {{{
void KeyFramer::sKeyFramer::clear() {
  clearState();
}
// }}}
// clearState {{{
void KeyFramer::sKeyFramer::clearState() {
  nbodies = 0;
  ndofs = 0;
  clearFrames();
}
// }}}
// clearFrames {{{
void KeyFramer::sKeyFramer::clearFrames() {
  state.clear();
  nframes = 0;
}
// }}}

// init {{{
void KeyFramer::sKeyFramer::init() {
  StringA posNames;
  StringA oriNames;
  String name, pname, oname;

  // set up kf bodies and entities
  for(String name: g4d->getNames()) {
    pname.clear() << name << ":pos";
    oname.clear() << name << ":ori";

    posNames.append(pname);
    oriNames.append(oname);

    addBody(pname, 3);
    addBody(oname, 4);
    addBody(name, 7);
  }

  uint F = g4d->getNumFrames();
  for(uint f = 0; f < F; f++) {
    addState();
    for(uint i = 0; i < posNames.N; i++) {
      name = g4d->getName(i);
      pname = posNames(i);
      oname = oriNames(i);

      setState(pname, g4d->query("pos", name, f), f);
      setState(oname, g4d->query("quat", name, f), f);
      setState(name, g4d->query("pose", name, f), f);
    }
  }
}
// }}}
// addBody {{{
void KeyFramer::sKeyFramer::addBody(String &name, uint body_ndofs) {
  CHECK(body_ndofs > 0, "Number of Dofs for a body must be positive.");
  CHECK(!names.contains(name), "Body name already exists.");

  nbodies++;
  ndofs += body_ndofs;

  cumdofs.append(nbodies == 1? 0: cumdofs.last()+dofs.last());
  dofs.append(body_ndofs);
  names.append(name);
}
// }}}

// addState {{{
void KeyFramer::sKeyFramer::addState(arr st) {
  CHECK(st.N == ndofs, "Wrong number of dofs.");
  nframes++;
  state.append(st);
  state.reshape(nframes, ndofs);
}
// }}}
// addState {{{
void KeyFramer::sKeyFramer::addState() {
  arr st(ndofs);
  st.setZero();
  addState(st);
}
// }}}
// setState {{{
void KeyFramer::sKeyFramer::setState(uint b, arr st, uint f) {
  CHECK(b < nbodies, "Invalid body index.");
  CHECK(dofs(b) == st.N, "Wrong number of dofs.");
  CHECK(f <= nframes, "Frame number out of bounds.");
  //for(uint i = 0; i < st.N; i++)
    //s->state(f, s->cumdofs(b)+i) = st(i);
  // TODO does this work?
  state[f]().replace(cumdofs(b), st.N, st);
}
// }}}
// setState {{{
void KeyFramer::sKeyFramer::setState(String n, arr st, uint f) {
  int b = names.findValue(n);
  CHECK(b >= 0, "Invalid name.");
  setState(b, st, f);
}
// }}}
// setState {{{
void KeyFramer::sKeyFramer::setState(arr st, uint f) {
  CHECK(st.N == ndofs, "Wrong number of dofs.");
  CHECK(f < nframes, "Frame number out of bounds.");
  state[f]() = st;
}
// }}}

// setupWindows {{{
void KeyFramer::sKeyFramer::setupWindows(MT::Array<arr> &wins, uint wlen) {
  //uint nwins = getNWindows(wlen);
  uint nwins = nframes>=wlen? nframes-wlen+1: 0;

  CHECK(wlen>0, "Window length has to be positive.");
  CHECK(nwins>0, "Not enough frames to have at least one whole window.");

  wins.resize(nwins);
  for(uint wi = 0; wi < nwins; wi++)
    wins(wi).referToSubRange(state, wi, wi+wlen-1);
}
// }}}

// ============================================================================
// KeyFramer
//

KeyFramer::KeyFramer(ors::KinematicWorld &kw, G4Data &g4d) {
  s = new sKeyFramer(&kw, &g4d);
}

KeyFramer::~KeyFramer() {
  delete s;
}

// updateOrs {{{
void KeyFramer::updateOrs(uint f) {
  arr xPos, xQuat;
  for(auto &b: s->kw->bodies) {
    xPos = s->g4d->query("pos", b->name, f);
    xQuat = s->g4d->query("quat", b->name, f);
    xPos.flatten();
    xQuat.flatten();

    b->X.pos.set(xPos.p);
    b->X.rot.set(xQuat.p);
  }
  s->kw->calc_fwdPropagateFrames();
  //s->kw.calcShapeFramesFromBodies(); TODO which one?
  s->kw->gl().text.clear() << "frame " << f << "/" << s->g4d->getNumFrames();
  s->kw->gl().update(NULL, false);
}
// }}}

// getCorrPCA {{{
arr KeyFramer::getCorrPCA(uint b1, uint b2, uint wlen, uint npc) {
  // TODO this still has to be fixed, in terms of the wlen offset
  //HALT("STILL HAVE TO FIX WLEN OFFSET")
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");
  CHECK(npc > 0 && npc <= 3, "Number of principal components must be positive but <=4.");

  MT::Array<arr> wins;
  s->setupWindows(wins, wlen);

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(s->nframes, npc);
  corr = -1;

  arr x, y, xx, yy, xy;
  arr sx, sy, sxx, syy, sxy;
  arr w, t;
  //double sx, sy, sxx, syy, sxy;
  double dwlen = wlen;
  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  corr.subRange(0, ff - 1).setZero();
  corr.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;

    x = wins(wi).cols(cumdofs1, cumdofs1+dofs);
    y = wins(wi).cols(cumdofs2, cumdofs2+dofs);

    // run pca on x and then y
    pca(x, t, w, x, npc);
    y = y * w;

    xx = x % x;
    yy = y % y;
    xy = x % y;

    sx = sum(x, 0);
    sy = sum(y, 0);
    sxx = sum(xx, 0);
    syy = sum(yy, 0);
    sxy = sum(xy, 0);

    corr[fi]() = (dwlen*sxy - sx % sy) / 
                  sqrt(
                      (dwlen*sxx - sx % sx) %
                      (dwlen*syy - sy % sy)
                  );

    //sx = sum(x);
    //sy = sum(y);
    //sxx = (~x*x).elem(0); // or even sum(elemWiseProd(x, x));
    //sxy = (~x*y).elem(0);
    //syy = (~y*y).elem(0);

    //corr(fi) = (sxy*wlen - sx*sy) / sqrt(fabs((sxx*wlen - sx*sx)*(syy*wlen - sy*sy)));
  }

  return corr;
}
// }}}
// getCorrPCA {{{
arr KeyFramer::getCorrPCA(const String &n1, const String &n2, uint wlen, uint npc) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getCorrPCA(b1, b2, wlen, npc);
}
// }}}
// getCorr {{{
arr KeyFramer::getCorr(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  MT::Array<arr> wins;
  s->setupWindows(wins, wlen);
  uint nwins = wins.N;

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(s->nframes, dofs);

  arr x, y, xx, yy, xy;
  arr sx, sy, sxx, syy, sxy;
  double dwlen = wlen;
  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  corr.subRange(0, ff - 1).setZero();
  corr.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;

    x = wins(wi).cols(cumdofs1, cumdofs1+dofs);
    y = wins(wi).cols(cumdofs2, cumdofs2+dofs);
    xx = x % x;
    yy = y % y;
    xy = x % y;

    sx = sum(x, 0);
    sy = sum(y, 0);
    sxx = sum(xx, 0);
    syy = sum(yy, 0);
    sxy = sum(xy, 0);

    //t1 = elemWiseProd(sx, sy);
    //t2 = elemWiseProd(sx, sx);
    //t3 = elemWiseProd(sy, sy);
    //t4a = dwlen*sxx - t2;
    //t4b = dwlen*syy - t3;
    //t4 = elemWiseProd(t4a, t4b);
    //t5 = sqrt(t4);
    //t6a = dwlen*sxy - t1;
    //t6 = elemWiseDiv(t6a, t5);
    //corr[wi]() = t6;
    corr[fi]() = (dwlen * sxy - sx % sy) /
                  sqrt(
                      (dwlen * sxx - sx % sx) %
                      (dwlen * syy - sy % sy
                  ));
  }

  return corr;
}
// }}}
// getCorr {{{
arr KeyFramer::getCorr(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getCorr(b1, b2, wlen);
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
  if(!force && s->g4d->hasBAM(typeVar)) {
    cout << " * " << typeVar << " already computed (force = 0). Skipping." << endl;
    return;
  }

  arr x, y, win, m;

  x = s->g4d->query(type);
  y.resize(x.d0, x.d1);
  y.setZero();

  uint ff = wlen / 2;
  uint ft = s->g4d->getNumFrames() - ff;
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

  s->g4d->appendBam(typeVar, y);
}
// }}}
// computeES {{{
void KeyFramer::computeES(const StringA &types, double alpha, bool force) {
  for(const String &type: types)
    computeES(type, alpha, force);
}
// }}}
// computeES {{{
void KeyFramer::computeES(const String &type, double alpha, bool force) {
  CHECK(alpha >= 0. && alpha <= 1., "Parameter alpha (" << alpha << ") must be 0 <= alpha <= 1.");
  String typeES = STRING(type << "ES");
  cout << " * computing " << typeES << endl;
  if(!force && s->g4d->hasBAM(typeES)) {
    cout << " * " << typeES << " already computed (force = 0). Skipping." <<
      endl;
    return;
  }
  uint numS = s->g4d->getNumSensors();
  uint numF = s->g4d->getNumFrames();
  uint numD = s->g4d->getNumDim(type);
  double beta = 1-alpha;

  arr x, y, es;
  x = s->g4d->query(type);
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

  s->g4d->appendBam(typeES, y);
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
  if(!force && s->g4d->hasBAM(typeSmooth)) {
    cout << " * " << typeSmooth << " already computed (force = 0). Skipping." <<
      endl;
    return;
  }
  uint numS = s->g4d->getNumSensors();
  uint numF = s->g4d->getNumFrames();
  uint numD = s->g4d->getNumDim(type);
  double beta = 1-alpha;

  arr x, y, es;
  x = s->g4d->query(type);
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

  s->g4d->appendBam(typeSmooth, y);
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
  if(!force && s->g4d->hasBAM(typeSpline)) {
    cout << " * " << typeSpline << " already computed (force = 0). Skipping." <<
      endl;
    return;
  }
  uint numS = s->g4d->getNumSensors();
  uint numF = s->g4d->getNumFrames();
  uint numD = s->g4d->getNumDim(type);

  arr x, y, tx, ty;

  x = s->g4d->query(type);
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

  s->g4d->appendBam(typeSpline, y);
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
  if(!force && s->g4d->hasBAM(typeSpeed)) {
    cout << " * " << typeSpeed << " already computed (force = 0). Skipping." << endl;
    return;
  }

  arr x, y;

  x = s->g4d->query(type);
  y.resize(x.d0, x.d1);
  y.setZero();

  uint numS = s->g4d->getNumSensors();
  uint numF = s->g4d->getNumFrames();
  for(uint i = 0; i < numS; i++) {
    for(uint f = 1; f < numF; f++)
      // speed in cm/s (measurements in 100cm, and at 120/s)
      y(i, f) = 12000. * sqrt(sumOfSqr(x[i][f] - x[i][f-1]));
  }

  s->g4d->appendBam(typeSpeed, y);
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
  if(!force && s->g4d->hasBAM(typeGP)) {
    cout << " * " << typeGP << " already computed (force = 0). Skipping." << endl;
    return;
  }
  uint numS = s->g4d->getNumSensors();
  uint numF = s->g4d->getNumFrames();
  uint numD = s->g4d->getNumDim(type);

  arr x, y;
  x = s->g4d->query(type);
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

  s->g4d->appendBam(typeGP, y);
}
// }}}
// computeDPos {{{
void KeyFramer::computeDPos(const String &b, bool force) {
  String typeDPos;
  typeDPos << b << "_dPos";
  cout << " * computing " << typeDPos << endl;
  if(!force && s->g4d->hasBAM(typeDPos)) {
    cout << " * " << typeDPos << " already computed (force = 0). Skipping." << endl;
    return;
  }
  uint numS = s->g4d->getNumSensors();
  uint numF = s->g4d->getNumFrames();
  uint numD = s->g4d->getNumDim("pos");

  arr y(numS, numF, numD);
  y.setZero();

  arr posX, quatX, posY;
  posX = s->g4d->query("pos", b);
  posY = s->g4d->query("pos");
  quatX = s->g4d->query("quat", b);
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

  s->g4d->appendBam(typeDPos, y);
}
// }}}
// computeDQuat {{{
void KeyFramer::computeDQuat(const String &b, bool force) {
  String typeDQuat;
  typeDQuat << b << "_dQuat";
  cout << " * computing " << typeDQuat << endl;
  if(!force && s->g4d->hasBAM(typeDQuat)) {
    cout << " * " << typeDQuat << " already computed (force = 0). Skipping." << endl;
    return;
  }
  uint numS = s->g4d->getNumSensors();
  uint numF = s->g4d->getNumFrames();
  uint numD = s->g4d->getNumDim("quat");

  arr y(numS, numF, numD);
  y.setZero();

  arr quatX, quatY;
  quatX = s->g4d->query("quat", b);
  quatY = s->g4d->query("quat");
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

  s->g4d->appendBam(typeDQuat, y);
}
// }}}

// getState {{{
arr KeyFramer::getState(uint b) {
  uint dofs = s->dofs(b);
  uint cumdofs = s->cumdofs(b);
  return s->state.cols(cumdofs, cumdofs + dofs);
}
// }}}
// getState {{{
arr KeyFramer::getState(const String &n) {
  int b = s->names.findValue(n);
  CHECK(b >= 0, "Invalid name.");

  return getState(b);
}
// }}}
// getStateVar {{{
arr KeyFramer::getStateVar(uint b, uint wlen) {
  arr var(s->nframes);
  arr win, m;
  arr state = getState(b);

  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  var.subRange(0, ff - 1).setZero();
  var.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;
    win.referToSubRange(state, wi, wi + wlen - 1);
    m = sum(win, 0) / (double)win.d0;
    m = ~repmat(m, 1, win.d0);
    var(fi) = sumOfSqr(win - m); // TODO is this right?
  }
  return var / (double) wlen;
}
// }}}
// getStateVar {{{
arr KeyFramer::getStateVar(const String &n, uint wlen) {
  int b = s->names.findValue(n);
  CHECK(b >= 0, "Invalid name.");

  return getStateVar(b, wlen);
}
// }}}
// getStateSpeed {{{
arr KeyFramer::getStateSpeed(uint b) {
  arr speed(s->nframes);
  arr state = getState(b);

  speed(0) = 0;
  for(uint t = 1; t < s->nframes; t++)
    speed(t) = 120. * 100. * sqrt(sumOfSqr((state[t] - state[t-1])));
  return speed;
}
// }}}
// getStateSpeed {{{
arr KeyFramer::getStateSpeed(const String &n) {
  int b = s->names.findValue(n);
  CHECK(b >= 0, "Invalid name.");

  return getStateSpeed(b);
}
// }}}

// getAngle {{{
arr KeyFramer::getAngle(uint b1, uint b2) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr angle(s->nframes);

  arr s1, s2;
  s1 = s->state.cols(cumdofs1, cumdofs1+dofs);
  s2 = s->state.cols(cumdofs2, cumdofs2+dofs);
  ors::Quaternion q1, q2, q;
  for(uint f = 0; f < s->nframes; f++) {
    q1.set(s1[f].p);
    q2.set(s2[f].p);
    q = q1 / q2;
    angle(f) = q.getRad();
    if(angle(f) > M_PI)
      angle(f) = 2*M_PI - angle(f);
  }

  return angle;
}
// }}}
// getAngle {{{
arr KeyFramer::getAngle(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getAngle(b1, b2);
}
// }}}
// getAngleVar {{{
arr KeyFramer::getAngleVar(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr var(s->nframes);
  arr win;
  arr angle = getAngle(b1, b2);

  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  var.subRange(0, ff - 1).setZero();
  var.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;
    win.referToSubRange(angle, wi, wi + wlen - 1);
    var(fi) = sumOfSqr(win - sum(win) / win.N);
  }

  return var / (double)wlen;
}
// }}}
// getAngleVar {{{
arr KeyFramer::getAngleVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getAngleVar(b1, b2, wlen);
}
// }}}

// getQuat {{{
arr KeyFramer::getQuat(uint b1, uint b2) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr quat(s->nframes, 4);

  arr s1, s2;
  s1 = s->state.cols(cumdofs1, cumdofs1+dofs);
  s2 = s->state.cols(cumdofs2, cumdofs2+dofs);
  ors::Quaternion q1, q2, q, A;
  for(uint f = 0; f < s->nframes; f++) {
    q1.set(s1[f].p);
    q2.set(s2[f].p);
    if(f == 0)
      A = q1 / q2;
    q = q1 / (A * q2);
    quat[f]() = {q.w, q.x, q.y, q.z};
  }

  return quat;
}
// }}}
// getQuat {{{
arr KeyFramer::getQuat(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getQuat(b1, b2);
}
// }}}
// getQuatVar {{{
arr KeyFramer::getQuatVar(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr var(s->nframes);
  arr win, m;
  arr quat = getQuat(b1, b2);

  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  var.subRange(0, ff - 1).setZero();
  var.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;
    win.referToSubRange(quat, wi, wi + wlen - 1);
    m = sum(win, 0) / (double)win.d0;
    m = ~repmat(m, 1, win.d0);
    var(fi) = sumOfSqr(win - m);
  }

  return var / (double) wlen;
}
// }}}
// getQuatVar {{{
arr KeyFramer::getQuatVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getQuatVar(b1, b2, wlen);
}
// }}}

// getPos {{{
arr KeyFramer::getPos(uint b1, uint b2) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr pos(s->nframes, 3);

  arr s1, s2, s3;
  s1 = s->state.cols(cumdofs1+3, cumdofs1+dofs);
  s2 = s->state.cols(cumdofs2+3, cumdofs2+dofs);
  s3 = s->state.cols(cumdofs1, cumdofs1+3);
  ors::Vector v1, v2, v, A;
  ors::Quaternion q1;
  for(uint f = 0; f < s->nframes; f++) {
    v1.set(s1[f].p);
    v2.set(s2[f].p);
    q1.set(s3[f].p);
    
    if(f == 0)
      A = q1 * (v2 - v1);
    v = q1 * (v2 - v1) - A;
    pos[f]() = {v.x, v.y, v.z};
  }

  return pos;
}
// }}}
// getPos {{{
arr KeyFramer::getPos(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPos(b1, b2);
}
// }}}
// getPosVar {{{
arr KeyFramer::getPosVar(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr var(s->nframes);
  arr win, m;
  arr pos = getPos(b1, b2);

  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  var.subRange(0, ff - 1).setZero();
  var.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;
    win.referToSubRange(pos, wi, wi + wlen - 1);
    m = sum(win, 0) / (double)win.d0;
    m = ~repmat(m, 1, win.d0);
    var(fi) = sumOfSqr(win - m);
  }

  return var / (double) wlen;
}
// }}}
// getPosVar {{{
arr KeyFramer::getPosVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPosVar(b1, b2, wlen);
}
// }}}

// getDiff {{{
arr KeyFramer::getDiff(uint b1, uint b2) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr s1, s2;
  s1 = s->state.cols(cumdofs1, cumdofs1+dofs);
  s2 = s->state.cols(cumdofs2, cumdofs2+dofs);

  return s2 - s1;
}
// }}}
// getDiff {{{
arr KeyFramer::getDiff(const String &n1, const String &n2) {
  
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getDiff(b1, b2);
}
// }}}
// getDiffVar {{{
arr KeyFramer::getDiffVar(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr var(s->nframes);
  arr win, m;
  arr diff = getDiff(b1, b2);

  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  var.subRange(0, ff - 1).setZero();
  var.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;
    win.referToSubRange(diff, wi, wi + wlen - 1);
    m = sum(win, 0) / (double)win.d0;
    m = ~repmat(m, 1, win.d0);
    var(fi) = sumOfSqr(win - m);
  }

  return var / (double)wlen;
}
// }}}
// getDiffVar {{{
arr KeyFramer::getDiffVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getDiffVar(b1, b2, wlen);
}
// }}}

// getPosLen {{{
arr KeyFramer::getPosLen(uint b1, uint b2) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr posDiff = getDiff(b1, b2);
  arr posLen(s->nframes);
  for(uint f = 0; f < s->nframes; f++)
    posLen(f) = ors::Vector(posDiff[f]).length();

  return posLen;
}
// }}}
// getPosLen {{{
arr KeyFramer::getPosLen(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPosLen(b1, b2);
}
// }}}
// getPosLenVar {{{
arr KeyFramer::getPosLenVar(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr var(s->nframes);
  arr win;
  arr posLen = getPosLen(b1, b2);

  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  var.subRange(0, ff - 1).setZero();
  var.subRange(ft, -1).setZero();
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi - ff;
    win.referToSubRange(posLen, wi, wi + wlen - 1);
    var(fi) = sumOfSqr(win - (sum(win)/win.N));
  }

  return var / (double) wlen;
}
// }}}
// getPosLenVar {{{
arr KeyFramer::getPosLenVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPosLenVar(b1, b2, wlen);
}
// }}}

// getTransfVar {{{
arr KeyFramer::getTransfVar(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr var(s->nframes);
  var.setZero();

  double l;
  arr win, mean;
  arr transf = getDiff(b1, b2);
  uint ff = wlen / 2;
  uint ft = s->nframes - ff;
  for(uint fi = ff; fi < ft; fi++) {
    uint wi = fi  - ff;
    win.referToSubRange(transf, wi, wi + wlen - 1);
    mean = sum(win, 0) / (double)win.d0;
    for(uint t = 0; t < win.d0; t++) {
      l = length(win[t] - mean);
      var(fi) += l*l;
    }
  }

  return var;
}
// }}}
// getTransfVar {{{
arr KeyFramer::getTransfVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getTransfVar(b1, b2, wlen);
}
// }}}

// getKeyFrames {{{
KeyFrameL KeyFramer::getKeyFrames(const uintA &vit) {
  KeyFrameL keyframes;
  KeyFrame *kf;

  bool kf_flag = false;
  for(uint f = 0; f < vit.d0; f++) {
    if(!kf_flag && vit(f) > .5) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = true;
    }
    else if(kf_flag && vit(f) < .5) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = false;
    }
  }
  return keyframes;
}
// }}}
// saveKeyFrameScreens {{{
void KeyFramer::saveKeyFrameScreens(const KeyFrameL &keyframes, uint df) {
  uint f;
  uint h, w;
  byteA img1, img2, img3;
  for(KeyFrame *kf: keyframes) {
    //cout << *kf;
    f = kf->getFrame();

    if(f < df || f+df >= s->g4d->getNumFrames())
      continue;

    // saving keyframe image
    updateOrs(f-df);
    s->kw->gl().text.clear() <<"frame " <<f-df << endl;
    s->kw->gl().update(NULL, true);
    flip_image(s->kw->gl().captureImage);
    byteA img1 = s->kw->gl().captureImage;

    updateOrs(f);
    s->kw->gl().text.clear() <<"frame " <<f << endl;
    s->kw->gl().update(NULL, true);
    flip_image(s->kw->gl().captureImage);
    byteA img2 = s->kw->gl().captureImage;

    updateOrs(f+df);
    s->kw->gl().text.clear() <<"frame " <<f+df << endl;
    s->kw->gl().update(NULL, true);
    flip_image(s->kw->gl().captureImage);
    byteA img3 = s->kw->gl().captureImage;

    h = img1.d0;
    w = 3*img1.d1;
    byteA comp(h, w, 3);

    for(uint i = 0; i < h; i++) {
      memcpy(comp[i]().p    , img1[i]().p, w);
      memcpy(comp[i]().p+w  , img2[i]().p, w);
      memcpy(comp[i]().p+2*w, img3[i]().p, w);
    }

    char ss[10];
    sprintf(ss, "%05d", f);
    write_ppm(comp, STRING("z.keyframe."<<ss<<".ppm"));
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
  //arr pAVar = getStateVar(bp1, wlen);
  //arr qAVar = getStateVar(bo1, wlen);
  //arr dpVar = getPosVar(b1, b2, wlen);
  //arr dqVar = getQuatVar(bo1, bo2, wlen);
  arr pAVar = s->g4d->query("posVar", bA);
  arr qAVar = s->g4d->query("quatVar", bA);
  arr dpVar = s->g4d->query(bA_posDeltaVar, bB);
  arr dqVar = s->g4d->query(bA_quatDeltaVar, bB);
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
  cout << "pi: " << pi << endl;
  cout << "log(pi): " << log(pi) << endl;
  cout << "rho_z[0]: " << rho_z[0] << endl;
  cout << "log(rho_z[0]): " << log(rho_z[0]) << endl;

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
  //arr pAVar = getStateVar(bp1, wlen);
  //arr pBVar = getStateVar(bp2, wlen);
  //arr qAVar = getStateVar(bo1, wlen);
  //arr qBVar = getStateVar(bo2, wlen);
  //arr dp = getPos(b1, b2);
  //arr dq = getQuat(bo1, bo2);
  arr pAVar = s->g4d->query("posVar", bA);
  arr qAVar = s->g4d->query("quatVar", bA);
  arr pBVar = s->g4d->query("posVar", bB);
  arr qBVar = s->g4d->query("quatVar", bB);
  arr dp = s->g4d->query(bA_posDelta, bB);
  arr dq = s->g4d->query(bA_quatDelta, bB);

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
      qy[t]() = my_sum(qzy[t], 1);
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
  cout << "pi: " << pi << endl;
  cout << "log(pi): " << log(pi) << endl;
  cout << "rho_z_cpq[0]: " << rho_z_cpq[0] << endl;
  cout << "log(rho_z_cpq[0]): " << log(rho_z_cpq[0]) << endl;

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
      qy[t]() = my_sum(qzy[t], 1);
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
  cout << "pi: " << pi << endl;
  cout << "log(pi): " << log(pi) << endl;
  cout << "rho_z_cpq[0]: " << rho_z_cpq[0] << endl;
  cout << "log(rho_z_cpq[0]): " << log(rho_z_cpq[0]) << endl;

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
  //arr pVar = getStateVar(bp, wlen);
  //arr qVar = getStateVar(bo, wlen);
  arr pVar = s->g4d->query("posVar", bb);
  arr qVar = s->g4d->query("quatVar", bb);
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
  cout << "pi: " << pi << endl;
  cout << "log(pi): " << log(pi) << endl;
  cout << "rho_z[0]: " << rho_z[0] << endl;
  cout << "log(rho_z[0]): " << log(rho_z[0]) << endl;

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
  arr pSpeedGP = s->g4d->query("posGPSpeed", bb);
  arr qSpeedGP = s->g4d->query("quatGPSpeed", bb);
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
  p_z = my_sum(p_zmm, 0);

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

      qmp[t]() = my_sum(qzmm[t], 1);
      normalizeDist(qmp[t]());

      qmq[t]() = my_sum(qzmm[t], 2);
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
  cout << "pi: " << pi << endl;
  cout << "log(pi): " << log(pi) << endl;
  cout << "rho_z[0]: " << rho_z[0] << endl;
  cout << "log(rho_z[0]): " << log(rho_z[0]) << endl;

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
  arr dpSpeedGP = s->g4d->query(bA_dPosGPSpeed, bB);
  arr dqSpeedGP = s->g4d->query(bA_dQuatGPSpeed, bB);
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
  p_z = my_sum(p_zmm, 0);

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

      qmp[t]() = my_sum(qzmm[t], 1);
      normalizeDist(qmp[t]());

      qmq[t]() = my_sum(qzmm[t], 2);
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
  cout << "pi: " << pi << endl;
  cout << "log(pi): " << log(pi) << endl;
  cout << "rho_z[0]: " << rho_z[0] << endl;
  cout << "log(rho_z[0]): " << log(rho_z[0]) << endl;

  wz[0]() = pi + log(rho_z[0]);
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test
    wz[t]() = log(rho_z[t]);
    for(uint k = 0; k < K; k++) {
      mi =temp[k].maxIndex();
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
// }}}
// EM_c {{{
//#define with_object_emission
//#define update_mu_p
//#define update_sigma_p
//#define update_mu_q
//#define updage_sigma_q
//#define update_mu_dp
//#define update_sigma_dp
//#define update_mu_dq
//#define update_sigma_dq
void KeyFramer::EM_c(KeyValueGraph &kvg, const String &bA, const String &bB) {
  // Computing other BAMS {{{
  double alpha = .3;
  bool force = true;
  force = false;
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
  arr pSpeed = s->g4d->query("posSmoothSpeed", bA);
  arr qSpeed = s->g4d->query("quatSmoothSpeed", bA);
#ifdef with_object_emission
  arr pBSpeed = s->g4d->query("posSmoothSpeed", bB);
  arr qBSpeed = s->g4d->query("quatSmoothSpeed", bB);
#endif
  arr dpSpeed = s->g4d->query(bA_dPosSmoothSpeed, bB);
  arr dqSpeed = s->g4d->query(bA_dQuatSmoothSpeed, bB);
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
  arr rho_z;
  arr rho_m, rho_mp, rho_mq;
  arr rho_mB, rho_mpB, rho_mqB;
  arr rho_r, rho_rdp, rho_rdq;
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
  rho_m.resize(T, M);
  rho_mp.resize(T, Mp);
  rho_mq.resize(T, Mq);
#ifdef with_object_emission
  rho_mB.resize(T, M);
  rho_mpB.resize(T, Mp);
  rho_mqB.resize(T, Mq);
#endif
  rho_r.resize(T, R);
  rho_rdp.resize(T, Rdp);
  rho_rdq.resize(T, Rdq);

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
#ifdef with_object_emission
        rho_mpB(t, k) = ::exp(
            -.5 * MT::sqr(pBSpeed(t) - mu_p(k)) / MT::sqr(sigma_p(k))
          );
        rho_mqB(t, k) = ::exp(
            -.5 * MT::sqr(qBSpeed(t) - mu_q(k)) / MT::sqr(sigma_q(k))
          );
#endif
        rho_rdp(t, k) = ::exp(
            -.5 * MT::sqr(dpSpeed(t) - mu_dp(k)) / MT::sqr(sigma_dp(k))
          );
        rho_rdq(t, k) = ::exp(
            -.5 * MT::sqr(dqSpeed(t) - mu_dq(k)) / MT::sqr(sigma_dq(k))
          );
      }

      for(uint m = 0; m < M; m++)
        rho_m(t, m) = sum(phi_mpq[m] % (rho_mp[t] ^ rho_mq[t]));
      for(uint r = 0; r < R; r++)
        rho_r(t, r) = sum(phi_rpq[r] % (rho_rdp[t] ^ rho_rdq[t]));
#ifdef with_object_emission
      for(uint m = 0; m < M; m++)
        rho_mB(t, m) = sum(phi_mpq[m] % (rho_mpB[t] ^ rho_mqB[t]));
      arr temp; temp.resize(TUP(M, M, R));
      tensorEquation(temp, rho_m[t] ^ rho_mB[t], {0, 1}, rho_r[t], {2}, 0);
      for(uint k = 0; k < K; k++)
        rho_z(t, k) = sum(phi_zmr[k] % temp);
#else
      for(uint k = 0; k < K; k++)
        rho_z(t, k) = sum(phi_zmr[k] % (rho_m[t] ^ rho_r[t]));
#endif
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

      //for(uint k = 0; k < K; k++)
        //qzmm[t][k]() = a(t, k) * b(t, k) * p_zmm[k] % ( rho_mp[t] ^ rho_mq[t] );
      //normalizeDist(qzmm[t]());

      //qmp[t]() = my_sum(qzmm[t], 1);
      //normalizeDist(qmp[t]());

      //qmq[t]() = my_sum(qzmm[t], 2);
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
  cout << "pi: " << pi << endl;
  cout << "log(pi): " << log(pi) << endl;
  cout << "rho_z[0]: " << rho_z[0] << endl;
  cout << "log(rho_z[0]): " << log(rho_z[0]) << endl;

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
  
  //plot = new KeyValueGraph();
  //plot->append("title", new String("Full System: Viterbi"));
  //plot->append("dataid", new bool(true));
  //plot->append("autolegend", new bool(true));
  //plot->append("stream", new double(.75));
  //plot->append("ymin", new double(-.1));
  //plot->append("ymax", new double(1.1));
  //plot->append("data", new String("vit"));
  //kvg.append("plot", plot);

  //plot = new KeyValueGraph();
  //plot->append("title", new String("p, q"));
  //plot->append("dataid", new bool(true));
  //plot->append("autolegend", new bool(true));
  //plot->append("stream", new double(.75));
  //plot->append("data", new String("pSpeed"));
  //plot->append("data", new String("qSpeed"));
  //kvg.append("plot", plot);

#ifdef with_object_emission
  kvg.append("data", "pBSpeed", new arr(pBSpeed));
  kvg.append("data", "qBSpeed", new arr(qBSpeed));

  //plot = new KeyValueGraph();
  //plot->append("title", new String("pBSpeed, qBSpeed"));
  //plot->append("dataid", new bool(true));
  //plot->append("autolegend", new bool(true));
  //plot->append("stream", new double(.75));
  //plot->append("data", new String("pBSpeed"));
  //plot->append("data", new String("qBSpeed"));
  //kvg.append("plot", plot);
#endif

  //plot = new KeyValueGraph();
  //plot->append("title", new String("dp, dq"));
  //plot->append("dataid", new bool(true));
  //plot->append("autolegend", new bool(true));
  //plot->append("stream", new double(.75));
  //plot->append("data", new String("dpSpeed"));
  //plot->append("data", new String("dqSpeed"));
  //kvg.append("plot", plot);
  
  // TODO remove this, and uncomment the previous
  force = true;
  computeES(STRING("pos"), alpha, force);
  computeSmooth(STRING("pos"), alpha, force);
  computeSpeed(STRINGS("pos", "posES", "posSmooth"), force);
  arr pSpeed_orig = s->g4d->query("posSpeed", bA);
  arr pSpeed_smooth = s->g4d->query("posSmoothSpeed", bA);
  arr pSpeed_es = s->g4d->query("posESSpeed", bA);
  kvg.append("data", "pSpeed_orig", new arr(pSpeed_orig));
  kvg.append("data", "pSpeed_smooth", new arr(pSpeed_smooth));
  kvg.append("data", "pSpeed_es", new arr(pSpeed_es));
  plot = new KeyValueGraph();
  plot->append("title", new String("Smoothing"));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(.75));
  plot->append("data", new String("pSpeed_orig"));
  plot->append("data", new String("pSpeed_smooth"));
  plot->append("data", new String("pSpeed_es"));
  kvg.append("plot", plot);
  // }}}
}
// }}}

// playScene {{{
void KeyFramer::playScene(const String &b) {
  KeyValueGraph kvg;
  arrL vits;
  for(String bO: s->g4d->getNames()) {
    if(s->g4d->isObject(bO)) {
      kvg.clear();
      EM_c(kvg, b, bO);
      vits.append(kvg.getItem("data", "vit")->value<arr>());
    }
  }

  uint numF = vits(0)->N;
  for(uint f = 0; f < numF; f++) {
    uint i = 0;
    for(String bO: s->g4d->getNames()) {
      if(s->g4d->isObject(bO)) {
        ors::Body *body = s->kw->getBodyByName(bO);
        for(ors::Shape *shape: body->shapes) {
          if(vits(i)->operator()(f)) {
            shape->color[0] = 1;
            shape->color[1] = .5;
            shape->color[2] = .5;
          }
          else {
            shape->color[0] = 1;
            shape->color[1] = 1;
            shape->color[2] = 1;
          }
        }
        i++;
      }
    }
    updateOrs(f);
  }
}
// }}}

