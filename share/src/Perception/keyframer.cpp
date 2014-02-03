#include "keyframer.h"
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>

struct KeyFramer::sKeyFramer {
  ors::KinematicWorld *kw;
  G4Data *g4d;

  uint nbodies, ndofs;
  uintA dofs, cumdofs;
  StringA names;

  uint nframes;
  arr state;

  double thresh;
  double dist;

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
  init();
  thresh = .95;
  dist = .03;
}

KeyFramer::sKeyFramer::~sKeyFramer() {}

void KeyFramer::sKeyFramer::clear() {
  clearState();
}

void KeyFramer::sKeyFramer::clearState() {
  nbodies = 0;
  ndofs = 0;
  clearFrames();
}

void KeyFramer::sKeyFramer::clearFrames() {
  state.clear();
  nframes = 0;
}

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

      setState(pname, g4d->queryPos(f, name), f);
      setState(oname, g4d->queryQuat(f, name), f);
      setState(name, g4d->query(f, name), f);
    }
  }
}

void KeyFramer::sKeyFramer::addBody(String &name, uint body_ndofs) {
  CHECK(body_ndofs > 0, "Number of Dofs for a body must be positive.");
  CHECK(!names.contains(name), "Body name already exists.");

  nbodies++;
  ndofs += body_ndofs;

  cumdofs.append(nbodies == 1? 0: cumdofs.last()+dofs.last());
  dofs.append(body_ndofs);
  names.append(name);
}

void KeyFramer::sKeyFramer::addState(arr st) {
  CHECK(st.N == ndofs, "Wrong number of dofs.");
  nframes++;
  state.append(st);
  state.reshape(nframes, ndofs);
}

void KeyFramer::sKeyFramer::addState() {
  arr st(ndofs);
  st.setZero();
  addState(st);
}

void KeyFramer::sKeyFramer::setState(uint b, arr st, uint f) {
  CHECK(b < nbodies, "Invalid body index.");
  CHECK(dofs(b) == st.N, "Wrong number of dofs.");
  CHECK(f <= nframes, "Frame number out of bounds.");
  /*
  for(uint i = 0; i < st.N; i++)
    s->state(f, s->cumdofs(b)+i) = st(i);
  */
  // TODO does this work?
  state[f]().replace(cumdofs(b), st.N, st);
}

void KeyFramer::sKeyFramer::setState(String n, arr st, uint f) {
  int b = names.findValue(n);
  CHECK(b >= 0, "Invalid name.");
  setState(b, st, f);
}

void KeyFramer::sKeyFramer::setState(arr st, uint f) {
  CHECK(st.N == ndofs, "Wrong number of dofs.");
  CHECK(f < nframes, "Frame number out of bounds.");
  state[f]() = st;
}

void KeyFramer::sKeyFramer::setupWindows(MT::Array<arr> &wins, uint wlen) {
  //uint nwins = getNWindows(wlen);
  uint nwins = nframes>=wlen? nframes-wlen+1: 0;

  CHECK(wlen>0, "Window length has to be positive.");
  CHECK(nwins>0, "Not enough frames to have at least one whole window.");

  wins.resize(nwins);
  for(uint wi = 0; wi < nwins; wi++)
    wins(wi).referToSubRange(state, wi, wi+wlen-1);
}

// ============================================================================
// KeyFramer
//

KeyFramer::KeyFramer(ors::KinematicWorld &kw, G4Data &g4d) {
  s = new sKeyFramer(&kw, &g4d);
}

KeyFramer::~KeyFramer() {
  delete s;
}

void KeyFramer::updateOrs(uint f) {
  arr x;
  for(auto &b: s->kw->bodies) {
    x = s->g4d->query(f, b->name);
    x.flatten();
    CHECK(length(x)!=0, "Why isn't interpolation on?");

    b->X.pos.set(x(0), x(1), x(2));
    b->X.rot.set(x(3), x(4), x(5), x(6));
  }
  s->kw->calcBodyFramesFromJoints();
  //s->kw.calcShapeFramesFromBodies(); TODO which one?
  s->kw->gl().text.clear() << "frame " << f << "/" << s->g4d->getNumFrames();
  s->kw->gl().update(NULL, false);
}

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

    /*
    sx = sum(x);
    sy = sum(y);
    sxx = (~x*x).elem(0); // or even sum(elemWiseProd(x, x));
    sxy = (~x*y).elem(0);
    syy = (~y*y).elem(0);

    corr(fi) = (sxy*wlen - sx*sy) / sqrt(fabs((sxx*wlen - sx*sx)*(syy*wlen - sy*sy)));
    */
  }

  return corr;
}

arr KeyFramer::getCorrPCA(const String &n1, const String &n2, uint wlen, uint npc) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getCorrPCA(b1, b2, wlen, npc);
}

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

    /*
    t1 = elemWiseProd(sx, sy);
    t2 = elemWiseProd(sx, sx);
    t3 = elemWiseProd(sy, sy);
    t4a = dwlen*sxx - t2;
    t4b = dwlen*syy - t3;
    t4 = elemWiseProd(t4a, t4b);
    t5 = sqrt(t4);
    t6a = dwlen*sxy - t1;
    t6 = elemWiseDiv(t6a, t5);
    corr[wi]() = t6;
    */
    corr[fi]() = (dwlen * sxy - sx % sy) /
                  sqrt(
                      (dwlen * sxx - sx % sx) %
                      (dwlen * syy - sy % sy
                  ));
  }

  return corr;
}

arr KeyFramer::getCorr(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getCorr(b1, b2, wlen);
}

arr KeyFramer::getState(uint b) {
  uint dofs = s->dofs(b);
  uint cumdofs = s->cumdofs(b);
  return s->state.cols(cumdofs, cumdofs + dofs);
}

arr KeyFramer::getState(const String &n) {
  int b = s->names.findValue(n);
  CHECK(b >= 0, "Invalid name.");

  return getState(b);
}

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

arr KeyFramer::getStateVar(const String &n, uint wlen) {
  int b = s->names.findValue(n);
  CHECK(b >= 0, "Invalid name.");

  return getStateVar(b, wlen);
}

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

arr KeyFramer::getAngle(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getAngle(b1, b2);
}

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

arr KeyFramer::getAngleVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getAngleVar(b1, b2, wlen);
}

arr KeyFramer::getQuat(uint b1, uint b2) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr quat(s->nframes, 4);

  arr s1, s2;
  s1 = s->state.cols(cumdofs1, cumdofs1+dofs);
  s2 = s->state.cols(cumdofs2, cumdofs2+dofs);
  ors::Quaternion q1, q2, q;
  ors::Quaternion A;
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

arr KeyFramer::getQuat(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getQuat(b1, b2);
}

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

arr KeyFramer::getQuatVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getQuatVar(b1, b2, wlen);
}

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
  ors::Vector v1, v2, v;
  ors::Quaternion q1;
  for(uint f = 0; f < s->nframes; f++) {
    v1.set(s1[f].p);
    v2.set(s2[f].p);
    q1.set(s3[f].p);
    
    v = q1 * (v2 - v1);
    pos[f]() = {v.x, v.y, v.z};
  }

  return pos;
}

arr KeyFramer::getPos(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPos(b1, b2);
}

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

arr KeyFramer::getPosVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPosVar(b1, b2, wlen);
}

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

arr KeyFramer::getDiff(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getDiff(b1, b2);
}

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

arr KeyFramer::getDiffVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getDiffVar(b1, b2, wlen);
}

arr KeyFramer::getPosLen(uint b1, uint b2) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  arr posDiff = getDiff(b1, b2);
  arr posLen(s->nframes);
  for(uint f = 0; f < s->nframes; f++)
    posLen(f) = ors::Vector(posDiff[f]).length();

  return posLen;
}

arr KeyFramer::getPosLen(const String &n1, const String &n2) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPosLen(b1, b2);
}

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

arr KeyFramer::getPosLenVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getPosLenVar(b1, b2, wlen);
}

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

arr KeyFramer::getTransfVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getTransfVar(b1, b2, wlen);
}

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

void KeyFramer::EM(uintA &vit, const String &b1, const String &b2, uint wlen) {
  // Observations {{{
  String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));

  arr c = getCorrPCA(bp1, bp2, wlen, 1).flatten();
  arr c3d = getCorr(bp1, bp2, wlen);

  arr pD = getPos(b1, b2);
  arr pAVar = getStateVar(bp1, wlen);
  arr pBVar = getStateVar(bp2, wlen);

  arr qD = getQuat(bo1, bo2);
  arr qAVar = getStateVar(bo1, wlen);
  arr qBVar = getStateVar(bo2, wlen);

  arr pVar = getPosVar(b1, b2, wlen);
  arr qVar = getQuatVar(bo1, bo2, wlen);

  // }}}
  // Parameters & other {{{
  uint T, K, J;
  double sigma_small, sigma_big;
  arr pi, A;
  arr c_mu, c_sigma;
  arr B;
  arr p_mu, p_sigma;
  arr q_mu, q_sigma;
  arr rho_z_c, rho_z_cpq, rho_z_cypq, rho_y_p, rho_y_q, rho_y_d;
  arr rho_y_pqd ;
  arr qz, qzz, qzy, qy;
  arr a, b;
  arr pz, pzz;

  pi = { 1, 0 };
  A = { .5, .5,
        .5, .5 };
  A.reshape(2, 2);
  c_mu = { 0, 1 };
  c_sigma = { 1, .2 };
  B = { .6, .01,
        .4, .99 };
  B.reshape(2, 2);
  sigma_small = .3; sigma_big = .7;
  p_mu = {0, 0}; p_sigma = { sigma_big, sigma_small };
  q_mu = {0, 0}; q_sigma = { sigma_big, sigma_small };

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
  // }}}

  arr prev_qz;
  for(uint i = 0; ; i++) {
    // Cout Parameters {{{
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "pi: " << pi << endl;
    cout << "A: " << A << endl;
    cout << "c_mu: " << c_mu << endl;
    //cout << "c_sigma: " << c_sigma << endl;
    cout << "B: " << B << endl;
    //cout << "q_mu: " << q_mu << endl;
    cout << "q_sigma: " << q_sigma << endl;
    //cout << "p_mu: " << p_mu << endl;
    cout << "p_sigma: " << p_sigma << endl;
    // }}}
    // COMPUTE EVIDENCES {{{
    //computeEvidences(rho, obs, theta);
    for(uint t = 0; t < T; t++) {
      for(uint j = 0; j < J; j++) {
        rho_y_p(t, j) = ::exp(
            -.5 * MT::sqr(pVar(t) - p_mu(j)) / MT::sqr(p_sigma(j))
          );
        rho_y_q(t, j) = ::exp(
            -.5 * MT::sqr(qVar(t) - q_mu(j)) / MT::sqr(q_sigma(j))
          );
        //rho_y_d(t, j) = ::exp(
            //-.5 * MT::sqr(qD(t)) / MT::sqr(d_sigma(j))
          //);
      }
    }
    rho_y_pqd = rho_y_p % rho_y_q; // % rho_y_d;
    for(uint t = 0; t < T; t++) {
      for(uint k = 0; k < K; k++)
        rho_z_c(t, k) = ::exp(
            -.5 * MT::sqr(c(t) - c_mu(k)) / (c_sigma(k) * c_sigma(k))
            );
      rho_z_cpq[t]() = rho_z_c[t] % (~B * rho_y_pqd[t]);
      rho_z_cypq[t]() = B % (rho_y_pqd[t] ^ rho_z_c[t]);
    }
    // }}}
    // E-STEP {{{
    //Estep(ql, theta, rho);
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
      // TODO is this right? shouldn't there be a rho_y somewhere?
      qzy[t]() = repmat(a[t] % b[t], 1, J) % ~rho_z_cypq[t];
      qy[t]() = sum(qzy[t], 0);
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
    //Mstep(theta, ql, obs);
    pi = qz[0];

    pz.setZero();
    pzz.setZero();
    for(uint t = 0; t < T-1; t++) {
      pz += qz[t];
      pzz += qzz[t];
    }
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        A(k, l) = pzz(k, l) / pz(l);

    arr w(K,T);
    arr qz_sum = sum(qz, 0);
    arr qzy_sum = sum(qzy, 0).reshape(K, J);

    for(uint t = 0; t < T; t++)
      for(uint k = 0; k < K; k++)
        w(k, t) = qz(t, k) / qz_sum(k);

    //c_mu = w*c;
    //c_sigma = w*(y%y) - mu%mu;

    /*
    arr e = qzy_sum[0];
    cout << "THESE SHOULD BE THE SAME: " << endl;
    cout << "qz_sum(0): " << qz_sum(0) << endl;
    cout << "sum(e): " << sum(e) << endl;
    e = qzy_sum[0] / qz_sum(0);
    cout << "e: " << e << endl;
    cout << "sum(e): " << sum(e) << endl;
    e = qzy_sum[0];
    normalizeDist(e);
    cout << "e: " << e << endl;
    cout << "sum(e): " << sum(e) << endl;
    //normalizeDist(e);
    for(uint j = 0; j < J; j++)
      B(j, 0) = e(j);
    */

    // TODO NB: activating this breaks viterbi.....
    /*
    for(uint j = 0; j < J; j++) {
      double n = 0, d = 0;
      for(uint t = 0; t < T; t++) {
        n += qy(t, j) * q(t) * q(t);
        d += qy(t, j);
      }
      q_sigma(j) = sqrt(n / d);
    }

    for(uint j = 0; j < J; j++) {
      double n = 0, d = 0;
      for(uint t = 0; t < T; t++) {
        n += qy(t, j) * p(t) * p(t);
        d += qy(t, j);
      }
      p_sigma(j) = sqrt(n / d);
    }
    */
    // }}}

    if(i && sumOfSqr(qz-prev_qz) < 1e-10) break;
    prev_qz = qz;
  }
  cout << endl;
  cout << "DONE!" << endl;

  // Viterbi {{{
  //viterbi(vit, theta, rho, c, q);
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

  vit.resize(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
  // }}}
}

