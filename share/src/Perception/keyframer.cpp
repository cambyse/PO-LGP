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

//uint KeyFramer::getNBodies() {
  //return s->nbodies;
//}

//uint KeyFramer::getCumNDofs(uint b) {
  //CHECK(b < s->nbodies, "Body index out of bounds.");
  //return s->cumdofs(b);
//}

//uint KeyFramer::getNDofs(uint b) {
  //CHECK(b < s->nbodies, "Body index out of bounds.");
  //return s->dofs(b);
//}

//uint KeyFramer::getNDofs() {
  //return s->ndofs;
//}


//uint KeyFramer::getNFrames() {
  //return s->nframes;
//}

//uint KeyFramer::getNWindows(uint wlen) {
  //return s->nframes>=wlen? s->nframes-wlen+1: 0;
//}

//arr KeyFramer::getState() {
  //return s->state;
//}

//arr KeyFramer::getState(uint f) {
  //CHECK(f < s->nframes, "Frame number out of bounds.");
  //return s->state.sub(f, f, 0, -1).resize(s->ndofs); // TODO can this be optimized?
//}

//arr KeyFramer::getState(uint f, uint b) {
  //CHECK(f < s->nframes, "Frame number out of bounds.");
  //CHECK(b < s->nbodies, "Body index out of bounds.");
  //uint dofs = s->dofs(b);
  //uint cumdofs = s->cumdofs(b);
  //return s->state.sub(f, f, cumdofs, cumdofs+dofs-1).resize(dofs); // TODO can this be optimized?
//}

/*
arr KeyFramer::getWindow(uint f) {
  CHECK(f < s->nframes, "Frame number out of bounds.");
  return s->state.sub(f, f+s->lwin-1, 0, -1);
}

arr KeyFramer::getWindow(uint f, uint b) {
  CHECK(f < s->nframes, "Frame number out of bounds.");
  CHECK(b < s->nbodies, "Body index out of bounds.");
  uint dofs = s->dofs(b);
  uint cumdofs = s->cumdofs(b);
  return s->state.sub(f, f+s->lwin-1, cumdofs, cumdofs+dofs-1);
}
*/

void KeyFramer::updateOrs(uint f) {
  arr x;
  for(auto &b: s->kw->bodies) {
    x = s->g4d->query(f, b->name);
    x.flatten();
    CHECK(length(x)!=0, "Why isn't interpolation on?");

    b->X.pos.set(x(0), x(1), x(2));
    b->X.rot.set(x(3), x(4), x(5), x(6));

    //s->kw->computeProxies();
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

/*
MT::Array<arr> KeyFramer::getCorrEnsemble(uint b1, uint b2, uintA &wlens, bool pca) {
  // NB: here nwins is not the same number of windows as in the other methods.
  // other methods: nwins = number of windows of a certain size throughout the
  // whole stream
  // this method: nwins = number of different window sizes.
  uint nwins = wlens.N;

  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");
  CHECK(nwins>0, "Specify at least one window size.");

  MT::Array<arr> corr(s->nframes);
  for(uint wi = 0; wi < nwins; wi++)
    corr(wi) = getCorr(b1, b2, wlens(wi), pca);

  return corr;
}

MT::Array<arr> KeyFramer::getCorrEnsemble(const String &n1, const String &n2, uintA &wlens, bool pca) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getCorrEnsemble(b1, b2, wlens, pca);
}
*/

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
  var.setZero();

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

  return var;
}

arr KeyFramer::getAngleVar(const String &n1, const String &n2, uint wlen) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return getAngleVar(b1, b2, wlen);
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
  var.setZero();

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
  var.setZero();

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

  return var;
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

ProxyL KeyFramer::getProxies(uint b1, uint b2) {
  ProxyL proxies;
  bool found;
  for(uint f = 0; f < s->nframes; f++) {
    updateOrs(f);
    found = false;
    for(auto &proxy: s->kw->proxies) {
      if(proxy->a == (int)b1 && proxy->b == (int)b2) {
        found = true;
        proxies.append(new ors::Proxy(*proxy));
        break;
      }
    }
    if(!found)
      proxies.append(NULL);
  }

  return proxies;
}

ProxyL KeyFramer::getProxies(const String &n1, const String &n2) {
  ors::Body *b1 = s->kw->getBodyByName(n1);
  CHECK(b1, "Invalid name.");
  ors::Body *b2 = s->kw->getBodyByName(n2);
  CHECK(b2, "Invalid name.");

  return getProxies(b1->index, b2->index);
}

void KeyFramer::calcProxies(uint b1, uint b2) {
  NIY;
}

arr KeyFramer::getDists(uint b1, uint b2) {
  arr dist;
  for(auto &shape: s->kw->bodies(b1)->shapes)
    if(shape->type != ors::markerST)
      shape->cont = true;
  for(auto &shape: s->kw->bodies(b2)->shapes)
    if(shape->type != ors::markerST)
      shape->cont = true;
  s->kw->swift().setCutoff(2.);
  s->kw->swift().initActivations();
  for(uint f = 0; f < s->nframes; f++) {
    updateOrs(f);
    ors::Proxy *minProxy = NULL;
    for(auto &proxy: s->kw->proxies) {
      if( s->kw->shapes(proxy->a)->body->index == b1
          && s->kw->shapes(proxy->b)->body->index == b2
          && (!minProxy || proxy->d < minProxy->d)) {
        minProxy = proxy;
      }
    }
    CHECK(minProxy, "Something is wrong here. Call Andrea.");
    dist.append(minProxy->d);
  }
  for(auto &shape: s->kw->bodies(b1)->shapes)
    shape->cont = false;
  for(auto &shape: s->kw->bodies(b2)->shapes)
    shape->cont = false;
  return dist;
}

arr KeyFramer::getDists(const String &n1, const String &n2) {
  ors::Body *b1 = s->kw->getBodyByName(n1);
  CHECK(b1, "Invalid name.");
  ors::Body *b2 = s->kw->getBodyByName(n2);
  CHECK(b2, "Invalid name.");

  return getDists(b1->index, b2->index);
}

void KeyFramer::calcProxies(const String &n1, const String &n2) {
  ors::Body *b1 = s->kw->getBodyByName(n1);
  CHECK(b1, "Invalid name.");
  ors::Body *b2 = s->kw->getBodyByName(n2);
  CHECK(b2, "Invalid name.");

  calcProxies(b1->index, b2->index);
}

void KeyFramer::clearProxies() {
  listDelete(s->kw->proxies);
}

KeyFrameL KeyFramer::getKeyFrames(const uintA &vit) {
  KeyFrameL keyframes;
  KeyFrame *kf;

  bool kf_flag = false;
  for(uint f = 0; f < vit.d0; f++) {
    if(!kf_flag && vit(f, 1) > .5) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = true;
    }
    else if(kf_flag && vit(f, 1) < .5) {
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

void KeyFramer::EM(uintA &vit, const arr &c, const arr &v) {
  arr p = { 1, 0 };
  arr A = { .5, .5, .5, .5 };
  A.reshape(2, 2);

  arr c_mu = { 0, 1 };
  arr c_sigma = { 1, .3 };

  arr B = { .9, 1, .1, 0 };
  B.reshape(2, 2);

  arr v_mu = { 0, 0 };
  arr v_sigma = { .7, .3 };

  arrL theta;
  theta.append(&p);
  theta.append(&A);
  theta.append(&c_mu);
  theta.append(&c_sigma);
  theta.append(&B);
  theta.append(&v_mu);
  theta.append(&v_sigma);

  uint T = c.d0, K = c_mu.N, J = v_mu.N;
  arr rho_z_c(T, K), rho_z_cv(T, K), rho_y_v(T, J), rho_z_cyv(T, J, K);
  arrL rho;
  rho.append(&rho_z_c);
  rho.append(&rho_z_cv);
  rho.append(&rho_y_v);
  rho.append(&rho_z_cyv);

  arr qz(T, K), qzz(T-1, K, K), qzy(T, K, J), qy(T, J);
  arrL ql;
  ql.append(&qz);
  ql.append(&qzz);
  ql.append(&qzy);
  ql.append(&qy);

  for(uint i = 0; i < 50; i++) {
    cout << endl;
    cout << "---------------------------" << endl;
    cout << "step: " << i << endl;
    cout << "p: " << p << endl;
    cout << "A: " << A << endl;
    cout << "c_mu: " << c_mu << endl;
    //cout << "c_sigma: " << c_sigma << endl;
    cout << "B: " << B << endl;
    //cout << "v_mu: " << v_mu << endl;
    cout << "v_sigma: " << v_sigma << endl;

    computeEvidences(rho, c, v, theta);
    Estep(ql, theta, rho);
    Mstep(theta, ql, c, v);
  }

  viterbiZ(vit, theta, rho, c, v);
}

void KeyFramer::viterbiZ(uintA &vit, arrL &theta, arrL &rho, const arr &c, const arr &v) {
  arr p, A, B;
  p.referTo(*theta(0));
  A.referTo(*theta(1));
  B.referTo(*theta(4));

  uint T = c.d0, K = A.d0, J = B.d0;

  arr rho_z_cv;
  rho_z_cv.referTo(*rho(1));

  double m;
  int mi;
  arr wz(T, K), wzind(T, K), temp;
  cout << "p: " << p << endl;
  cout << "log(p): " << log(p) << endl;
  cout << "rho_z_cv[0]: " << rho_z_cv[0] << endl;
  cout << "log(rho_z_cv[0]): " << log(rho_z_cv[0]) << endl;

  wz[0]() = p + log(rho_z_cv[0]);
  // TODO this shouldn't be necessary
  //for(uint k = 0; k < K; k++)
    //wzind(0, k) = k;

  //cout << "wz[0]: " << wz[0] << endl;
  //cout << "repmat: " << ~repmat(wz[0], 1, 2) << endl;
  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test

    wz[t]() = log(rho_z_cv[t]);
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
    //cout << "wzind[" << t << "]: " << wzind[t] << endl;
  }

  vit.resize(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
}

/*
void KeyFramer::viterbiZY(uintA &vit, arrL &theta, arrL &rho, const arr &c, const arr &v) {
  arr p, A, B;
  p.referTo(*theta(0));
  A.referTo(*theta(1));
  B.referTo(*theta(4));

  uint T = c.d0, K = A.d0, J = B.d0;

  arr rho_z_cv, rho_y_v;
  rho_z_cv.referTo(*rho(1));
  rho_y_v.referTo(*rho(2));

  double m;
  int mi;
  arr wz(T, K), wy(T, J), wzind(T, K), wyind(T, K), temp;
  cout << "p: " << p << endl;
  cout << "log(p): " << log(p) << endl;
  cout << "rho_z_cv[0]: " << rho_z_cv[0] << endl;
  cout << "log(rho_z_cv[0]): " << log(rho_z_cv[0]) << endl;

  wy[0]() = log(rho_y_v[0]);
  temp = log(B) + ~repmat(wy[0], 1, K);
  for(uint k = 0; k < K; k++) {
    m = temp(k, 0);
    mi = 0;
    for(uint j = 1; j < J; j++) {
      if(m < temp(k, j)) {
        m = temp(k, j);
        mi = j;
      }
    }
    wz[0]() = log(p) + log(rho_z_cv[0]);
    wyind(0, k) = mi;
  }
  // TODO this shouldn't be necessary
  //for(uint k = 0; k < K; k++)
    //wzind(0, k) = k;

  //cout << "wz[0]: " << wz[0] << endl;
  //cout << "repmat: " << ~repmat(wz[0], 1, 2) << endl;

  for(uint t = 1; t < T; t++) {
    temp = log(A) + ~repmat(wz[t-1], 1, K); // TODO is this necessary? test

    wz[t]() = log(rho_z_cv[t]);
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
    //cout << "wzind[" << t << "]: " << wzind[t] << endl;
  }

  vit.resize(T);
  vit(T-1) = (wz(T-1, 0) > wz(T-1, 1))? 0: 1;
  for(uint t = T-1; t > 0; t--)
    vit(t-1) = wzind(t, vit(t));
}
*/

void KeyFramer::computeEvidences(arrL &rho, const arr &c, const arr &v, const arrL &theta) {
  arr c_mu, c_sigma, B, v_mu, v_sigma;
  c_mu.referTo(*theta(2));
  c_sigma.referTo(*theta(3));
  B.referTo(*theta(4));
  v_mu.referTo(*theta(5));
  v_sigma.referTo(*theta(6));

  arr rho_z_c, rho_z_cv, rho_y_v, rho_z_cyv;
  // TODO correct referrals
  rho_z_c.referTo(*rho(0));
  rho_z_cv.referTo(*rho(1));
  rho_y_v.referTo(*rho(2));
  rho_z_cyv.referTo(*rho(3));
  
  //-- evidences from observations
  uint T = c.d0, K = c_mu.N, J = v_mu.N;
  for(uint t = 0; t < T; t++) {
    for(uint j = 0; j < J; j++)
      rho_y_v(t, j) = ::exp(
          -.5 * MT::sqr(v(t) - v_mu(j)) / (v_sigma(j) * v_sigma(j))
        );
    for(uint k = 0; k < K; k++)
      rho_z_c(t, k) = ::exp(
          -.5 * MT::sqr(c(t) - c_mu(k)) / (c_sigma(k) * c_sigma(k))
          );
    rho_z_cv[t]() = rho_z_c[t] * sum(rho_y_v[t]);
    for(uint k = 0; k < K; k++)
      for(uint j = 0; j < J; j++)
        rho_z_cyv(t, j, k) = rho_z_c(t, k) * B(j, k) * rho_y_v(t, j);
  }
}

void KeyFramer::Estep(arrL &ql, const arrL &theta, const arrL &rho) {
  arr rho_z_c, rho_z_cv, rho_y_v, rho_z_cyv;
  rho_z_c.referTo(*rho(0));
  rho_z_cv.referTo(*rho(1));
  rho_y_v.referTo(*rho(2));
  rho_z_cyv.referTo(*rho(3));

  arr qz, qzz, qzy, qy;
  qz.referTo(*ql(0));
  qzz.referTo(*ql(1));
  qzy.referTo(*ql(2));
  qy.referTo(*ql(3));

  arr p, A, B;
  p.referTo(*theta(0));
  A.referTo(*theta(1));
  B.referTo(*theta(4));

  uint T = qz.d0, K = A.d0, J = B.d0;

  // alpha and beta
  arr a(T, K), b(T, K);
  a[0]() = p;   //initialization of alpha
  b[T-1]() = 1; //initialization of beta
  //--- fwd and bwd iterations:
  for(uint t = 1; t < T; t++) {
    a[t]() =  A * (rho_z_cv[t-1] % a[t-1]); // %=element-wise multiplication, *=inner product
    normalizeDist(a[t]()); //for numerical stability
  }
  for(uint t = T-1; t--; ) {
    b[t]() = ~A * (rho_z_cv[t+1] % b[t+1]);
    normalizeDist(b[t]());
  }

  for(uint t = 0; t < T; t++) {
    qz[t]() = a[t]() % rho_z_cv[t]() % b[t](); // %=element-wise multiplication
    for(uint k = 0; k < K; k++)
      for(uint j = 0; j < J; j++)
        qzy(t, k, j) = a(t, k) * b(t, k) * rho_z_cyv(t, j, k);
    qy[t]() = sum(qzy[t], 0);
  }
  for(uint t = 0; t < T-1; t++)
    for(uint k = 0; k < K; k++)
      for(uint l = 0; l < K; l++)
        qzz(t, k, l) = a(t, l)*rho_z_cv(t, l)*A(k, l)*rho_z_cv(t+1, k)*b(t+1, k);

  for(uint t = 0; t < T; t++) {
    normalizeDist(qz[t]());
    normalizeDist(qzy[t]());
    normalizeDist(qy[t]());
  }
  for(uint t = 0; t < T-1; t++)
    normalizeDist(qzz[t]());
}

void KeyFramer::Mstep(arrL& theta, const arrL &ql, const arr& c, const arr &v){
  arr qz, qzz, qzy, qy;
  qz.referTo(*ql(0));
  qzz.referTo(*ql(1));
  qzy.referTo(*ql(2));
  qy.referTo(*ql(3));

  arr p, A, c_mu, B, v_sigma;
  p.referTo(*theta(0));
  A.referTo(*theta(1));
  c_mu.referTo(*theta(2));
  B.referTo(*theta(4));
  v_sigma.referTo(*theta(6));

  uint T = c.d0, K = A.d0, J = B.d0;
  p = qz[0];

  arr pz(K), pzz(K, K);
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

  c_mu = w*c;
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
  for(uint j = 0; j < J; j++) {
    double n = 0, d = 0;
    for(uint t = 0; t < T; t++) {
      n += qy(t, j) * v(t) * v(t);
      d += qy(t, j);
    }
    v_sigma(j) = sqrt(n / d);
  }

  //cout << "=========================" << endl;
  //cout << "q: " << q[1] << endl;
  //cout << "q_y: " << q_y[1] << endl;
  //cout << "B: " << B[0] << endl;
  //cout << "=========================" << endl;
}

