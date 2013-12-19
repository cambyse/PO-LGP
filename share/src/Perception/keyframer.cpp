#include "keyframer.h"
#include <Ors/ors_swift.h>

struct KeyFramer::sKeyFramer {
  ors::KinematicWorld *G;
  G4Data *g4d;

  uint nbodies, ndofs;
  uintA dofs, cumdofs;
  StringA names;

  uint nframes;
  arr state;

  double thresh;
  double dist;

  sKeyFramer(ors::KinematicWorld *_G, G4Data *_g4d);
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

KeyFramer::sKeyFramer::sKeyFramer(ors::KinematicWorld *_G, G4Data *_g4d): G(_G), g4d(_g4d) {
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

KeyFramer::KeyFramer(ors::KinematicWorld &G, G4Data &g4d) {
  s = new sKeyFramer(&G, &g4d);
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
  for(auto &b: s->G->bodies) {
    x = s->g4d->query(f, b->name);
    x.reshape(x.N); // TODO there should be a nicer way.. // resetD doesn't work, probably using it all wrong
    CHECK(length(x)!=0, "Why isn't interpolation on?");

    b->X.pos.set(x(0), x(1), x(2));
    b->X.rot.set(x(3), x(4), x(5), x(6));

    s->G->computeProxies();
  }
  s->G->calcBodyFramesFromJoints();
  //s->G.calcShapeFramesFromBodies(); TODO which one?
  s->G->gl().text.clear() << "frame " << f << "/" << s->g4d->getNumFrames();
}

arr KeyFramer::getCorrPCA(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  MT::Array<arr> wins;
  s->setupWindows(wins, wlen);

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(s->nframes);

  arr x, y, w, t;
  double sx, sy, sxx, syy, sxy;
  for(uint fi = 0; fi < s->nframes; fi++) {
    if(fi < wlen-1) {
      corr(fi) = 0;
      continue;
    }
    uint wi = fi -wlen +1;

    x = wins(wi).cols(cumdofs1, cumdofs1+dofs);
    y = wins(wi).cols(cumdofs2, cumdofs2+dofs);

    // center x // TODO put this into pca..
    t = sum(x, 0);
    for(uint i = 0; i < x.d0; i++)
      x[i]() -= t;

    // run pca on x
    pca(x, t, w, x, 1);

    // apply pca on y
    pca(y, w, y);

    sx = sum(x);
    sy = sum(y);
    sxx = (~x*x).elem(0); // or even sum(elemWiseProd(x, x));
    sxy = (~x*y).elem(0);
    syy = (~y*y).elem(0);

    corr(fi) = (sxy*wlen - sx*sy) / sqrt(fabs((sxx*wlen - sx*sx)*(syy*wlen - sy*sy)));
  }

  return corr;
}

arr KeyFramer::getCorrWOPCA(uint b1, uint b2, uint wlen) {
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
  arr t1, t2, t3, t4a, t4b, t4, t5, t6a, t6;
  double dwlen = wlen;
  for(uint fi = 0; fi < nwins; fi++) {
    if(fi < wlen-1) {
      corr[fi]() = 0;
      continue;
    }
    uint wi = fi -wlen +1;

    x = wins(wi).cols(cumdofs1, cumdofs1+dofs);
    y = wins(wi).cols(cumdofs2, cumdofs2+dofs);
    xx = elemWiseProd(x, x);
    yy = elemWiseProd(y, y);
    xy = elemWiseProd(x, y);

    sx = sum(x, 0);
    sy = sum(y, 0);
    sxx = sum(xx, 0);
    syy = sum(yy, 0);
    sxy = sum(xy, 0);

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
  }

  return corr;
}

arr KeyFramer::getCorr(uint b1, uint b2, uint wlen, bool pca) {
  return pca? getCorrPCA(b1, b2, wlen): getCorrWOPCA(b1, b2, wlen);
}

arr KeyFramer::getCorr(const String &n1, const String &n2, uint wlen, bool pca) {
  int b1 = s->names.findValue(n1);
  CHECK(b1 >= 0, "Invalid name.");
  int b2 = s->names.findValue(n2);
  CHECK(b2 >= 0, "Invalid name.");

  return pca? getCorrPCA(b1, b2, wlen): getCorrWOPCA(b1, b2, wlen);
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

  ors::Quaternion q1, q2, q;
  arr s1, s2;

  s1 = s->state.cols(cumdofs1, cumdofs1+dofs);
  s2 = s->state.cols(cumdofs2, cumdofs2+dofs);
  for(uint f = 0; f < s->nframes; f++) {
    q1.set(s1(f, 0), s1(f, 1), s1(f, 2), s1(f, 3));
    q2.set(s2(f, 0), s2(f, 1), s2(f, 2), s2(f, 3));
    q = q1/q2;
    angle(f) = q.getRad();
    // TODO I really want an angle between 0 and PI
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

  arr t;
  arr angle = getAngle(b1, b2);
  for(uint fi = 0; fi < s->nframes; fi++) {
    if(fi < wlen-1) {
      var(fi) = 0;
      continue;
    }
    t.referToSubRange(angle, fi-wlen+1, fi);
    var(fi) = sumOfSqr(t-mean(t));
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

ProxyL KeyFramer::getProxies(uint b1, uint b2) {
  ProxyL proxies;
  bool found;
  for(uint f = 0; f < s->nframes; f++) {
    updateOrs(f);
    found = false;
    for(auto &proxy: s->G->proxies) {
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
  ors::Body *b1 = s->G->getBodyByName(n1);
  CHECK(b1, "Invalid name.");
  ors::Body *b2 = s->G->getBodyByName(n2);
  CHECK(b2, "Invalid name.");

  return getProxies(b1->index, b2->index);
}

void KeyFramer::calcProxies(uint b1, uint b2) {
  NIY;
}

arr KeyFramer::getDists(uint b1, uint b2) {
  arr dist;
  for(auto &shape: s->G->bodies(b1)->shapes)
    if(shape->type != ors::markerST)
      shape->cont = true;
  for(auto &shape: s->G->bodies(b2)->shapes)
    if(shape->type != ors::markerST)
      shape->cont = true;
  s->G->swift().setCutoff(2.);
  s->G->swift().initActivations();
  for(uint f = 0; f < s->nframes; f++) {
    updateOrs(f);
    ors::Proxy *minProxy = NULL;
    for(auto &proxy: s->G->proxies) {
      if( s->G->shapes(proxy->a)->body->index == b1
          && s->G->shapes(proxy->b)->body->index == b2
          && (!minProxy || proxy->d < minProxy->d)) {
        minProxy = proxy;
      }
    }
    CHECK(minProxy, "Something is wrong here. Call Andrea.");
    dist.append(minProxy->d);
  }
  for(auto &shape: s->G->bodies(b1)->shapes)
    shape->cont = false;
  for(auto &shape: s->G->bodies(b2)->shapes)
    shape->cont = false;
  return dist;
}

arr KeyFramer::getDists(const String &n1, const String &n2) {
  ors::Body *b1 = s->G->getBodyByName(n1);
  CHECK(b1, "Invalid name.");
  ors::Body *b2 = s->G->getBodyByName(n2);
  CHECK(b2, "Invalid name.");

  return getDists(b1->index, b2->index);
}

void KeyFramer::calcProxies(const String &n1, const String &n2) {
  ors::Body *b1 = s->G->getBodyByName(n1);
  CHECK(b1, "Invalid name.");
  ors::Body *b2 = s->G->getBodyByName(n2);
  CHECK(b2, "Invalid name.");

  calcProxies(b1->index, b2->index);
}

void KeyFramer::clearProxies() {
  listDelete(s->G->proxies);
}

KeyFrameL KeyFramer::getKeyFrames(const arr &corr, const ProxyL &proxies) {
  KeyFrameL keyframes;
  KeyFrame *kf;

  bool kf_flag = false;
  for(uint f = 0; f < corr.N; f++) {
    if(!kf_flag && corr(f) >= s->thresh) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = true;
    }
    else if(kf_flag && corr(f) < s->thresh) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = false;
    }
    /*
    // TODO for now it's like this, continue..
    if(proxies(f) == NULL)
      continue;

    if(!kf_flag && 
        corr(f) >= s->thresh && proxies(f)->d <= s->dist) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = true;
    }
    else if(kf_flag && 
        (corr(f) < s->thresh || proxies(f)->d > s->dist)) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = false;
    }
    */
  }
  return keyframes;
}

void KeyFramer::saveKeyFrameScreens(const KeyFrameL &keyframes, uint df) {
  uint f;
  uint h, w;
  byteA img1, img2, img3;
  for(KeyFrame *kf: keyframes) {
    cout << *kf;
    f = kf->getFrame();

    if(f < df || f+df >= s->g4d->getNumFrames())
      continue;

    // saving keyframe image
    updateOrs(f-df);
    s->G->gl().text.clear() <<"frame " <<f-df << endl;
    s->G->gl().update(NULL, true);
    flip_image(s->G->gl().captureImage);
    byteA img1 = s->G->gl().captureImage;

    updateOrs(f);
    s->G->gl().text.clear() <<"frame " <<f << endl;
    s->G->gl().update(NULL, true);
    flip_image(s->G->gl().captureImage);
    byteA img2 = s->G->gl().captureImage;

    updateOrs(f+df);
    s->G->gl().text.clear() <<"frame " <<f+df << endl;
    s->G->gl().update(NULL, true);
    flip_image(s->G->gl().captureImage);
    byteA img3 = s->G->gl().captureImage;

    h = img1.d0;
    w = 3*img1.d1;
    byteA comp(h, w, 3);

    for(uint i = 0; i < h; i++) {
      memcpy(comp[i]().p    , img1[i]().p, w);
      memcpy(comp[i]().p+w  , img2[i]().p, w);
      memcpy(comp[i]().p+2*w, img3[i]().p, w);
    }

    write_ppm(comp, STRING("keyframe_"<<f<<".ppm"));
  }
}
