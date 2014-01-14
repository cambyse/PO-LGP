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
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");
  CHECK(npc > 0 && npc <= 3, "Number of principal components must be positive but <=4.");

  MT::Array<arr> wins;
  s->setupWindows(wins, wlen);

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(s->nframes, npc);

  arr x, y, xx, yy, xy;
  arr sx, sy, sxx, syy, sxy;
  arr w, t;
  //double sx, sy, sxx, syy, sxy;
  double dwlen = wlen;
  for(uint fi = 0; fi < wlen-1; fi++)
    corr[fi]() = 0;
  for(uint fi = wlen-1; fi < s->nframes; fi++) {
    uint wi = fi -wlen +1;

    x = wins(wi).cols(cumdofs1, cumdofs1+dofs);
    y = wins(wi).cols(cumdofs2, cumdofs2+dofs);

    // center x
    // TODO put this into pca..
    // TODO should already be there..
    /*
    t = sum(x, 0);
    for(uint i = 0; i < x.d0; i++)
      x[i]() -= t;
    */

    // run pca on x and then y
    pca(x, t, w, x, npc);
    y = y * w;

    xx = elemWiseProd(x, x);
    yy = elemWiseProd(y, y);
    xy = elemWiseProd(x, y);

    sx = sum(x, 0);
    sy = sum(y, 0);
    sxx = sum(xx, 0);
    syy = sum(yy, 0);
    sxy = sum(xy, 0);

    corr[wi]() = elemWiseDiv(
                    dwlen*sxy - elemWiseProd(sx, sy),
                    sqrt(elemWiseProd(
                        dwlen*sxx - elemWiseProd(sx, sx),
                        dwlen*syy - elemWiseProd(sy, sy)
                    ))
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
  for(uint fi = 0; fi < wlen-1; fi++)
    corr[fi]() = 0;
  for(uint fi = wlen-1; fi < nwins; fi++) {
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
    corr[wi]() = elemWiseDiv(
                    dwlen*sxy - elemWiseProd(sx, sy),
                    sqrt(elemWiseProd(
                        dwlen*sxx - elemWiseProd(sx, sx),
                        dwlen*syy - elemWiseProd(sy, sy)
                    ))
                  );
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

KeyFrameL KeyFramer::getKeyFrames(const arr &q) {
  KeyFrameL keyframes;
  KeyFrame *kf;

  bool kf_flag = false;
  for(uint f = 0; f < q.d0; f++) {
    if(!kf_flag && q(f, 1) >= .5) {
      kf = new KeyFrame(f);
      keyframes.append(kf);
      kf_flag = true;
    }
    else if(kf_flag && q(f, 1) < .5) {
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

arr KeyFramer::EM(const arr &c, const arr &v) {
  arr p = { 1, 0 };
  arr A = { .5, .5, .5, .5 };
  A.reshape(2, 2);

  arr c_mu = { 0, .9 };
  arr c_sigma = { .7, .3 };

  arr y_p = { .5, .5, 1, 0 };
  y_p.reshape(2, 2);

  arr v_mu = { 0, 0 };
  arr v_sigma = { 1, .7 };

  arrL theta;
  theta.append(&p);
  theta.append(&A);
  theta.append(&c_mu);
  theta.append(&c_sigma);
  theta.append(&y_p);
  theta.append(&v_mu);
  theta.append(&v_sigma);

  uint T = c.d0, K = c_mu.N, J = v_mu.N;
  arr rho_z(T, K), rho_y(T, J);
  arrL rho;
  rho.append(&rho_z);
  rho.append(&rho_y);

  arr q(T, K), q_pair(T-1, K, K), q_y(T, J);
  arrL ql;
  ql.append(&q);
  ql.append(&q_pair);
  ql.append(&q_y);

  for(uint i = 0; i < 10; i++) {
    cout << endl;
    cout << "step: " << i << endl;
    cout << "p: " << p << endl;
    cout << "A: " << A << endl;
    cout << "c_mu: " << c_mu << endl;
    //cout << "c_sigma: " << c_sigma << endl;
    cout << "y_p: " << y_p << endl;
    //cout << "v_mu: " << v_mu << endl;
    //cout << "v_sigma: " << v_sigma << endl;

    computeEvidences(rho, c, v, theta);
    EstepQ(ql, theta, rho);
    Mstep(theta, ql, c, v); // TODO fix.. corr % var?
  }

  return q;
}

void KeyFramer::computeEvidences(arrL &rho, const arr &c, const arr &v, const arrL &theta) {
  arr c_mu, c_sigma, y_p, v_mu, v_sigma;
  c_mu.referTo(*theta(2));
  c_sigma.referTo(*theta(3));
  y_p.referTo(*theta(4));
  v_mu.referTo(*theta(5));
  v_sigma.referTo(*theta(6));

  arr rho_z, rho_y;
  rho_z.referTo(*rho(0));
  rho_y.referTo(*rho(1));
  
  //-- evidences from observations
  uint T = c.d0, K = c_mu.N, J = v_mu.N;
  arr c_rho, v_rho;
  c_rho.resize(T, K);
  v_rho.resize(T, K);
  //rho_y.resize(T, J);

  for(uint t = 0; t < T; t++) {
    for(uint k = 0; k < K; k++) {
      c_rho(t, k) = ::exp(
          -.5 * MT::sqr(c(t) - c_mu(k)) / (c_sigma(k) * c_sigma(k))
          );

      v_rho(t, k) = 0;
      for(uint j = 0; j < J; j++)
        v_rho(t, k) += y_p(k, j) * ::exp(
            -.5 * MT::sqr(v(t) - v_mu(j)) / (v_sigma(j) * v_sigma(j))
            );
    }
    for(uint j = 0; j < J; j++)
      rho_y(t, j) = ::exp(
          -.5 * MT::sqr(v(t) - v_mu(j)) / (v_sigma(j) * v_sigma(j))
        );
  }
  rho_z = c_rho % v_rho;
}

void KeyFramer::Estep(arr& a, arr& b, const arr& P0, const arr& P, const arr& rho) {
  uint T=rho.d0, K=P.d0;
  a.resize(T, K);
  b.resize(T, K);
  a[0]() = P0;   //initialization of alpha
  b[T-1]() = 1./K; //initialization of beta
  //--- fwd and bwd iterations:
  for(uint t = 1; t < T; t++) {
    a[t]() =  P * (rho[t-1]%a[t-1]); // %=element-wise multiplication, *=inner product
    normalizeDist(a[t]()); //for numerical stability
  }
  for(uint t = T-1; t--; ) {
    b[t]() = ~P * (rho[t+1]%b[t+1]);
    normalizeDist(b[t]());
  }
}

void KeyFramer::EstepQ(arrL &ql, const arrL &theta, const arrL &rho) {
  arr rho_z, rho_y;
  rho_z.referTo(*rho(0));
  rho_y.referTo(*rho(1));

  arr q, q_pair, q_y;
  q.referTo(*ql(0));
  q_pair.referTo(*ql(1));
  q_y.referTo(*ql(2));

  arr P0, P, y_p;
  P0.referTo(*theta(0));
  P.referTo(*theta(1));
  y_p.referTo(*theta(4));

  uint T = rho.d0, K = P.d0, J = y_p.d0;
  arr a,b;
  Estep(a, b, P0, P, rho_z);

  //q.resize(T, K);
  //q_pair.resize(T-1, K, K);
  //q_y.resize(T, J);

  for(uint t = 0; t < T; t++)
    q[t]() = a[t]() % rho_z[t]() % b[t](); // %=element-wise multiplication
  for(uint t = 0; t < T-1; t++)
    for(uint k = 0; k < K; k++)
      for(uint j = 0; j < K; j++)
        q_pair(t, k, j) = a(t, k)*rho_z(t, k)*P(j, k)*rho_z(t+1, j)*b(t+1, j);
  for(uint t = 0; t < T; t++)
    for(uint j = 0; j < J; j++)
      q_y(t, j) = y_p(0, j) * rho_y(t, j);

  for(uint t = 0; t < T; t++)
    normalizeDist(q[t]());
  for(uint t = 0; t < T-1; t++)
    normalizeDist(q_pair[t]());
  for(uint t = 0; t < T-1; t++)
    normalizeDist(q_y[t]());
  cout << "----------------------" << endl;
  cout << "q: " << q << endl;
  cout << "q[0]: " << q << endl;
  cout << "^^^^^^^^^^^^^^^^^^^^^^" << endl;
  //cout << "q_y: " << q_y << endl;
  //cout << "q_y[0]: " << q_y[0] << endl;
  cout << "----------------------" << endl;
}

void KeyFramer::Mstep(arrL& theta, const arrL &ql, const arr& c, const arr &v){
  arr q, q_pair, q_y;
  q.referTo(*ql(0));
  q_pair.referTo(*ql(1));
  q_y.referTo(*ql(2));

  arr P0, P, c_mu, y_p;
  P0.referTo(*theta(0));
  P.referTo(*theta(1));
  c_mu.referTo(*theta(2));
  y_p.referTo(*theta(4));

  uint T = c.d0, K = P.d0, J = y_p.d0;
  P0 = q[0];

  P.setZero();
  arr p(K); p.setZero();
  for(uint t = 0; t < T-1; t++) {
    p += q[t];
    P += q_pair[t];
  }
  for(uint k = 0; k < K; k++)
    for(uint j = 0; j < K; j++)
      P(j, k) = P(j, k) / p(k);

  arr w(K,T);
  arr qsum = sum(q,0);
  for(uint t = 0; t < T; t++)
    for(uint k = 0; k < K; k++)
      w(k, t) = q(t, k) / qsum(k);

  c_mu = w*c;
  //c_sigma = w*(y%y) - mu%mu;
  
  y_p[0]() = 0;
  for(uint j = 0; j < J; j++) {
    for(uint t = 0; t < T; t++)
      y_p(0, j) += q_y(t, j) * q(t, 0);
    y_p(0, j) /= qsum(0);
  }

  cout << "=========================" << endl;
  cout << "q: " << q[1] << endl;
  cout << "q_y: " << q_y[1] << endl;
  cout << "y_p: " << y_p[0] << endl;
  cout << "=========================" << endl;
}

