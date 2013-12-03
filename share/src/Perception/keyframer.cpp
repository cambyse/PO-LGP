#include <Core/geo.h>
#include "keyframer.h"

struct KeyFramer::sKeyFramer {
  uint nbodies, ndofs;
  uintA dofs, cumdofs;
  StringA names;

  uint nframes;
  arr state;

  arr sum, mean;
  arr err;

  sKeyFramer();
  ~sKeyFramer();
};

KeyFramer::sKeyFramer::sKeyFramer() {}
KeyFramer::sKeyFramer::~sKeyFramer() {}

KeyFramer::KeyFramer() {
  s = new sKeyFramer();
  clear();
}

KeyFramer::~KeyFramer() {
  delete s;
}

void KeyFramer::clear() {
  clearState();
}

void KeyFramer::clearState() {
  s->nbodies = 0;
  s->ndofs = 0;
  clearFrames();
}

void KeyFramer::clearFrames() {
  s->state.clear();
  s->nframes = 0;
}

void KeyFramer::addBody(String &name, uint body_ndofs) {
  CHECK(body_ndofs > 0, "Number of Dofs for a body must be positive.");
  CHECK(!s->names.contains(name), "Body name already exists.");

  s->nbodies++;
  s->ndofs += body_ndofs;

  s->cumdofs.append(s->nbodies == 1? 0: s->cumdofs.last()+s->dofs.last());
  s->dofs.append(body_ndofs);
  s->names.append(name);
}

uint KeyFramer::getNBodies() {
  return s->nbodies;
}

uint KeyFramer::getCumNDofs(uint b) {
  CHECK(b < s->nbodies, "Body index out of bounds.");
  return s->cumdofs(b);
}

uint KeyFramer::getNDofs(uint b) {
  CHECK(b < s->nbodies, "Body index out of bounds.");
  return s->dofs(b);
}

uint KeyFramer::getNDofs() {
  return s->ndofs;
}

void KeyFramer::addState(arr st) {
  CHECK(st.N == s->ndofs, "Wrong number of dofs.");
  s->nframes++;
  s->state.append(st);
  s->state.reshape(s->nframes, s->ndofs);
}

void KeyFramer::addState() {
  arr st(s->ndofs);
  st.setZero();
  addState(st);
}

void KeyFramer::setState(uint b, arr st, uint f) {
  CHECK(b < s->nbodies, "Invalid body index.");
  CHECK(s->dofs(b) == st.N, "Wrong number of dofs.");
  CHECK(f <= s->nframes, "Frame number out of bounds.");
  /*
  for(uint i = 0; i < st.N; i++)
    s->state(f, s->cumdofs(b)+i) = st(i);
  */
  // TODO does this work?
  s->state[f]().replace(s->cumdofs(b), st.N, st);
}

void KeyFramer::setState(String n, arr st, uint f) {
  int b = s->names.findValue(n);
  CHECK(b >= 0, "Invalid name.");
  setState(b, st, f);
}

void KeyFramer::setState(arr st, uint f) {
  CHECK(st.N == s->ndofs, "Wrong number of dofs.");
  CHECK(f < s->nframes, "Frame number out of bounds.");
  s->state[f]() = st;
}

void KeyFramer::setupWindows(MT::Array<arr> &wins, uint wlen) {
  uint nwins = getNWindows(wlen);

  CHECK(wlen>0, "Window length has to be positive.");
  CHECK(nwins>0, "Not enough frames to have at least one whole window.");

  wins.resize(nwins);
  for(uint wi = 0; wi < nwins; wi++)
    wins(wi).referToSubRange(s->state, wi, wi+wlen-1);
}


uint KeyFramer::getNFrames() {
  return s->nframes;
}

uint KeyFramer::getNWindows(uint wlen) {
  return s->nframes>=wlen? s->nframes-wlen+1: 0;
}

arr KeyFramer::getState() {
  return s->state;
}

arr KeyFramer::getState(uint f) {
  CHECK(f < s->nframes, "Frame number out of bounds.");
  return s->state.sub(f, f, 0, -1).resize(s->ndofs); // TODO can this be optimized?
}

arr KeyFramer::getState(uint f, uint b) {
  CHECK(f < s->nframes, "Frame number out of bounds.");
  CHECK(b < s->nbodies, "Body index out of bounds.");
  uint dofs = s->dofs(b);
  uint cumdofs = s->cumdofs(b);
  return s->state.sub(f, f, cumdofs, cumdofs+dofs-1).resize(dofs); // TODO can this be optimized?
}

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

arr KeyFramer::getCorrWOPCA(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  MT::Array<arr> wins;
  setupWindows(wins, wlen);
  uint nwins = wins.N;

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(nwins, dofs);

  arr x, y, xx, yy, xy;
  arr sx, sy, sxx, syy, sxy;
  arr t1, t2, t3, t4a, t4b, t4, t5, t6a, t6;
  double dwlen = wlen;
  for(uint wi = 0; wi < nwins; wi++) {
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

arr KeyFramer::getCorrPCA(uint b1, uint b2, uint wlen) {
  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");

  MT::Array<arr> wins;
  setupWindows(wins, wlen);
  uint nwins = wins.N;

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(nwins);

  arr x, y, w, t;
  double sx, sy, sxx, syy, sxy;
  for(uint wi = 0; wi < nwins; wi++) {
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

    corr(wi) = (sxy*wlen - sx*sy) / sqrt(fabs((sxx*wlen - sx*sx)*(syy*wlen - sy*sy)));
      //cout << "wii " << wii << " wi " << wi << " value " << corr(wii).elem(wi) << endl;
      //

      /*
      arr D;
      D.append(x);
      D.append(y);
      D.reshape(2, D.N/2);
      gnuplot(~D);
      */

      /*
      cout << "x: " << x << endl;
      cout << "y: " << y << endl;
      cout << "sx: " << sx << endl;
      cout << "sy: " << sy << endl;
      cout << "sxx: " << sxx << endl;
      cout << "sxy: " << sxy << endl;
      cout << "syy: " << syy << endl;
      cout << "N: " << N << endl;
      cout << "corr: " << endl;
      cout << "c1: " << (sxy*N - sx*sy) << endl;
      cout << "c2: " << (sxx*N - sx*sx) << endl;
      cout << "c3: " << (syy*N - sy*sy) << endl;
      cout << "c4: " << fabs((sxx*N - sx*sx)*(syy*N - sy*sy)) << endl;
      cout << "c5: " << sqrt(fabs((sxx*N - sx*sx)*(syy*N - sy*sy))) << endl;
      cout << "c6: " << corr(wii).elem(wi) << endl;
      */
  }

  return corr;
}

arr KeyFramer::getCorr(uint b1, uint b2, uint wlen, bool pca) {
  return pca? getCorrPCA(b1, b2, wlen): getCorrWOPCA(b1, b2, wlen);
}

MT::Array<arr> KeyFramer::getCorrEnsemble(uint b1, uint b2, uintA &wlens, bool pca) {
  // NB: here nwins is not the same number of windows as in the other methods.
  // other methods: nwins = number of windows of a certain size throughout the
  // whole stream
  // this method: nwins = number of different window sizes.
  uint nwins = wlens.N;

  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");
  CHECK(nwins>0, "Specify at least one window size.");

  MT::Array<arr> corr(nwins);
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

  arr angle = getAngle(b1, b2);
  arr t;
  double m;

  uint nwins = getNWindows(wlen);
  arr var(nwins);
  for(uint wi = 0; wi < nwins; wi++) {
    t.referToSubRange(angle, wi, wi+wlen-1);
    var(wi) = sumOfSqr(t-mean(t));
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

void KeyFramer::keyframes() {
  // TODO find a good structure to contain keyframes..
}

