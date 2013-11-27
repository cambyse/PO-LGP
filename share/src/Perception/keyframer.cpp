#include <unistd.h> // TODO why is this here?
#include "keyframer.h"

struct KeyFramer::sKeyFramer {
  uint nbodies, ndofs;
  uintA dofs, cumdofs;
  int agent;

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
  s->agent = -1;
  clearFrames();
}

void KeyFramer::clearFrames() {
  s->state.clear();
  s->nframes = 0;
}

void KeyFramer::addBody(uint body_ndofs) {
  CHECK(body_ndofs > 0, "Number of Dofs for a body must be positive.");

  s->nbodies++;
  s->ndofs += body_ndofs;

  s->dofs.append(body_ndofs);
  s->cumdofs.append(s->nbodies == 1? 0: s->cumdofs.last()+body_ndofs);
}

uint KeyFramer::getNBodies() {
  return s->nbodies;
}

uint KeyFramer::getAgentNDofs() {
  CHECK(s->agent >= 0, "Agent body not selected.");
  return getNDofs(s->agent);
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

void KeyFramer::addState(arr st, uint f) {
  CHECK(st.N == s->ndofs, "Wrong number of dofs.");
  CHECK(f <= s->nframes, "Frame number out of bounds.");
  if(f == s->nframes)
    addState(st);
  else
    s->state[f]() = st;
}

uint KeyFramer::getNFrames() {
  return s->nframes;
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

void KeyFramer::setAgent(uint a) {
  CHECK(a < s->nbodies, "Agent index out of bounds.");
  s->agent = a;
}

/*
// TODO here is the magic
  if(s->nframes >= s->lwin) {
    s->nwindows++;
    s->windows.append();
    s->windows.last().referToSubRange(s->state, s->nframes-s->lwin, -1);
  }

  +
*/

/* TODO here it is,also
void KeyFramer::setLWin(uint l) {
  CHECK(l > 1, "Window length must be > 1.");
  s->lwin = l;
  s->nwindows = (s->nframes>=s->lwin? s->nframes-s->lwin+1: 0);
  s->windows.resize(s->nwindows);
  for(uint w = 0; w < s->nwindows; w++)
    s->windows(w).referToSubRange(s->state, w, w+l-1);
}
*/

arr KeyFramer::getCorrWOPCA(uint b1, uint b2, uint wlen) {
  uint nwins = s->nframes>=wlen? s->nframes-wlen+1: 0;

  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");
  CHECK(wlen>0, "Window length has to be positive.");
  CHECK(nwins>0, "Not enough frames to have at least one whole window.");

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(nwins, dofs);

  MT::Array<arr> wins(nwins);
  for(uint wi = 0; wi < nwins; wi++)
    wins(wi).referToSubRange(s->state, wi, wi+wlen-1);

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
  uint nwins = s->nframes>=wlen? s->nframes-wlen+1: 0;

  CHECK(s->dofs(b1)==s->dofs(b2), "Doesn't support bodies with different number of dofs.");
  CHECK(wlen>0, "Window length has to be positive.");
  CHECK(nwins>0, "Not enough frames to have at least one whole window.");

  uint dofs = s->dofs(b1);
  uint cumdofs1 = s->cumdofs(b1);
  uint cumdofs2 = s->cumdofs(b2);

  arr corr(nwins);

  MT::Array<arr> wins(nwins);
  for(uint wi = 0; wi < nwins; wi++)
    wins(wi).referToSubRange(s->state, wi, wi+wlen-1);

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

MT::Array<arr> KeyFramer::getCorrEnsemble(uint b1, uint b2, uintA wlens, bool pca) {
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

void KeyFramer::keyframes() {
  // TODO find a good structure to contain keyframes..
}

