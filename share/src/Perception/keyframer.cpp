#include <unistd.h> // TODO why is this here?
#include "keyframer.h"

struct KeyFramer::sKeyFramer {
  uint nbodies, ndofs;
  uintA dofs, cumdofs;
  int agent;

  uint nframes;
  arr state;

  uint lwin;

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
  s->lwin = 0;
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
  s->state.resize(s->nframes, s->ndofs);
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

void KeyFramer::setAgent(uint a) {
  CHECK(a < s->nbodies, "Agent index out of bounds.");
  s->agent = a;
}

void KeyFramer::setLWin(uint l) {
  CHECK(l > 1, "Window length must be > 1.");
  s->lwin = l;
}

void KeyFramer::run() {
  CHECK(s->nframes > 0, "No frames available.");
  CHECK(s->lwin > 1, "Window length should be > 1.");
  CHECK(s->nframes > s->lwin, "Number of frames should be > window length.");
  arr win, tr, te, Phy, PhyT, W, y, diff;

  arr o = ones(s->lwin, 1);
  MT::Array<arr*> blocks;
  blocks.append(&o);
  blocks.append(&tr);

  uint agent_dofs = s->dofs(s->agent);
  uint agent_cumdofs = s->cumdofs(s->agent);
  s->err.resize(s->ndofs, s->nframes-s->lwin);
  for(uint f = 0; f < s->nframes-s->lwin; f++) {
    win = getWindow(f);
    tr = win.sub(0, -1, agent_cumdofs, agent_cumdofs+agent_dofs-1);
    te = win.sub(0, -1, 0, -1);

    Phy = catCol(blocks);
    PhyT = ~Phy;

    W = inverse(PhyT*Phy)*PhyT*te;
    y = Phy*W;

    diff = y-te;
    diff = ~diff; // TODO fix this..

    /*
    cout << "y = " << y.d0 << "x" << y.d1 << endl;
    cout << "te = " << te.d0 << "x" << te.d1 << endl;
    cout << "diff = " << diff.d0 << "x" << diff.d1 << endl;
    usleep(10000000);
    */

    /*
    cout << "diff = " << diff.d0 << "x" << diff.d1 << endl;
    cout << "diff = " << diff << endl;

    cout << "err = " << err.d0 << "x" << err.d1 << endl;
    cout << "err = " << err << endl;
    */

    uint dofi;
    for(uint b = 0; b < s->nbodies; b++) {
      for(uint d = 0; d < s->dofs(b); d++) {
        dofi = s->cumdofs(b) + d;
        /*
        cout << "dofi = " << dofi << endl;
        cout << "diff[dofi] = " << diff[dofi] << endl;
        cout << "length -- = " << length(diff[dofi]) << endl;
        usleep(1000000);
        */
        s->err(dofi, f) = length(diff[dofi]);
      }
    }
  }
  s->err = ~s->err;
}

/* TODO
arr KetFramer::gerErr(int b, int d) {
  CHECK(b >= -1 && b < nbodies, "Value of 'b' out of bounds.");

}
*/

arr KeyFramer::getErr() {
  return s->err.sub(0, -1, 0, -1);
}

arr KeyFramer::getErr(uint b) {
  CHECK(b < s->nbodies, "Body index out of bounds.");
  uint cumdofs = s->cumdofs(b);
  uint dofs = s->dofs(b);
  return s->err.sub(0, -1, cumdofs, cumdofs+dofs-1);
}

void KeyFramer::keyframes() {
  // TODO find a good structure to contain keyframes..
}

void KeyFramer::test() {
  cout << "err = " << s->err.d0 << "x" << s->err.d1 << endl;
}

