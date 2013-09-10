#include <unistd.h>
#include "keyframer.h"

KeyFramer::KeyFramer() {
  clear();
}

KeyFramer::~KeyFramer() {
}

void KeyFramer::clear() {
  lwin = 0;
  clearS();
}

void KeyFramer::clearS() {
  nbodies = 0;
  ndofs = 0;
  agent = 0;
  clearT();
}

void KeyFramer::clearT() {
  s.clear();
  T = 0;
}

void KeyFramer::addBody(int nd) {
  CHECK(nd > 0, "Value of 'nd' out of bounds.");

  nbodies++;
  ndofs += nd;

  dofs.append(nd);
  if(nbodies == 1)
    cumdofs.append(0);
  else
    cumdofs.append(cumdofs.last() + nd);
}

int KeyFramer::getNBodies() {
  return nbodies;
}

int KeyFramer::getCumNDofs(int b) {
  CHECK(b >= 0 && b < nbodies, "Value of 'b' out of bounds.");
  return cumdofs(b);
}

int KeyFramer::getNDofs(int b) {
  CHECK(b >= -1 && b < nbodies, "Value of 'b' out of bounds.");
  if(b == -1)
    return ndofs;
  return dofs(b);
}

int KeyFramer::agentDofs() {
  return dofs(agent);
}

void KeyFramer::addState(arr st) {
  CHECK(st.N == ndofs, "State with wrong ndofs");
  T++;
  s.append(st);
  s.resize(T, ndofs);
}

void KeyFramer::addState(arr st, int t) {
  CHECK(st.N == ndofs, "State with wrong ndofs");
  CHECK(t >= 0 && t <= T, "Value of 't' out of bounds.");
  if(t == T)
    addState(st);
  else
    s[t]() = st;
}

int KeyFramer::getT() {
  return T;
}

arr KeyFramer::getState(int t) {
  CHECK(t >= 0 && t < T, "Value of 't' out of bounds.");
  return s.sub(t, t, 0, -1).resize(ndofs);
}

arr KeyFramer::getState(int t, int b) {
  CHECK(t >= 0 && t < T, "Value of 't' out of bounds.");
  CHECK(b >= 0 && b < nbodies, "Value of 'b' out of bounds.");
  return s.sub(t, t, cumdofs(b), cumdofs(b) + dofs(b) - 1).resize(dofs(b));
}

arr KeyFramer::getWindow(int t) {
  CHECK(t >= 0 && t < T, "Value of 't' out of bounds.");
  return s.sub(t, t+lwin-1, 0, -1);
}

arr KeyFramer::getWindow(int t, int b) {
  CHECK(t >= 0 && t < T, "Value of 't' out of bounds.");
  CHECK(b >= -1 && b < nbodies, "Value of 'b' out of bounds.");
  return s.sub(t, t+lwin-1, cumdofs(b), cumdofs(b) + dofs(b) - 1);
}

void KeyFramer::setAgent(int a) {
  CHECK(a >= 0 && a < nbodies, "Value of 'a' out of bounds.");
  agent = a;
}

void KeyFramer::setLWin(int l) {
  CHECK(l > 1, "Value of 'lwin' out of bounds.");
  lwin = l;
}

void KeyFramer::run() {
  CHECK(T > 0, "Value of 'T' out of bounds.");
  CHECK(lwin > 1, "Value of 'lwin' out of bounds.");
  CHECK(T > lwin, "T > lwin.");
  arr win, tr, te, Phy, PhyT, W, y, diff;

  arr o = ones(lwin, 1);
  MT::Array<arr*> blocks;
  blocks.append(&o);
  blocks.append(&tr);

  err.resize(ndofs, T-lwin);
  for(int t = 0; t < T-lwin; t++) {
    win = getWindow(t);
    tr = win.sub(0, -1, cumdofs(agent), cumdofs(agent) + dofs(agent) - 1);
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

    int dofi;
    for(int b = 0; b < nbodies; b++) {
      for(int d = 0; d < dofs(b); d++) {
        dofi = cumdofs(b) + d;
        /*
        cout << "dofi = " << dofi << endl;
        cout << "diff[dofi] = " << diff[dofi] << endl;
        cout << "norm -- = " << norm(diff[dofi]) << endl;
        usleep(1000000);
        */
        err(dofi, t) = norm(diff[dofi]);
      }
    }
  }
  err = ~err;
}

/* TODO
arr KetFramer::gerErr(int b, int d) {
  CHECK(b >= -1 && b < nbodies, "Value of 'b' out of bounds.");

}
*/

arr KeyFramer::getErr() {
  return err.sub(0, -1, 0, -1);
}

arr KeyFramer::getErr(int b) {
  CHECK(b >= 0 && b < nbodies, "Value of 'b' out of bounds.");
  return err.sub(0, -1, cumdofs(b), cumdofs(b) + dofs(b) - 1);
}

void KeyFramer::keyframes() {
  // TODO find a good structure to contain keyframes..
}

void KeyFramer::test() {
  cout << "err = " << err.d0 << "x" << err.d1 << endl;
}
