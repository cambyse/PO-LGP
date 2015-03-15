#include <Perception/keyframer.h>

void loadKF(KeyFramer &kf) {
  /*
  kf.addBody(2);

  kf.addBody(3);
  kf.setAgent(1);

  kf.addBody(4);

  kf.setLWin(2); // TODO doesn't exist anymore
  kf.addState({ 0, 0, 1, 2, 3, 0, 0, 0, 0 });
  kf.addState({ 1, 1, 0, 0, 0, 1, 2, 3, 4 });
  kf.addState({ 2, 2, 1, 1, 1, 2, 4, 6, 8 });
  */
  kf.addBody(3);

  kf.addBody(3);
  kf.setAgent(1);

  kf.addBody(3);

  kf.addState({ 0, 0, 1, 2, 3, 0, 0, 0, 0 });
  kf.addState({ 1, 1, 0, 0, 0, 1, 2, 3, 4 });
  kf.addState({ 2, 2, 1, 1, 1, 2, 4, 6, 8 });
  kf.addState({ 3, 1, 2, 0, 2, 1, 5, 5, 9 });
}

void testKF(KeyFramer &kf) {
  uint nb, ndofs, nframes;
  arr state, corr;

  uint wlen = 3;
  // Assuming now that I only hve positional DoFs
  uint checkNBodies = 3;
  uint checkAgentNDofs = 3;
  intA checkNDofs = {3, 3, 3};
  intA checkCumNDofs = {0, 3, 6};
  uint checkNFrames = 4;
  arr checkState = { 0, 0, 1, 2, 3, 0, 0, 0, 0,
                     1, 1, 0, 0, 0, 1, 2, 3, 4,
                     2, 2, 1, 1, 1, 2, 4, 6, 8,
                     3, 1, 2, 0, 2, 1, 5, 5, 9 };
  checkState.reshape(checkNFrames, checkState.N/checkNFrames);

  nb = kf.getNBodies();
  cout << "num bodies: " << nb << endl;
  CHECK_EQ(nb , checkNBodies, "Wrong num bodies");

  ndofs = kf.getAgentNDofs();
  cout << "num dofs agent: " << ndofs << endl;
  CHECK_EQ(ndofs , checkAgentNDofs, "Wrong num agent dofs");

  for(uint b = 0; b < nb; b++) {
    ndofs = kf.getNDofs(b);
    cout << "num dofs body " << b << ": " << ndofs << endl;
    CHECK_EQ(ndofs , checkNDofs(b), "Wrong num dofs");
  }

  for(uint b = 0; b < nb; b++) {
    ndofs = kf.getCumNDofs(b);
    cout << "num cum dofs body " << b << ": " << ndofs << endl;
    CHECK_EQ(ndofs , checkCumNDofs(b), "Wrong num cum dofs");
  }

  nframes = kf.getNFrames();
  cout << "nframes: " << nframes << endl;
  CHECK_EQ(nframes , checkNFrames, "Wrong nframes");

  cout << endl;
  cout << "== STATE ======================================" << endl;
  state = kf.getState();
  cout << "state: " << state << endl;
  CHECK_EQ(state , checkState, "Wrong state");

  cout << endl;
  cout << "== CORR =======================================" << endl;

  cout << endl;
  corr = kf.getCorr(0, 1, wlen, false);
  cout << "corr 0 1 WOPCA: " << corr << endl;

  cout << endl;
  corr = kf.getCorr(0, 2, wlen, false);
  cout << "corr 0 2 WOPCA: " << corr << endl;

  cout << endl;
  corr = kf.getCorr(1, 2, wlen, false);
  cout << "corr 1 2 WOPCA: " << corr << endl;

  cout << endl;
  corr = kf.getCorr(0, 1, wlen, true);
  cout << "corr 0 1 PCA: " << corr << endl;

  cout << endl;
  corr = kf.getCorr(0, 2, wlen, true);
  cout << "corr 0 2 PCA: " << corr << endl;

  cout << endl;
  corr = kf.getCorr(1, 2, wlen, true);
  cout << "corr 1 2 PCA: " << corr << endl;
}

int main(int argc, char** argv) {
  KeyFramer kf;
  loadKF(kf);
  testKF(kf);
}

