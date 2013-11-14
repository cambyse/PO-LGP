#include <Perception/keyframer.h>


void loadKF(KeyFramer &kf) {
  kf.addBody(2);

  kf.addBody(3);
  kf.setAgent(1);

  kf.addBody(4);
}

void testKF(KeyFramer &kf) {
  uint nb, ndofs;

  uint checkNBodies = 3;
  uint checkAgentNDofs = 3;
  intA checkNDofs = {2, 3, 4};
  intA checkCumNDofs = {0, 3, 7};
  
  nb = kf.getNBodies();
  cout << "num bodies: " << nb << endl;
  CHECK(nb == checkNBodies, "Wrong num bodies");

  ndofs = kf.getAgentNDofs();
  cout << "num dofs agent: " << ndofs << endl;
  CHECK(ndofs == checkAgentNDofs, "Wrong num agent dofs");

  for(uint b = 0; b < nb; b++) {
    ndofs = kf.getNDofs(b);
    cout << "num dofs body " << b << ": " << ndofs << endl;
    CHECK(ndofs == checkNDofs(b), "Wrong num dofs");
  }

  for(uint b = 0; b < nb; b++) {
    ndofs = kf.getCumNDofs(b);
    cout << "num cum dofs body " << b << ": " << ndofs << endl;
    CHECK(ndofs == checkCumNDofs(b), "Wrong num cum dofs");
  }
}

int main(int argc, char** argv) {
  KeyFramer kf;
  loadKF(kf);
  testKF(kf);
}

