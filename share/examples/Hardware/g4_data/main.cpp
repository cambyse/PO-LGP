#include <Perception/g4data.h>
#include <Core/array.h>

void loadData(G4Data &g4d) {
  MT::String meta = MT::getParameter<MT::String>("meta");
  MT::String poses = MT::getParameter<MT::String>("poses");

  g4d.loadData(meta, poses, true);
}

void testData(G4Data &g4d) {
  uint checkF              = 10;
  uint checkN              = 14;
  uint checkNrh            = 4;
  uint checkNrhthumb       = 1;
  uint checkNbook          = 1;
  uintA checkD9            = {         checkN,        7 };
  uintA checkDbook         = { checkF, checkNbook,    7 };
  uintA checkD9book        = {         checkNbook,    7 };
  uintA checkDrh           = { checkF, checkNrh,      7 };
  uintA checkD9rh          = {         checkNrh,      7 };
  uintA checkDrhthumb      = { checkF, checkNrhthumb, 7 };
  uintA checkD9rhthumb     = {         checkNrhthumb, 7 };
  uintA checkDPosrhthumb   = { checkF, checkNrhthumb, 3 };
  uintA checkDPos9rhthumb  = {         checkNrhthumb, 3 };

  uint F, N, v;
  arr data;
  MT::Array<uint> dim;

  cout << endl;
  cout << "========================================" << endl;
  cout << "frames" << endl;
  cout << "========================================" << endl;
  F = g4d.getNumFrames();
  cout << "F              = " << F << endl;
  CHECK(F == checkF, "wrong value of F");

  cout << endl;
  cout << "========================================" << endl;
  cout << "sensors" << endl;
  cout << "========================================" << endl;
  N = g4d.getNumSensors();
  cout << "N              = " << N << endl;
  CHECK(N == checkN, "wrong value of N");

  cout << endl;
  cout << "========================================" << endl;
  cout << "sensors" << endl;
  cout << "========================================" << endl;
  N = g4d.getNumSensors("rh");
  cout << "N rh           = " << N << endl;
  CHECK(N == checkNrh, "wrong value of N");

  cout << endl;
  cout << "========================================" << endl;
  cout << "sensors" << endl;
  cout << "========================================" << endl;
  N = g4d.getNumSensors("rh:thumb");
  cout << "N rh:thumb     = " << N << endl;
  CHECK(N == checkNrhthumb, "wrong value of N");

  cout << endl;
  cout << "========================================" << endl;
  cout << "sensors" << endl;
  cout << "========================================" << endl;
  N = g4d.getNumSensors("book");
  cout << "N book         = " << N << endl;
  CHECK(N == checkNbook, "wrong value of N");

  cout << endl;
  cout << "========================================" << endl;
  cout << "dims" << endl;
  cout << "========================================" << endl;
  data = g4d.query(9u);
  dim = data.getDim();
  cout << "dim 9          = " << dim << endl;
  CHECK(dim == checkD9, "wrong value of dim");

  data = g4d.query("book");
  dim = data.getDim();
  cout << "dim book       = " << dim << endl;
  CHECK(dim == checkDbook, "wrong value of dim");

  data = g4d.query(9u, "book");
  dim = data.getDim();
  cout << "dim 9 book     = " << dim << endl;
  CHECK(dim == checkD9book, "wrong value of dim");

  data = g4d.query("rh");
  dim = data.getDim();
  cout << "dim rh         = " << dim << endl;
  CHECK(dim == checkDrh, "wrong value of dim");

  data = g4d.query(9u, "rh");
  dim = data.getDim();
  cout << "dim 9 rh       = " << dim << endl;
  CHECK(dim == checkD9rh, "wrong value of dim");

  data = g4d.query("rh:thumb");
  dim = data.getDim();
  cout << "dim rh thumb   = " << dim << endl;
  CHECK(dim == checkDrhthumb, "wrong value of dim");

  data = g4d.query(9u, "rh:thumb");
  dim = data.getDim();
  cout << "dim 9 rh thumb = " << dim << endl;
  CHECK(dim == checkD9rhthumb, "wrong value of dim");

  data = g4d.queryPos("rh:thumb");
  dim = data.getDim();
  cout << "dim Pos rh thumb = " << dim << endl;
  CHECK(dim == checkDPosrhthumb, "wrong value of dim");

  data = g4d.queryPos(9u, "rh:thumb");
  dim = data.getDim();
  cout << "dim Pos 9 rh thumb = " << dim << endl;
  CHECK(dim == checkDPos9rhthumb, "wrong value of dim");

  cout << endl;
  cout << "========================================" << endl;
  cout << "values" << endl;
  cout << "========================================" << endl;
  data = g4d.query();
  v = data(0, 0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value");
  v = data(0, 1, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value");
  v = data(0, 2, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value");

  cout << endl;
  cout << "========================================" << endl;
  cout << "values" << endl;
  cout << "========================================" << endl;
  data = g4d.query(1u);
  v = data(0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");
  v = data(1, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");
  v = data(2, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");

  cout << endl;
  cout << "========================================" << endl;
  cout << "values" << endl;
  cout << "========================================" << endl;
  data = g4d.query("rh");
  v = data(2, 0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value");
  v = data(2, 1, 0);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value");
  v = data(2, 2, 0);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value");
  v = data(2, 3, 0);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value");

  cout << endl;
  cout << "========================================" << endl;
  cout << "values" << endl;
  cout << "========================================" << endl;
  data = g4d.query(0, "rh:thumb");
  v = data(0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value");
  v = data(0, 1);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");
  v = data(0, 2);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value");

  cout << endl;
  cout << "========================================" << endl;
  cout << "interpolation (beginning)" << endl;
  cout << "========================================" << endl;
  data = g4d.query(0, "bbox:lid");
  v = data(0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value (interpolation)");
  v = data(0, 1);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value (interpolation)");
  v = data(0, 2);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value (interpolation)");

  cout << endl;
  cout << "========================================" << endl;
  cout << "interpolation (middle)" << endl;
  cout << "========================================" << endl;
  data = g4d.query(4, "bbox:lid");
  v = data(0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value (interpolation)");
  v = data(0, 1);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value (interpolation)");
  v = data(0, 2);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value (interpolation)");

  data = g4d.query(5, "bbox:lid");
  v = data(0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value (interpolation)");
  v = data(0, 1);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value (interpolation)");
  v = data(0, 2);
  cout << "value = " << v << endl;
  CHECK(v == 4, "wrong value (interpolation)");

  cout << endl;
  cout << "========================================" << endl;
  cout << "interpolation (middle)" << endl;
  cout << "========================================" << endl;
  data = g4d.query(7, "bbox:lid");
  v = data(0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value (interpolation)");
  v = data(0, 1);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value (interpolation)");
  v = data(0, 2);
  cout << "value = " << v << endl;
  CHECK(v == 4, "wrong value (interpolation)");

  cout << endl;
  cout << "========================================" << endl;
  cout << "interpolation (end)" << endl;
  cout << "========================================" << endl;
  data = g4d.query(9, "bbox:lid");
  v = data(0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value (interpolation)");
  v = data(0, 1);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value (interpolation)");
  v = data(0, 2);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value (interpolation)");

  cout << endl;
  cout << "========================================" << endl;
  cout << "missing" << endl;
  cout << "========================================" << endl;
  cout << g4d.getMissing() << endl;

  cout << "========================================" << endl;
  cout << "missingno" << endl;
  cout << "========================================" << endl;
  cout << g4d.getMissingNo() << endl;

  cout << "========================================" << endl;
  cout << "missingt" << endl;
  cout << "========================================" << endl;
  cout << g4d.getMissingF() << endl;
}

int main(int argc, char **argv) {

  G4Data g4d;
  loadData(g4d);
  testData(g4d);

  return 0;
}

