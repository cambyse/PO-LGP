#include <Perception/g4data.h>
#include <Core/array.h>

void loadData(G4Data &g4d) {
  MT::String meta = MT::getParameter<MT::String>("meta");
  MT::String poses = MT::getParameter<MT::String>("poses");

  g4d.loadData(meta, poses);
}

void testData(G4Data &g4d) {
  uint checkT           = 11;
  uint checkN           = 14;
  uint checkNrh         = 4;
  uint checkNrhthumb    = 1;
  uint checkNbook       = 1;
  uintA checkD10        = { 1,      checkN,        7 };
  uintA checkDbook      = { checkT, checkNbook,    7 };
  uintA checkD10book    = { 1,      checkNbook,    7 };
  uintA checkDrh        = { checkT, checkNrh,      7 };
  uintA checkD10rh      = { 1,      checkNrh,      7 };
  uintA checkDrhthumb   = { checkT, checkNrhthumb, 7 };
  uintA checkD10rhthumb = { 1,      checkNrhthumb, 7 };

  uint T, N, v;
  arr data;
  MT::Array<uint> dim;

  T = g4d.getNumTimesteps();
  cout << "T               = " << T << endl;
  CHECK(T == checkT, "wrong value of T");

  N = g4d.getNumSensors();
  cout << "N               = " << N << endl;
  CHECK(N == checkN, "wrong value of N");

  N = g4d.getNumSensors("rh");
  cout << "N rh            = " << N << endl;
  CHECK(N == checkNrh, "wrong value of N");

  N = g4d.getNumSensors("rh:thumb");
  cout << "N rh:thumb      = " << N << endl;
  CHECK(N == checkNrhthumb, "wrong value of N");

  N = g4d.getNumSensors("book");
  cout << "N book          = " << N << endl;
  CHECK(N == checkNbook, "wrong value of N");

  data = g4d.query(10);
  dim = data.getDim();
  cout << "dim 10          = " << dim << endl;
  CHECK(dim == checkD10, "wrong value of dim");

  data = g4d.query("book");
  dim = data.getDim();
  cout << "dim book        = " << dim << endl;
  CHECK(dim == checkDbook, "wrong value of dim");

  data = g4d.query(10, "book");
  dim = data.getDim();
  cout << "dim 10 book     = " << dim << endl;
  CHECK(dim == checkD10book, "wrong value of dim");

  data = g4d.query("rh");
  dim = data.getDim();
  cout << "dim rh          = " << dim << endl;
  CHECK(dim == checkDrh, "wrong value of dim");

  data = g4d.query(10, "rh");
  dim = data.getDim();
  cout << "dim 10 rh       = " << dim << endl;
  CHECK(dim == checkD10rh, "wrong value of dim");

  data = g4d.query("rh:thumb");
  dim = data.getDim();
  cout << "dim rh thumb    = " << dim << endl;
  CHECK(dim == checkDrhthumb, "wrong value of dim");

  data = g4d.query(10, "rh:thumb");
  dim = data.getDim();
  cout << "dim 10 rh thumb = " << dim << endl;
  CHECK(dim == checkD10rhthumb, "wrong value of dim");

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

  data = g4d.query(1);
  v = data(0, 0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");
  v = data(0, 1, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");
  v = data(0, 2, 0);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");

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

  data = g4d.query(0, "rh:thumb");
  v = data(0, 0, 0);
  cout << "value = " << v << endl;
  CHECK(v == 1, "wrong value");
  v = data(0, 0, 1);
  cout << "value = " << v << endl;
  CHECK(v == 2, "wrong value");
  v = data(0, 0, 2);
  cout << "value = " << v << endl;
  CHECK(v == 3, "wrong value");
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;
  loadData(g4d);
  testData(g4d);

  return 0;
}

