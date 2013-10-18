#include <Perception/g4data.h>

void loadData(G4Data &g4d) {
  MT::String fname = MT::getParameter<MT::String>("filename");
  g4d.loadData(fname);
}

void setObjIDs(G4Data &g4d) {
  g4d.addSensor("rh", "thumb", 0, 0);
  g4d.addSensor("rh", "index", 0, 1);
  g4d.addSensor("rh", "thumb", 0, 2);

  g4d.addSensor("lh", "thumb", 1, 0);
  g4d.addSensor("lh", "index", 1, 1);
  g4d.addSensor("lh", "thumb", 1, 2);

  g4d.addSensor("lh", "back", 2, 0);
  g4d.addSensor("rh", "back", 2, 1);

  g4d.addSensor("small_box", 3, 0);
  g4d.addSensor("bottle", 3, 1);
  g4d.addSensor("book", 3, 2);

  g4d.addSensor("ball", 4, 0);
  g4d.addSensor("big_box", "box", 4, 1);
  g4d.addSensor("big_box", "lid", 4, 2);
}

void testData(G4Data &g4d) {
  // T = number frames
  // N = number bodies
  // M = number sensors for a body

  cout << "frame 10" << endl;
  cout << g4d.query(10).getDim() << endl; // returns a 1xNx7 Array
  cout << "(should be 1, 15, 7)" << endl << endl;

  cout << "book" << endl;
  cout << g4d.query("book").getDim() << endl; // returns a TxMx7 Array
  cout << "(should be T, 1, 7)" << endl << endl;

  cout << "book @ t = 10" << endl;
  cout << g4d.query("book", 10).getDim() << endl; // returns a 1xMx7 Array
  cout << "(should be 1, 1, 7)" << endl << endl;

  cout << "right_hand" << endl;
  cout << g4d.query("right_hand").getDim() << endl; // returns a TxMx7 Array
  cout << "(should be T, 4, T)" << endl << endl;

  cout << "right_hand @ t = 10" << endl;
  cout << g4d.query("right_hand", 10).getDim() << endl; // returns 1xMx7 Array
  cout << "(should be 1, 4, 7)" << endl << endl;

  cout << "right_hand::thumb" << endl;
  cout << g4d.query("right_hand", "thumb").getDim() << endl; // returns a Tx1x7 Array
  cout << "(should be T, 1, 7)" << endl << endl;

  cout << "right_hand::thumb @ t = 10" << endl;
  cout << g4d.query("right_hand", "thumb", 10).getDim() << endl; // returns a 1x1x7 Array
  cout << "(should be 1, 1, 7)" << endl << endl;
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;
  loadData(g4d);
  setObjIDs(g4d);

  testData(g4d);

  return 0;
}

