#include "module_G4Recorder.h"

REGISTER_MODULE(G4Recorder)

G4Recorder::G4Recorder(): Module("G4Recorder") {
}

void G4Recorder::open() {
  file.open(STRING("z." << g4data.name << '.' << MT::getNowString() << ".dat"));
}

void G4Recorder::close() {
  file.close();
}

void G4Recorder::step() {
/*
  uint rev = g4data.readAccess();
  G4DataStruct g4d = g4data();
  floatA poses = g4d.poses;
  double time = g4d.timestamp.tv_sec + g4d.timestamp.tv_nsec/1000000000.;
  g4data.deAccess();

  MT::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", rev, time);
  file << tag << ' ' << poses << endl;
*/
  uint rev = g4data.readAccess();
  G4DataStruct g4d = g4data();
  g4data.deAccess();

  MT::String tag;
  tag.resize(10, false);
  sprintf(tag.p, "%6i", rev);
  file << tag << ' ' << g4d << endl;
}
