#include "module_G4Recorder.h"

REGISTER_MODULE(G4Recorder)

G4Recorder::G4Recorder(): Module("G4Recorder") {
}

void G4Recorder::open() {
  file.open(STRING("z." << poses.name << '.' << MT::getNowString() << ".dat"));
}

void G4Recorder::close() {
  file.close();
}

void G4Recorder::step() {
  uint rev = poses.readAccess();
  floatA data = poses();
  double tstamp = poses.tstamp();
  poses.deAccess();

  MT::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", rev, tstamp);
  file << tag << ' ' << data << endl;
}
